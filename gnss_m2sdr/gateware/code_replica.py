#
# This file is part of gnss-m2sdr.
#
# Code NCO + code RAM with configurable taps and a sub-chip subcarrier.
# SPDX-License-Identifier: BSD-2-Clause

"""Code NCO and multi-tap code replica with sub-chip (BOC/CBOC/TMBOC) shaping.

A fractional code-phase accumulator (``frac_bits`` chips) advances by
``code_step`` each enabled sample. Its integer chip index addresses a code RAM
(``max_code_length`` chips) whose chip bit is mapped to +/-1 (bit 1 -> +1, bit 0
-> -1, the GNSSSignals.jl polarity). Each tap reads the replica at its own
signed offset from that phase, so a bank of taps at host-chosen positions --
Very Early, Early, Prompt, Late, Very Late -- comes out of one NCO.

    code_step = round(f_chip / fs * 2**frac_bits)   (f_chip = chip_rate*(1+doppler))
    tap_offset[t] = sample_shift[t] * code_step     (|offset| < 2**frac_bits)

The offsets are *whole numbers of input samples* times `code_step`, not raw chip
fractions: Tracking.jl quantises its preferred shifts the same way
(`calc_preferred_code_shift_to_sample_shift`) and its discriminators recover the
spacing from the correlator they are handed, so anything else mis-scales the DLL
loop gain. With five taps that is not a loop-gain nuisance but a correctness
requirement -- the VE/VL distance enters the discriminator separately and there
is no single number to re-derive it from, which is why GNSSReceiver's contract
hands over the whole `tap_sample_shifts` array and says "program exactly these".
The host does the quantisation (`GNSSChannel.tap_offset_word`).

Taps, and why they are cheap
----------------------------
Every tap offset is smaller than one chip (`MAX_TAP_OFFSET_CHIPS`), so however
many taps there are they only ever read chip index ``idx - 1``, ``idx`` or
``idx + 1``. The code RAM is therefore replicated **three** times -- one copy
per address, not one per tap -- and each tap muxes between the three outputs.
Five taps cost exactly what three did: ``3 x max_code_length`` words, still a
one-write/one-async-read distributed RAM. (The obvious alternative, a copy per
tap, would have been 5/3 of the memory for the same answer.)

The registered chip window
--------------------------
Those three words are held in *registers* (``w_prev``/``w_cur``/``w_next``), not
taken straight off the asynchronous RAM outputs. With an async read the
per-sample path ran ``code_length`` -> wrap arithmetic -> RAM address -> LUTRAM
and its output mux tree -> tap muxes -> correlator DSP: seventeen logic levels,
and at ``max_code_length = 10230`` the output mux alone is 160:1. That is what
missed setup by 2.875 ns on the first real five-tap build (PR #34). Registering
the window leaves only the tap muxes in front of the DSP and gives the RAM read
a clock cycle of its own.

The window is exact, not delayed. The index only ever moves one chip at a time,
so an advance *shifts* the three words and needs exactly one new one -- the chip
two ahead, which the "far" read port has been addressed at (``idx_far``) since
the previous advance. ``restart`` is the only jump, and the other two read ports
exist for it alone: they sit permanently on the chips either side of
``restart_chip``, so a rebase reloads all three words in the restart cycle
itself and the first sample after it already has the right replica.

What the window does defer by one chip, deliberately, is a *reconfiguration*
under a running index: a new ``code_length`` moves the wrap-around word
(``code[last]``, which an early tap reads at chip 0) and a ``code_load`` write
becomes visible only once the index has moved past it. Both are committed by the
arming ``restart``, which reloads the whole window, and the bank suppresses dumps
until then -- so no record can describe a half-applied shape either way.

Sub-chip modulation
-------------------
A BOC-family replica is the primary chip times a subcarrier that varies *inside*
the chip. GNSSSignals models the subcarrier as a function of the code phase
alone, and it is constant across each of ``P`` equal sub-chips, so:

    replica(phase) = chip(floor(phase)) * lut[floor(frac(phase) * P)]

``P`` (``subchips``) and the table are per channel and runtime-programmable;
see gnss_m2sdr/subcarrier.py for what to put in the table and why the index is
exact rather than a quantisation of the transition points. ``subchips = 1`` with
the reset table (+1) is plain BPSK, i.e. exactly the replica this module
produced before it had a subcarrier at all.

TMBOC alternates between two subcarriers according to the chip's position in a
33-chip pattern. Rather than run a counter that has to stay in step with the
code wrap and with every acquisition handover, the code RAM is **two** bits
wide: the chip, and a "use the other table" bit written alongside it. Any
pattern, any code length and any start chip then work by construction. The
second bit is only built when ``max_subchips > 1``; a LOC-only build keeps the
1-bit RAM it always had.

Runtime code length
-------------------
``max_code_length`` is a *build* parameter -- it sizes the code RAM and the chip
address, and no runtime write can make a block RAM deeper. ``code_length`` is a
*runtime* input: the number of chips actually in use, anywhere in
``1 .. max_code_length``, so one build tracks a 1023-chip GPS L1 C/A, a
4092-chip Galileo E1 and a 10230-chip GPS L1C code on different channels. The
wrap comparisons are all against ``code_length - 1``, and they use ``>=`` rather
than ``==``: a chip index left behind by a *shorter* new length would otherwise
run the accumulator all the way around the address space before wrapping again,
which is a lock lost for a whole code period instead of the one chip a rebase
costs. The channel still commits the length on ``restart`` (see bank.py), so
that path is a safety net, not the mechanism.

``restart`` rebases the accumulator onto ``(restart_chip, restart_frac)`` rather
than always onto 0, so an acquisition handover can start tracking at the code
phase the CPU measured. Both inputs default to 0, which is the plain
"restart at the beginning of the code" behaviour.

Cost
----
Code RAM is ``3 * word_bits * max_code_length`` bits per channel, where
``word_bits`` is 1 for a LOC-only build and 2 once the subcarrier select bit
exists -- independent of the tap count. See docs/signal_configuration.md for
the table.
"""

from migen import *

from litex.gen import *

from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH
from gnss_m2sdr.record_format import TAP_LAYOUTS, TAPS_EPL, tap_short_names


def replica_bits_for(max_subchips):
    """Default width of a replica sample for a build's sub-chip depth.

    A LOC-only build's replica is +/-1 and stays 2 bits signed, which is what it
    has always been -- so nothing about a GPS L1 C/A build changes. A build with
    a subcarrier has to carry CBOC's four levels (+/-25 and +/-13 at the
    amplitudes GNSSSignals' own table uses), so 8 bits.
    """
    return 2 if max_subchips <= 1 else 8


class CodeReplica(LiteXModule):
    """Code NCO + multi-tap code replica for one channel.

    Parameters
    ----------
    prn             : PRN used to initialise the code RAM (may be reloaded later).
    frac_bits       : fractional code-phase / tap-offset resolution.
    max_code_length : code RAM depth = longest primary code this build can hold.
    num_taps        : 3 (E/P/L) or 5 (VE/E/P/L/VL).
    max_subchips    : sub-chip table depth; 1 = no subcarrier (plain BPSK).
    replica_bits    : signed width of a replica sample (see replica_bits_for).

    Ports
    -----
    code_step   : in  (frac_bits) - fractional chips advanced per enabled sample.
    code_length : in - primary-code chips in use (1..max_code_length).
    subchips    : in - sub-chips per chip in force (1..max_subchips).
    tap_offset  : in  [num_taps] (frac_bits+1, signed) - each tap's offset in
                  chips, fixed-point. Positive is early (ahead of prompt).
                  The prompt tap's is tied to 0 by the bank.
    stb         : in  - advance one sample when high.
    restart     : in  - rebase code phase onto (restart_chip, restart_frac).
    restart_chip : in - chip index loaded by ``restart`` (0 = start of the code).
    restart_frac : in (frac_bits) - fractional chip phase loaded by ``restart``.
    replica     : out [num_taps] (replica_bits, signed) - this sample's replica
                  per tap, earliest first. ``early``/``prompt``/``late`` (and
                  ``very_early``/``very_late``) name the same signals.
    chip_index  : out - current prompt chip index (0..code_length-1).
    code_frac   : out (frac_bits) - fractional code phase this sample.
    epoch       : out - high on the sample whose advance wraps the last chip -> 0.
    replica_unsupported : out - the programmed replica shape cannot be evaluated
                  (see below); the channel suppresses dumps while it is high.
    """
    def __init__(self, prn=1, frac_bits=24, max_code_length=CA_CODE_LENGTH,
                 code_init=None, num_taps=TAPS_EPL, max_subchips=1,
                 replica_bits=None, staged_length=False):
        assert max_code_length >= 2, "a code needs at least two chips"
        assert max_subchips >= 1, "a chip has at least one sub-chip"
        self.max_code_length = max_code_length
        self.num_taps        = num_taps
        self.max_subchips    = max_subchips
        self.replica_bits    = (replica_bits_for(max_subchips)
                                if replica_bits is None else replica_bits)
        replica_bits = self.replica_bits
        # Named taps for the layouts the wire format defines; a bare index
        # otherwise. The 3/5 restriction belongs to the record, not to the NCO,
        # so this module stays a plain N-tap replica.
        tap_names = (tap_short_names(num_taps) if num_taps in TAP_LAYOUTS
                     else tuple(f"t{t}" for t in range(num_taps)))
        self.tap_names = tap_names

        # 1 bit of chip, plus the TMBOC "other subcarrier" bit once there is a
        # subcarrier at all. A LOC-only build keeps its 1-bit code RAM.
        word_bits  = 1 if max_subchips <= 1 else 2
        # Length the build powers up with, and the depth the registered code
        # window's reset values are taken at.
        code_len_reset = min(CA_CODE_LENGTH, max_code_length)
        self.word_bits = word_bits
        sub_bits   = bits_for(max_subchips)
        idx_bits   = bits_for(max(1, max_subchips - 1))

        self.code_step  = Signal(frac_bits)
        # Runtime primary-code length. Reset to the built-in code's length so a
        # build that is never configured behaves exactly as it did when the
        # length was a constant.
        self.code_length = Signal(bits_for(max_code_length),
                                  reset=code_len_reset)
        # Sub-chips per chip in force. Reset 1 = no subcarrier.
        self.subchips   = Signal(sub_bits, reset=1)
        # One signed offset per tap, earliest first. Prompt's is tied to 0.
        self.tap_offset = [Signal((frac_bits + 1, True)) for _ in range(num_taps)]
        self.stb        = Signal()
        self.restart    = Signal()
        # Phase loaded by `restart`. Both default to 0, i.e. plain "restart at
        # the start of the code"; the host sets them to hand a CPU-acquired code
        # phase over, so the first integration starts on the measured phase
        # instead of chip 0.
        self.restart_chip = Signal(max=max_code_length)
        self.restart_frac = Signal(frac_bits)
        self.replica    = [Signal((replica_bits, True)) for _ in range(num_taps)]
        alias = {"ve": "very_early", "e": "early", "p": "prompt",
                 "l": "late", "vl": "very_late"}
        for name, sig in zip(tap_names, self.replica):
            if name in alias:
                setattr(self, alias[name], sig)
        self.chip_index = Signal(max=max_code_length)
        self.code_frac  = Signal(frac_bits)
        self.epoch      = Signal()
        # "The replica shape in force cannot be evaluated." Two ways to get
        # there, both of which would otherwise produce a plausible-looking wrong
        # replica rather than an error: a sub-chip count outside the table, and
        # a tap offset of a whole chip or more (the taps only reach idx +/- 1,
        # so a larger offset silently lands on the wrong chip).
        self.replica_unsupported = Signal()

        # The code length a `restart` wraps with, i.e. the length in force for the
        # window it loads. On a bank this is the *staged* CSR -- the value the
        # restart is about to commit -- driven from outside (`staged_length`),
        # because it is known a cycle early and the code-RAM read address must
        # not wait on `restart` -> the commit mux -> `code_length - 1`. That
        # chain measured 2.9 ns of a path that already had the LUTRAM read on
        # the end of it. Standalone it simply follows `code_length`.
        self.restart_length = Signal(bits_for(max_code_length),
                                     reset=code_len_reset)
        if not staged_length:
            self.comb += self.restart_length.eq(self.code_length)

        # Runtime code-load port (host writes the acquired PRN's code here).
        self.load_we  = Signal()
        self.load_adr = Signal(max=max_code_length)
        self.load_dat = Signal()          # chip: 1 -> +1, 0 -> -1
        self.load_sub = Signal()          # TMBOC: use the second subcarrier table
        # Runtime subcarrier-table load port.
        self.lut_we   = Signal()
        self.lut_sel  = Signal()          # 0 = table A, 1 = table B (TMBOC)
        self.lut_adr  = Signal(idx_bits)
        self.lut_dat  = Signal((replica_bits, True))

        # # #

        # Code RAM, `word_bits` x max_code_length, replicated three times -- one
        # copy per *address* (idx-1, idx, idx+1), not one per tap, since no tap
        # reaches further than a chip. Each is a simple 1-write + 1-async-read
        # distributed RAM (clean LUTRAM template) and all three share the host
        # load port. Added via `specials +=` (not named attributes) so AutoCSR
        # does not CSR-map them. Initialised with `code_init`, or with `prn`'s
        # C/A code (zero-padded past 1023), so sim/power-on work without a load.
        if code_init is None:
            # The built-in C/A code is a *default*, not a request: it is clipped
            # to whatever depth the build has (a bank sized for an 8-chip test
            # code is still legal), and zero-padded past 1023.
            init = list(ca_code_reference(prn))[:max_code_length]
        else:
            init = list(code_init)
            assert len(init) <= max_code_length, (
                f"code_init has {len(init)} chips, more than max_code_length "
                f"{max_code_length}")
        init += [0] * (max_code_length - len(init))
        # `code_init` may carry the subcarrier-select bit in bit 1 alongside the
        # chip in bit 0 (the same layout the load port writes). A plain list of
        # 0/1 chips therefore selects table A everywhere, which is what every
        # modulation but TMBOC wants.
        init = [w & ((1 << word_bits) - 1) for w in init]

        load_word = Signal(word_bits)
        self.comb += load_word.eq(self.load_dat if word_bits == 1
                                  else Cat(self.load_dat, self.load_sub))

        def make_port(adr):
            m  = Memory(word_bits, max_code_length, init=init)
            rp = m.get_port(async_read=True)
            wp = m.get_port(write_capable=True)
            self.specials += m, rp, wp
            self.comb += [
                wp.adr.eq(self.load_adr),
                wp.dat_w.eq(load_word),
                wp.we.eq(self.load_we),
                rp.adr.eq(adr),
            ]
            return rp

        # Last valid chip index for the length currently programmed.
        last     = Signal(max=max_code_length)
        at_last  = Signal()
        idx      = self.chip_index
        idx_next = Signal(max=max_code_length)  # idx + 1 (wrapped)
        self.comb += [
            last.eq(self.code_length - 1),
            # `>=`, not `==`: shortening code_length under a running index must
            # wrap on the next chip rather than after a lap of the RAM.
            at_last.eq(idx >= last),
            If(at_last, idx_next.eq(0)).Else(idx_next.eq(idx + 1)),
        ]

        # The three chips around `restart_chip`, and the one after those, for a
        # rebase. Wrapping uses the same `>=` rule as the running index, so a
        # handover phase left over from a longer code lands where the running
        # index would have.
        r_last = Signal(max=max_code_length)   # last chip index a restart wraps at
        r_prev = Signal(max=max_code_length)   # restart_chip - 1 (wrapped)
        r_next = Signal(max=max_code_length)   # restart_chip + 1 (wrapped)
        r_far  = Signal(max=max_code_length)   # restart_chip + 2 (wrapped)
        self.comb += [
            r_last.eq(self.restart_length - 1),
            If(self.restart_chip == 0, r_prev.eq(r_last),
            ).Else(r_prev.eq(self.restart_chip - 1)),
            If(self.restart_chip >= r_last, r_next.eq(0),
            ).Else(r_next.eq(self.restart_chip + 1)),
            If(r_next >= r_last, r_far.eq(0)).Else(r_far.eq(r_next + 1)),
        ]

        # The three chip words the taps read -- code[idx-1], code[idx],
        # code[idx+1] -- held in REGISTERS rather than read out of the RAM
        # combinationally. This is the timing fix: with an asynchronous read the
        # per-sample path ran
        #
        #   code_length -> last -> idx+/-1 -> RAM address -> LUTRAM + output mux
        #   tree -> tap word mux -> subcarrier amplitude -> correlator DSP,
        #
        # seventeen logic levels deep, and at max_code_length = 10230 the LUTRAM
        # output mux alone is a 160:1 tree. Registering the window takes the RAM
        # *and* `code_length` out of that path entirely and leaves only the tap
        # muxes in front of the DSP; the RAM read becomes an ordinary
        # register-to-register hop with a whole clock cycle to itself.
        #
        # The window is exact, not delayed, because the index only ever moves by
        # one chip: on an advance the three words shift by one and the single
        # new word (the chip *two* ahead, addressed by `idx_far`) has been
        # waiting at the far read port since the previous advance. `restart` is
        # the only jump, and that is what the other two read ports are for --
        # they sit permanently on the chips either side of `restart_chip`, so a
        # rebase reloads all three words in the restart cycle itself, with no
        # dead sample after it.
        far_adr = Signal(max=max_code_length)
        p_prev = make_port(r_prev)          # code[restart_chip - 1]
        p_cur  = make_port(self.restart_chip)
        p_next = make_port(far_adr)         # code[idx + 2], or code[restart_chip + 1]

        w_prev = Signal(word_bits, reset=init[code_len_reset - 1])
        w_cur  = Signal(word_bits, reset=init[0])
        w_next = Signal(word_bits, reset=init[1])
        # Address the far port reads while the code runs: two chips ahead, i.e.
        # the word the window needs after the next advance.
        idx_far = Signal(max=max_code_length, reset=2 % code_len_reset)
        self.comb += far_adr.eq(Mux(self.restart, r_next, idx_far))

        # Code NCO: fractional accumulator + chip index with mod-code_length wrap.
        acc_next = Signal(frac_bits + 1)
        self.comb += acc_next.eq(self.code_frac + self.code_step)
        # epoch is COMBINATIONAL and aligned with the wrapping strobe (the sample
        # that completes the last chip -> 0). It must coincide with `stb` because
        # consumers sample it on `stb` cycles, and `stb` is sparse on hardware
        # (one pulse every fs/sys_clk cycles) -- a registered epoch would land
        # on a non-stb cycle and be missed.
        self.comb += self.epoch.eq(
            self.stb & ~self.restart & acc_next[frac_bits] & at_last)

        # The advance that moves the chip window on, named once: `chip_index`,
        # the window and `idx_far` all move together on it.
        chip_adv = Signal()
        self.comb += chip_adv.eq(self.stb & acc_next[frac_bits])

        # The state one clock from now. Everything the taps look at --
        # the fractional phase and the three-chip window -- is a register, so
        # what each tap reads can be worked out a cycle in advance and
        # registered too. That is what the per-tap logic below does, and it is
        # why none of this arithmetic sits in front of the correlator's DSP.
        # `nf` deliberately does NOT depend on `stb`. Everything downstream of it
        # is the deep part -- a 26-bit phase add and the sub-chip multiply -- and
        # `stb` arrives late (it is the RX strobe, gated by `control.enable`,
        # routed across the bank), so putting it in front of that chain cost
        # 1.733 ns. Instead `stb` gates the *registers* at the far end: when no
        # sample is strobed they simply hold, which is the same answer because
        # `code_frac` holds too.
        nf = Signal(frac_bits)                     # code_frac after the next load
        nw_prev = Signal(word_bits)                # ... and the window
        nw_cur  = Signal(word_bits)
        nw_next = Signal(word_bits)
        self.comb += [
            If(self.restart,
                nf.eq(self.restart_frac),
                nw_prev.eq(p_prev.dat_r),
                nw_cur.eq(p_cur.dat_r),
                nw_next.eq(p_next.dat_r),
            ).Else(
                nf.eq(acc_next[:frac_bits]),
                If(chip_adv,
                    nw_prev.eq(w_cur), nw_cur.eq(w_next), nw_next.eq(p_next.dat_r),
                ).Else(
                    nw_prev.eq(w_prev), nw_cur.eq(w_cur), nw_next.eq(w_next),
                ),
            ),
        ]

        # Subcarrier tables. Registers rather than a RAM: they are at most
        # `max_subchips` entries and every tap reads a *different* index in the
        # same cycle, so a memory would need one port per tap.
        if max_subchips > 1:
            lut_a = Array(Signal((replica_bits, True), reset=1)
                          for _ in range(max_subchips))
            lut_b = Array(Signal((replica_bits, True), reset=1)
                          for _ in range(max_subchips))
            self.sync += If(self.lut_we,
                *[If(self.lut_adr == i,
                     If(self.lut_sel, lut_b[i].eq(self.lut_dat))
                     .Else(lut_a[i].eq(self.lut_dat)))
                  for i in range(max_subchips)],
            )

        # Per-tap replica: pick the chip the offset lands on, then shape it with
        # the sub-chip the offset's fraction lands in.
        #
        # Both choices -- *which* chip word (`word_r`) and *which* sub-chip of it
        # (`k_r`) -- are REGISTERS, computed from the phase and window the taps
        # will see next cycle rather than the ones they see now. Only the two
        # amplitude muxes are left in front of the correlator DSP.
        #
        # This is the second half of the timing fix (see the registered chip
        # window above). `floor(sub * subchips)` is a 24x4 multiply whose *top*
        # bits are wanted, so every carry in it is on the path; combinationally
        # it put the fractional-phase adder, that multiply, the subcarrier table
        # mux and the sign mux between `code_frac` and a DSP48E1 `B` pin --
        # 8.493 ns of a 8.000 ns cycle. Computing it one cycle ahead costs no
        # latency, because `code_frac` and the window are registers whose next
        # value is already known this cycle.
        #
        # The one visible difference: a `tap_offset` or `subchips` write reaches
        # the replica on the next strobe or `restart` rather than on the next
        # cycle. Both are programmed before the arming `restart`, which is itself
        # a cycle the registers below load on, so the first sample of an
        # integration already has them.
        for t in range(num_taps):
            nphase = Signal((frac_bits + 2, True))  # next code_frac + offset
            nadv   = Signal()                       # lands on the next chip
            nret   = Signal()                       # lands on the previous chip
            nsub   = Signal(frac_bits)              # fractional phase within it
            nword  = Signal(word_bits)
            word_r = Signal(word_bits, reset=init[0])
            self.comb += [
                nphase.eq(nf + self.tap_offset[t]),
                nadv.eq(nphase >= (1 << frac_bits)),
                nret.eq(nphase < 0),
                If(nadv,
                    nsub.eq(nphase - (1 << frac_bits)), nword.eq(nw_next),
                ).Elif(nret,
                    nsub.eq(nphase + (1 << frac_bits)), nword.eq(nw_prev),
                ).Else(
                    nsub.eq(nphase), nword.eq(nw_cur),
                ),
            ]
            self.sync += If(self.restart | self.stb, word_r.eq(nword))
            if max_subchips <= 1:
                # No subcarrier: the replica is the chip, as it always was.
                self.comb += self.replica[t].eq(Mux(word_r[0], 1, -1))
            else:
                # floor(sub * subchips): the sub-chip the tap sits in. Computed
                # from the *whole* fraction, not from its top bits, because the
                # transitions of a P = 12 subcarrier are not on a dyadic grid
                # and rounding them would put the replica one sub-chip out at
                # some phases -- a plausible wrong answer, not an error.
                nprod = Signal(frac_bits + sub_bits)
                k_r   = Signal(idx_bits)
                amp   = Signal((replica_bits, True))
                self.comb += nprod.eq(nsub * self.subchips)
                self.sync += If(self.restart | self.stb, k_r.eq(nprod[frac_bits:]))
                self.comb += [
                    amp.eq(Mux(word_r[1], lut_b[k_r], lut_a[k_r])),
                    self.replica[t].eq(Mux(word_r[0], amp, -amp)),
                ]

        # An unevaluable replica shape. `subchips` has to index the table, and a
        # tap offset of exactly -1 chip (the one out-of-range value the signed
        # register can hold) would wrap onto idx-1 with a zero fraction instead
        # of reaching idx-2.
        min_offset = -(1 << frac_bits)
        bad = [self.subchips == 0, self.subchips > max_subchips]
        bad += [self.tap_offset[t] == min_offset for t in range(num_taps)]
        self.comb += self.replica_unsupported.eq(Cat(*bad) != 0)

        self.sync += [
            If(self.restart,
                self.code_frac.eq(self.restart_frac),
                self.chip_index.eq(self.restart_chip),
            ).Elif(self.stb,
                self.code_frac.eq(acc_next[:frac_bits]),
                If(chip_adv,  # chip boundary crossed
                    self.chip_index.eq(idx_next),
                ),
            ),
        ]

        # The registered code window, moved in lockstep with `chip_index` above
        # so that (w_prev, w_cur, w_next) is always exactly
        # (code[idx-1], code[idx], code[idx+1]) for the index the taps see this
        # cycle -- no sample is ever read one chip late.
        #
        # A rebase reloads all three from the ports parked around `restart_chip`;
        # an advance shifts them and pulls in the one genuinely new word from the
        # far port, which has been addressed at `idx_far` (two chips ahead) since
        # the previous advance.
        #
        # Two consequences, both of which the bank already arranges around: a
        # `code_length` change moves the wrap-around word (code[last], seen by an
        # early tap at chip 0) only from the next advance, and a `code_load` write
        # is only visible once the index has moved past it. Both are committed by
        # the arming `restart`, which reloads the whole window, and dumps are
        # suppressed until then -- see bank.py.
        self.sync += [
            If(self.restart | chip_adv,
                w_prev.eq(nw_prev),
                w_cur.eq(nw_cur),
                w_next.eq(nw_next),
            ),
            If(self.restart,
                idx_far.eq(r_far),
            ).Elif(chip_adv,
                If(idx_far >= last, idx_far.eq(0)).Else(idx_far.eq(idx_far + 1)),
            ),
        ]
