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
                 replica_bits=None):
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
        self.word_bits = word_bits
        sub_bits   = bits_for(max_subchips)
        idx_bits   = bits_for(max(1, max_subchips - 1))

        self.code_step  = Signal(frac_bits)
        # Runtime primary-code length. Reset to the built-in code's length so a
        # build that is never configured behaves exactly as it did when the
        # length was a constant.
        self.code_length = Signal(bits_for(max_code_length),
                                  reset=min(CA_CODE_LENGTH, max_code_length))
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
        idx_prev = Signal(max=max_code_length)  # idx - 1 (wrapped)
        self.comb += [
            last.eq(self.code_length - 1),
            # `>=`, not `==`: shortening code_length under a running index must
            # wrap on the next chip rather than after a lap of the RAM.
            at_last.eq(idx >= last),
            If(at_last, idx_next.eq(0)).Else(idx_next.eq(idx + 1)),
            If(idx == 0, idx_prev.eq(last)).Else(idx_prev.eq(idx - 1)),
        ]

        p_prev = make_port(idx_prev)
        p_cur  = make_port(idx)
        p_next = make_port(idx_next)

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
        for t in range(num_taps):
            phase = Signal((frac_bits + 2, True))   # code_frac + offset, signed
            adv   = Signal()                        # landed on the next chip
            ret   = Signal()                        # landed on the previous chip
            sub   = Signal(frac_bits)               # fractional phase within it
            word  = Signal(word_bits)
            self.comb += [
                phase.eq(self.code_frac + self.tap_offset[t]),
                adv.eq(phase >= (1 << frac_bits)),
                ret.eq(phase < 0),
                If(adv,
                    sub.eq(phase - (1 << frac_bits)), word.eq(p_next.dat_r),
                ).Elif(ret,
                    sub.eq(phase + (1 << frac_bits)), word.eq(p_prev.dat_r),
                ).Else(
                    sub.eq(phase), word.eq(p_cur.dat_r),
                ),
            ]
            if max_subchips <= 1:
                # No subcarrier: the replica is the chip, as it always was.
                self.comb += self.replica[t].eq(Mux(word[0], 1, -1))
            else:
                # floor(sub * subchips): the sub-chip the tap sits in. Computed
                # from the *whole* fraction, not from its top bits, because the
                # transitions of a P = 12 subcarrier are not on a dyadic grid
                # and rounding them would put the replica one sub-chip out at
                # some phases -- a plausible wrong answer, not an error.
                prod = Signal(frac_bits + sub_bits)
                k    = Signal(idx_bits)
                amp  = Signal((replica_bits, True))
                self.comb += [
                    prod.eq(sub * self.subchips),
                    k.eq(prod[frac_bits:]),
                    amp.eq(Mux(word[1], lut_b[k], lut_a[k])),
                    self.replica[t].eq(Mux(word[0], amp, -amp)),
                ]

        # An unevaluable replica shape. `subchips` has to index the table, and a
        # tap offset of exactly -1 chip (the one out-of-range value the signed
        # register can hold) would wrap onto idx-1 with a zero fraction instead
        # of reaching idx-2.
        min_offset = -(1 << frac_bits)
        bad = [self.subchips == 0, self.subchips > max_subchips]
        bad += [self.tap_offset[t] == min_offset for t in range(num_taps)]
        self.comb += self.replica_unsupported.eq(Cat(*bad) != 0)

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
        self.sync += [
            If(self.restart,
                self.code_frac.eq(self.restart_frac),
                self.chip_index.eq(self.restart_chip),
            ).Elif(self.stb,
                self.code_frac.eq(acc_next[:frac_bits]),
                If(acc_next[frac_bits],  # chip boundary crossed
                    self.chip_index.eq(idx_next),
                ),
            ),
        ]
