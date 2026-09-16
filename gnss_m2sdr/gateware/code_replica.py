#
# This file is part of gnss-m2sdr.
#
# Code NCO + code ROM with configurable Early/Prompt/Late taps.
# SPDX-License-Identifier: BSD-2-Clause

"""Code NCO and Early/Prompt/Late code replica.

A fractional code-phase accumulator (``frac_bits`` chips) advances by
``code_step`` each enabled sample. Its integer chip index addresses a code RAM
(``max_code_length`` chips, 0/1) whose output is mapped to +/-1 (bit 1 -> +1,
bit 0 -> -1, the GNSSSignals.jl polarity). Early/Late taps read the RAM at the
neighbouring chip selected by a runtime ``spacing`` (chips, fixed-point like
``frac_bits``), so the correlator spacing is configurable as in Tracking.jl.

    code_step    = round(f_chip / fs * 2**frac_bits)   (f_chip = chip_rate*(1+doppler))
    spacing_word = sample_shift * code_step             (0 < spacing_word < 2**frac_bits)

The spacing is a *whole number of input samples* times `code_step`, not a raw
chip fraction: Tracking.jl quantises the preferred chip shift the same way
(`calc_preferred_code_shift_to_sample_shift`) and `dll_disc` normalises with the
quantised spacing, so anything else mis-scales the DLL loop gain. The host does
that quantisation (`GNSSChannel.spacing_word`); it also enforces
`spacing_word < 2**frac_bits`, since the E/L taps only reach `idx +/- 1`.

``restart`` rebases the accumulator onto ``(restart_chip, restart_frac)`` rather
than always onto 0, so an acquisition handover can start tracking at the code
phase the CPU measured. Both inputs default to 0, which is the plain
"restart at the beginning of the code" behaviour.

Runtime code length
-------------------
``max_code_length`` is a *build* parameter -- it sizes the code RAM and the chip
address, and no runtime write can make a block RAM deeper. ``code_length`` is a
*runtime* input: the number of chips actually in use, anywhere in
``1 .. max_code_length``, so one build tracks a 1023-chip GPS L1 C/A, a
4092-chip Galileo E1 and a 10230-chip GPS L5 code on different channels. The
wrap comparisons are all against ``code_length - 1``, and they use ``>=`` rather
than ``==``: a chip index left behind by a *shorter* new length would otherwise
run the accumulator all the way around the address space before wrapping again,
which is a lock lost for a whole code period instead of the one chip a rebase
costs. The channel still commits the length on ``restart`` (see bank.py), so
that path is a safety net, not the mechanism.

The code RAM is one 1-bit x ``max_code_length`` array per tap. Depth is the
build's real cost: at 3 taps that is ``3 * max_code_length`` bits per channel,
so a 4-channel bank costs 12 kbit at 1023 chips (distributed RAM territory) and
123 kbit at 10230 (block RAM, ~2 RAMB18 per channel on Artix-7). See
docs/signal_configuration.md for the table.
"""

from migen import *

from litex.gen import *

from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH


class CodeReplica(LiteXModule):
    """Code NCO + E/P/L code replica for one channel.

    Parameters
    ----------
    prn             : PRN used to initialise the code RAM (may be reloaded later).
    frac_bits       : fractional code-phase / spacing resolution.
    max_code_length : code RAM depth = longest primary code this build can hold.

    Ports
    -----
    code_step : in  (frac_bits) - fractional chips advanced per enabled sample.
    code_length : in - primary-code chips in use (1..max_code_length).
    spacing   : in  (frac_bits) - E/L half-spacing in chips (0..1), fixed-point.
    stb       : in  - advance one sample when high.
    restart   : in  - rebase code phase onto (restart_chip, restart_frac).
    restart_chip : in - chip index loaded by ``restart`` (0 = start of code).
    restart_frac : in (frac_bits) - fractional chip phase loaded by ``restart``.
    early, prompt, late : out (2, signed) - replica chips (-1/+1) this sample.
    chip_index : out - current prompt chip index (0..code_length-1).
    code_frac  : out (frac_bits) - fractional code phase this sample.
    epoch      : out - high on the sample whose advance wraps the last chip -> 0.
    """
    def __init__(self, prn=1, frac_bits=24, max_code_length=CA_CODE_LENGTH,
                 code_init=None):
        assert max_code_length >= 2, "a code needs at least two chips"
        self.max_code_length = max_code_length

        self.code_step  = Signal(frac_bits)
        # Runtime primary-code length. Reset to the built-in code's length so a
        # build that is never configured behaves exactly as it did when the
        # length was a constant.
        self.code_length = Signal(bits_for(max_code_length),
                                  reset=min(CA_CODE_LENGTH, max_code_length))
        self.spacing    = Signal(frac_bits)
        self.stb        = Signal()
        self.restart    = Signal()
        # Phase loaded by `restart`. Both default to 0, i.e. plain "restart at
        # the start of the code"; the host sets them to hand a CPU-acquired code
        # phase over, so the first integration starts on the measured phase
        # instead of chip 0.
        self.restart_chip = Signal(max=max_code_length)
        self.restart_frac = Signal(frac_bits)
        self.early      = Signal((2, True))
        self.prompt     = Signal((2, True))
        self.late       = Signal((2, True))
        self.chip_index = Signal(max=max_code_length)
        self.code_frac  = Signal(frac_bits)
        self.epoch      = Signal()

        # Runtime code-load port (host writes the acquired PRN's code here).
        self.load_we  = Signal()
        self.load_adr = Signal(max=max_code_length)
        self.load_dat = Signal()

        # # #

        # Code RAM (0/1), 1 bit x max_code_length. Replicated 3x (Early/Prompt/
        # Late) so each RAM is a simple 1-write + 1-async-read distributed RAM
        # (clean LUTRAM template); all three share the host load port. Added via
        # `specials +=` (not named attributes) so AutoCSR does not CSR-map them.
        # Initialised with `code_init`, or with `prn`'s C/A code (zero-padded
        # past 1023), so sim/power-on work without an explicit load.
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

        def make_replica():
            m  = Memory(1, max_code_length, init=init)
            rp = m.get_port(async_read=True)
            wp = m.get_port(write_capable=True)
            self.specials += m, rp, wp
            self.comb += [
                wp.adr.eq(self.load_adr),
                wp.dat_w.eq(self.load_dat),
                wp.we.eq(self.load_we),
            ]
            return rp

        p_e = make_replica()
        p_p = make_replica()
        p_l = make_replica()

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

        # Early leads prompt by `spacing`, Late trails by `spacing`.
        #   early_phase = frac + spacing  -> next chip when it reaches >= 1
        #   late_phase  = frac - spacing  -> previous chip when frac < spacing
        early_adv = Signal()
        late_ret  = Signal()
        self.comb += [
            early_adv.eq((self.code_frac + self.spacing) >= (1 << frac_bits)),
            late_ret.eq(self.code_frac < self.spacing),
            p_p.adr.eq(idx),
            p_e.adr.eq(Mux(early_adv, idx_next, idx)),
            p_l.adr.eq(Mux(late_ret,  idx_prev, idx)),
            # Map RAM bit {0,1} -> {-1,+1}.
            self.prompt.eq(Mux(p_p.dat_r, 1, -1)),
            self.early.eq( Mux(p_e.dat_r, 1, -1)),
            self.late.eq(  Mux(p_l.dat_r, 1, -1)),
        ]

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
