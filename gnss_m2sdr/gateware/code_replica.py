#
# This file is part of gnss-m2sdr.
#
# Code NCO + code ROM with configurable Early/Prompt/Late taps.
# SPDX-License-Identifier: BSD-2-Clause

"""Code NCO and Early/Prompt/Late code replica.

A fractional code-phase accumulator (``frac_bits`` chips) advances by
``code_step`` each enabled sample. Its integer chip index addresses a code ROM
(1023 chips, 0/1) whose output is mapped to +/-1 (bit 1 -> +1, bit 0 -> -1, the
GNSSSignals.jl polarity). Early/Late taps read the ROM at the neighbouring chip
selected by a runtime ``spacing`` (chips, fixed-point like ``frac_bits``), so
the correlator spacing is configurable as in Tracking.jl.

    code_step    = round(f_chip / fs * 2**frac_bits)   (f_chip = chip_rate*(1+doppler))
    spacing_word = round(spacing_chips * 2**frac_bits)  (0 < spacing < 1)
"""

from migen import *

from litex.gen import *

from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH


class CodeReplica(LiteXModule):
    """Code NCO + E/P/L code replica for one channel.

    Parameters
    ----------
    prn        : PRN used to initialise the code ROM (may be reloaded later).
    frac_bits  : fractional code-phase / spacing resolution.
    code_length: number of chips (1023 for GPS L1 C/A).

    Ports
    -----
    code_step : in  (frac_bits) - fractional chips advanced per enabled sample.
    spacing   : in  (frac_bits) - E/L half-spacing in chips (0..1), fixed-point.
    stb       : in  - advance one sample when high.
    restart   : in  - reset code phase and chip index to 0.
    early, prompt, late : out (2, signed) - replica chips (-1/+1) this sample.
    chip_index : out - current prompt chip index (0..code_length-1).
    code_frac  : out (frac_bits) - fractional code phase this sample.
    epoch      : out - high on the sample whose advance wraps the last chip -> 0.
    """
    def __init__(self, prn=1, frac_bits=24, code_length=CA_CODE_LENGTH):
        self.code_step  = Signal(frac_bits)
        self.spacing    = Signal(frac_bits)
        self.stb        = Signal()
        self.restart    = Signal()
        self.early      = Signal((2, True))
        self.prompt     = Signal((2, True))
        self.late       = Signal((2, True))
        self.chip_index = Signal(max=code_length)
        self.code_frac  = Signal(frac_bits)
        self.epoch      = Signal()

        # # #

        # Code ROM (0/1), 1 bit x code_length, three async read ports (E/P/L).
        init = ca_code_reference(prn)
        self.specials.mem = mem = Memory(1, code_length, init=init)
        p_e = mem.get_port(async_read=True)
        p_p = mem.get_port(async_read=True)
        p_l = mem.get_port(async_read=True)
        self.specials += p_e, p_p, p_l

        idx      = self.chip_index
        idx_next = Signal(max=code_length)  # idx + 1 (wrapped)
        idx_prev = Signal(max=code_length)  # idx - 1 (wrapped)
        self.comb += [
            If(idx == (code_length - 1), idx_next.eq(0)).Else(idx_next.eq(idx + 1)),
            If(idx == 0, idx_prev.eq(code_length - 1)).Else(idx_prev.eq(idx - 1)),
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
            # Map ROM bit {0,1} -> {-1,+1}.
            self.prompt.eq(Mux(p_p.dat_r, 1, -1)),
            self.early.eq( Mux(p_e.dat_r, 1, -1)),
            self.late.eq(  Mux(p_l.dat_r, 1, -1)),
        ]

        # Code NCO: fractional accumulator + chip index with mod-code_length wrap.
        acc_next = Signal(frac_bits + 1)
        self.comb += acc_next.eq(self.code_frac + self.code_step)
        self.sync += [
            self.epoch.eq(0),
            If(self.restart,
                self.code_frac.eq(0),
                self.chip_index.eq(0),
            ).Elif(self.stb,
                self.code_frac.eq(acc_next[:frac_bits]),
                If(acc_next[frac_bits],  # chip boundary crossed
                    If(idx == (code_length - 1),
                        self.chip_index.eq(0),
                        self.epoch.eq(1),
                    ).Else(
                        self.chip_index.eq(idx + 1),
                    ),
                ),
            ),
        ]
