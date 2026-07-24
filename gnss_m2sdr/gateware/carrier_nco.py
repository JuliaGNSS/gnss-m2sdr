#
# This file is part of gnss-m2sdr.
#
# Carrier NCO: phase accumulator + sin/cos lookup (carrier replica).
# SPDX-License-Identifier: BSD-2-Clause

"""Carrier NCO producing a quantized cos/sin replica for carrier wipe-off.

A ``phase_bits``-wide phase accumulator advances by ``freq_word`` each enabled
sample; the top ``lut_addr_bits`` index sin/cos ROMs quantized to ``amp_bits``
signed bits (peak ``2**(amp_bits-1)-1``). Amplitude matches the JuliaGNSS
SinCosLUT.jl convention (Int8, peak 127) by default.

freq_word = round(f / fs * 2**phase_bits), where f is the residual carrier
(IF + Doppler) and fs the sample rate.
"""

import math

from migen import *

from litex.gen import *


def _sincos_tables(addr_bits, amp_bits):
    n    = 1 << addr_bits
    peak = (1 << (amp_bits - 1)) - 1
    sin_t, cos_t = [], []
    for i in range(n):
        ang = 2 * math.pi * i / n
        sin_t.append(int(round(peak * math.sin(ang))))
        cos_t.append(int(round(peak * math.cos(ang))))
    return sin_t, cos_t


class CarrierNCO(LiteXModule):
    """Carrier NCO / replica generator.

    Parameters
    ----------
    phase_bits    : phase accumulator width.
    lut_addr_bits : number of phase MSBs used to index the sin/cos ROMs.
    amp_bits      : signed replica sample width (peak = 2**(amp_bits-1)-1).

    Ports
    -----
    freq_word : in  (phase_bits) - phase increment per enabled sample.
    stb       : in  - advance one sample when high.
    set_phase : in  - load ``phase_in`` into the accumulator (one cycle).
    phase_in  : in  (phase_bits) - new phase for ``set_phase``.
    cos, sin  : out (amp_bits, signed) - carrier replica for the current sample.
    phase     : out (phase_bits) - current accumulator value.
    """
    def __init__(self, phase_bits=32, lut_addr_bits=8, amp_bits=8):
        self.freq_word = Signal(phase_bits)
        self.stb       = Signal()
        self.set_phase = Signal()
        self.phase_in  = Signal(phase_bits)
        self.cos       = Signal((amp_bits, True))
        self.sin       = Signal((amp_bits, True))
        self.phase     = Signal(phase_bits)

        # # #

        sin_t, cos_t = _sincos_tables(lut_addr_bits, amp_bits)
        self.specials.sin_mem = sin_mem = Memory(amp_bits, 1 << lut_addr_bits, init=sin_t)
        self.specials.cos_mem = cos_mem = Memory(amp_bits, 1 << lut_addr_bits, init=cos_t)
        sin_rd = sin_mem.get_port(async_read=True)
        cos_rd = cos_mem.get_port(async_read=True)
        self.specials += sin_rd, cos_rd

        addr = Signal(lut_addr_bits)
        self.comb += [
            addr.eq(self.phase[phase_bits - lut_addr_bits:]),
            sin_rd.adr.eq(addr),
            cos_rd.adr.eq(addr),
            self.sin.eq(sin_rd.dat_r),
            self.cos.eq(cos_rd.dat_r),
        ]

        self.sync += [
            If(self.set_phase,
                self.phase.eq(self.phase_in),
            ).Elif(self.stb,
                self.phase.eq(self.phase + self.freq_word),
            ),
        ]
