#
# This file is part of gnss-m2sdr.
#
# GPS L1 C/A (Gold) code generator.
# SPDX-License-Identifier: BSD-2-Clause

"""GPS L1 C/A code generation (IS-GPS-200).

Two 10-stage LFSRs (G1, G2) generate a length-1023 Gold code. G1 uses feedback
taps [3, 10]; G2 uses [2, 3, 6, 8, 9, 10]. Per-PRN code phase is selected by
XORing two G2 stages (the phase-selector table below). Both registers start at
all-ones and advance one chip per ``shift`` strobe, wrapping every 1023 chips.
"""

from migen import *

from litex.gen import *

# Pure-Python C/A reference + PRN table live in gps_ca (no migen), so host
# tooling can import them without the gateware dependencies. Re-exported here
# for backward compatibility.
from gnss_m2sdr.gps_ca import (
    CA_PHASE_SELECT, CA_CODE_LENGTH, ca_code_reference, ca_first10_octal,
)


# Migen module. ------------------------------------------------------------------------------------

class CACodeGenerator(LiteXModule):
    """Length-1023 GPS L1 C/A code generator.

    Parameters
    ----------
    prn : int
        Satellite PRN (1..32) selecting the G2 phase-selector taps.

    Ports
    -----
    shift   : in  - advance one chip when high (one sys cycle).
    restart : in  - reset both LFSRs to all-ones (chip index -> 0).
    chip    : out - current (prompt) code chip, 0/1.
    index   : out - current chip index 0..1022.
    epoch   : out - high on the cycle whose ``shift`` completes chip 1022->0 wrap.
    """
    def __init__(self, prn=1):
        assert prn in CA_PHASE_SELECT, f"unsupported PRN {prn}"
        s1, s2 = CA_PHASE_SELECT[prn]

        self.shift   = Signal()
        self.restart = Signal()
        self.chip    = Signal()
        self.index   = Signal(max=CA_CODE_LENGTH)
        self.epoch   = Signal()

        # # #

        g1 = Signal(10, reset=0x3FF)  # g1[i] == stage (i+1)
        g2 = Signal(10, reset=0x3FF)

        g2_out = Signal()
        self.comb += [
            g2_out.eq(g2[s1 - 1] ^ g2[s2 - 1]),
            self.chip.eq(g1[9] ^ g2_out),
        ]

        fb1 = Signal()
        fb2 = Signal()
        self.comb += [
            fb1.eq(g1[2] ^ g1[9]),                                  # taps 3,10
            fb2.eq(g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]),  # taps 2,3,6,8,9,10
        ]

        self.sync += [
            self.epoch.eq(0),
            If(self.restart,
                g1.eq(0x3FF),
                g2.eq(0x3FF),
                self.index.eq(0),
            ).Elif(self.shift,
                g1.eq(Cat(fb1, g1[0:9])),
                g2.eq(Cat(fb2, g2[0:9])),
                If(self.index == (CA_CODE_LENGTH - 1),
                    self.index.eq(0),
                    self.epoch.eq(1),
                ).Else(
                    self.index.eq(self.index + 1),
                ),
            ),
        ]
