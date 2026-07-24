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

# PRN phase-selector taps (G2 stage pair), IS-GPS-200, PRN 1..32. ----------------------------------

CA_PHASE_SELECT = {
     1: ( 2,  6),  2: ( 3,  7),  3: ( 4,  8),  4: ( 5,  9),  5: ( 1,  9),
     6: ( 2, 10),  7: ( 1,  8),  8: ( 2,  9),  9: ( 3, 10), 10: ( 2,  3),
    11: ( 3,  4), 12: ( 5,  6), 13: ( 6,  7), 14: ( 7,  8), 15: ( 8,  9),
    16: ( 9, 10), 17: ( 1,  4), 18: ( 2,  5), 19: ( 3,  6), 20: ( 4,  7),
    21: ( 5,  8), 22: ( 6,  9), 23: ( 1,  3), 24: ( 4,  6), 25: ( 5,  7),
    26: ( 6,  8), 27: ( 7,  9), 28: ( 8, 10), 29: ( 1,  6), 30: ( 2,  7),
    31: ( 3,  8), 32: ( 4,  9),
}

CA_CODE_LENGTH = 1023


# Software reference (used by tests / host tooling). -----------------------------------------------

def ca_code_reference(prn):
    """Return the length-1023 C/A code for ``prn`` as a list of 0/1 chips."""
    s1, s2 = CA_PHASE_SELECT[prn]
    g1 = [1] * 10  # g1[i] == stage (i+1)
    g2 = [1] * 10
    out = []
    for _ in range(CA_CODE_LENGTH):
        g1_out = g1[9]
        g2_out = g2[s1 - 1] ^ g2[s2 - 1]
        out.append(g1_out ^ g2_out)
        # Advance one chip.
        fb1 = g1[2] ^ g1[9]                              # taps 3,10
        fb2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]  # taps 2,3,6,8,9,10
        g1 = [fb1] + g1[:9]
        g2 = [fb2] + g2[:9]
    return out


def ca_first10_octal(prn):
    """First-10-chips octal representation for cross-checking vs IS-GPS-200."""
    chips = ca_code_reference(prn)[:10]
    value = 0
    for c in chips:
        value = (value << 1) | c
    return oct(value)


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
