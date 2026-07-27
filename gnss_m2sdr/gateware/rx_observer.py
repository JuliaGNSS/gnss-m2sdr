#
# This file is part of gnss-m2sdr.
#
# RX word -> GNSS sample observer (AD9361 1R1T / 2R2T de-interleaving).
# SPDX-License-Identifier: BSD-2-Clause

"""Non-intrusive observer turning AD9361 RX words into a GNSS sample stream.

The AD9361 RX datapath hands the SoC 64-bit words holding two 16-bit
sign-extended I/Q pairs -- {ia, qa} in bits [0:32], {ib, qb} in bits [32:64]
(litex_m2sdr ad9361/core.py). What the 'a'/'b' slots *mean* depends on the PHY
channel mode (litex_m2sdr ad9361/phy.py):

  * 2R2T (mode=0): 'a' is RX1, 'b' is RX2 -- two antennas, so one word carries
    exactly one sample instant of the GNSS stream, across both antennas.
  * 1R1T (mode=1): 'a' and 'b' are two *consecutive* samples of the single RX
    stream, so one word carries two samples and the word rate is only fs/2.

Taking only bits [0:32] unconditionally therefore halves the sample rate in
1R1T: the host programs carrier_fw/code_step for fs while the bank only ever
sees fs/2, integrated_samples per code period halves, and nothing locks -- with
no error anywhere. Hence the mode-aware de-interleave below.

With num_ants=2 the 'b' slot is not ignored but becomes antenna 1 of the same
sample instant: both AD9361 RX chains run off the shared LO, so RX1/RX2 are
phase-coherent, which is what post-correlation beamforming needs (see
record_format.py). 1R1T has no second antenna at all, so antenna 1 mirrors
antenna 0 there and `ants_valid` drops to 1 -- reporting two identical antennas
would give the host's beamformer a singular covariance.

Timing: the 'b' sample is emitted one sys_clk cycle after its word. In 1R1T the
word rate is fs/2 <= 30.72 MHz against a 125 MHz sys_clk, so a new word cannot
legitimately arrive while a 'b' sample is still pending. If one does anyway (a
burst out of the RX buffer after DMA0 stalled, in which case samples were
already lost upstream), the incoming word wins and the pending 'b' is dropped:
samples stay in order rather than being emitted out of sequence.
"""

from migen import *

from litex.gen import *

from gnss_m2sdr.record_format import N_ANTS_MAX


class RXSampleObserver(LiteXModule):
    """64-bit RX word stream -> (sample_i_ants, sample_q_ants, sample_stb).

    mode_1r1t is the AD9361 PHY channel mode (0 = 2R2T, 1 = 1R1T); on the SoC it
    comes straight from the PHY's ``control`` CSR, which lives in sys_clk like
    this module, so no resynchronization is needed.

    Antenna 0 is also exposed as sample_i / sample_q, so single-antenna wiring is
    unchanged. `ants_valid` is how many antennas the stream actually carries.
    """
    def __init__(self, data_width=64, num_ants=1):
        assert 1 <= num_ants <= N_ANTS_MAX, f"1..{N_ANTS_MAX} antennas (AD9361 is 2T2R)"
        self.rx_data   = Signal(data_width)  # Accepted RX word {ia, qa, ib, qb}.
        self.rx_stb    = Signal()            # rx_stream.valid & rx_stream.ready.
        self.mode_1r1t = Signal()            # AD9361 PHY mode: 0 = 2R2T, 1 = 1R1T.

        self.sample_i_ants = [Signal((16, True)) for _ in range(num_ants)]
        self.sample_q_ants = [Signal((16, True)) for _ in range(num_ants)]
        self.sample_i   = self.sample_i_ants[0]   # same Signal, not a copy
        self.sample_q   = self.sample_q_ants[0]
        self.sample_stb = Signal()
        self.ants_valid = Signal(max=num_ants + 1, reset=num_ants)

        # # #

        # 1R1T only: hold the word's second sample ('b') for the next cycle.
        # pending is cleared implicitly (it is re-driven every cycle), and an
        # incoming word overrides a still-pending 'b'.
        pending   = Signal()
        pending_i = Signal((16, True))
        pending_q = Signal((16, True))
        self.sync += [
            pending.eq(self.rx_stb & self.mode_1r1t),
            If(self.rx_stb,
                pending_i.eq(self.rx_data[32:48]),
                pending_q.eq(self.rx_data[48:64]),
            ),
        ]
        self.comb += [
            If(self.rx_stb,
                self.sample_i.eq(self.rx_data[0:16]),
                self.sample_q.eq(self.rx_data[16:32]),
                self.sample_stb.eq(1),
            ).Elif(pending,
                self.sample_i.eq(pending_i),
                self.sample_q.eq(pending_q),
                self.sample_stb.eq(1),
            ),
        ]

        if num_ants > 1:
            # Antenna 1 is the word's 'b' slot -- RX2, sampled simultaneously
            # with RX1. In 1R1T that slot is the *next sample* of RX1 instead
            # (already emitted as antenna 0 on the following cycle), so there is
            # no second antenna: mirror antenna 0 and report one valid antenna.
            self.comb += [
                If(self.mode_1r1t,
                    self.sample_i_ants[1].eq(self.sample_i),
                    self.sample_q_ants[1].eq(self.sample_q),
                ).Else(
                    self.sample_i_ants[1].eq(self.rx_data[32:48]),
                    self.sample_q_ants[1].eq(self.rx_data[48:64]),
                ),
                self.ants_valid.eq(Mux(self.mode_1r1t, 1, num_ants)),
            ]
