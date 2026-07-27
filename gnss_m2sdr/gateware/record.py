#
# This file is part of gnss-m2sdr.
#
# Correlator-dump recorder: channel dumps -> 64-bit record stream (for DMA1).
# SPDX-License-Identifier: BSD-2-Clause

"""Serialize per-channel correlator dumps into the DMA record stream.

Each channel's dump (produced once per code period) is latched into a holding
register with a per-channel sequence counter; an overflow flag is raised if a
new dump arrives before the previous one has been serialized (should not happen
at ~1 kHz dumps) -- a dump landing on the very cycle the previous record is
retired still fits, because the holding register frees on that cycle. A
round-robin FSM emits the 6-word record (see
record_format.py) for each pending channel into an output SyncFIFO, whose
source is connected to the DMA1 sink at SoC level.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litepcie.common import dma_layout

from gnss_m2sdr.record_format import RECORD_WORDS, FLAG_OVERFLOW


class ChannelDumpPort:
    """Signals a TrackingChannel drives into the recorder (one per channel)."""
    def __init__(self, accum_bits, code_frac_bits):
        self.stb                = Signal()
        self.ie = Signal((accum_bits, True)); self.qe = Signal((accum_bits, True))
        self.ip = Signal((accum_bits, True)); self.qp = Signal((accum_bits, True))
        self.il = Signal((accum_bits, True)); self.ql = Signal((accum_bits, True))
        self.integrated_samples = Signal(32)
        self.sample_index       = Signal(64)
        self.code_phase         = Signal(code_frac_bits)
        self.prn                = Signal(8)


class CorrelatorRecorder(LiteXModule):
    def __init__(self, n_channels, accum_bits=32, code_frac_bits=24, fifo_depth=64):
        self.ports  = [ChannelDumpPort(accum_bits, code_frac_bits) for _ in range(n_channels)]
        self.source = stream.Endpoint(dma_layout(64))
        self.overflow = Signal(n_channels)  # sticky per-channel overflow status

        # # #

        self.fifo = fifo = stream.SyncFIFO(dma_layout(64), fifo_depth, buffered=True)
        self.comb += fifo.source.connect(self.source)

        def s32(x):  # low 32 bits of a signed accumulator
            return x[:32]

        # Per-channel holding registers + pending/seq/overflow.
        pend  = Array(Signal() for _ in range(n_channels))
        seqs  = Array(Signal(8) for _ in range(n_channels))
        hold  = []

        # `pend` must have exactly one driver.  The serializer FSM below only
        # pulses `retiring` combinationally (Migen flattens submodule fragments
        # after the parent's, so a NextValue(pend[ch], 0) inside the FSM would
        # silently override a same-cycle capture here); the capture block owns
        # the flag.  `retire[i]` marks the cycle on which channel i's holding
        # register is freed, so a dump strobing on that cycle is captured -- it
        # is a fresh integration, not an overflow.
        ch       = Signal(max=max(2, n_channels))
        retiring = Signal()
        retire   = Signal(n_channels)
        self.comb += [retire[i].eq(retiring & (ch == i)) for i in range(n_channels)]

        for i, p in enumerate(self.ports):
            h = dict(
                sidx=Signal(64), nsamp=Signal(32), cphase=Signal(code_frac_bits),
                prn=Signal(8), seq=Signal(8), flags=Signal(8),
                ie=Signal(32), qe=Signal(32), ip=Signal(32), qp=Signal(32),
                il=Signal(32), ql=Signal(32),
            )
            hold.append(h)
            self.sync += [
                If(p.stb,
                    If(pend[i] & ~retire[i],
                        self.overflow[i].eq(1),  # sticky
                    ).Else(
                        pend[i].eq(1),
                        h["sidx"].eq(p.sample_index),
                        h["nsamp"].eq(p.integrated_samples),
                        h["cphase"].eq(p.code_phase),
                        h["prn"].eq(p.prn),
                        h["seq"].eq(seqs[i]),
                        h["flags"].eq(Mux(self.overflow[i], FLAG_OVERFLOW, 0)),
                        h["ie"].eq(s32(p.ie)), h["qe"].eq(s32(p.qe)),
                        h["ip"].eq(s32(p.ip)), h["qp"].eq(s32(p.qp)),
                        h["il"].eq(s32(p.il)), h["ql"].eq(s32(p.ql)),
                        seqs[i].eq(seqs[i] + 1),
                        self.overflow[i].eq(0),  # cleared once captured
                    ),
                ).Elif(retire[i],
                    pend[i].eq(0),
                ),
            ]

        # Round-robin serializer FSM (`ch` is declared with `retire` above).
        widx = Signal(max=RECORD_WORDS)
        word = Signal(64)

        # Selected channel's holding register -> current record word.
        def build_words(h, chan_id):
            return [
                h["sidx"],
                Cat(h["seq"], h["flags"], h["prn"], C(chan_id, 8), h["nsamp"]),  # w1 low..high
                Cat(h["ip"], h["qp"]),
                Cat(h["ie"], h["qe"]),
                Cat(h["il"], h["ql"]),
                Cat(h["cphase"], Signal(64 - code_frac_bits)),
            ]
        cases = {}
        for i, h in enumerate(hold):
            words_i = build_words(h, i)
            cases[i] = word.eq(Array(words_i)[widx])
        self.comb += Case(ch, cases)

        self.fsm = fsm = FSM(reset_state="SCAN")
        fsm.act("SCAN",
            If(pend[ch],
                NextState("EMIT"),
            ).Else(
                NextValue(ch, Mux(ch == (n_channels - 1), 0, ch + 1)),
            ),
        )
        fsm.act("EMIT",
            fifo.sink.valid.eq(1),
            fifo.sink.data.eq(word),
            fifo.sink.first.eq(widx == 0),
            fifo.sink.last.eq(widx == (RECORD_WORDS - 1)),
            If(fifo.sink.ready,
                If(widx == (RECORD_WORDS - 1),
                    NextValue(widx, 0),
                    retiring.eq(1),  # comb: the capture block clears pend[ch]
                    NextValue(ch, Mux(ch == (n_channels - 1), 0, ch + 1)),
                    NextState("SCAN"),
                ).Else(
                    NextValue(widx, widx + 1),
                ),
            ),
        )
