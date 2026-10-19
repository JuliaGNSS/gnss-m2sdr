#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""The RX tap: what the correlator is handed and what the host receives name the
same samples, through the SoC's pipeline buffer and a host that stalls."""

import random
import unittest

from migen import *
from litex.gen import *
from litex.soc.interconnect import stream

import test.migen_compat  # noqa: F401

from gnss_m2sdr.tdm import params as P
from gnss_m2sdr.tdm.engine import RXTap


class TapChain(LiteXModule):
    """RXTap -> the SoC's pipeline buffer -> a host, with every transfer
    registered, so a simulator reads exactly what moved on each edge."""
    def __init__(self, first_index, words, host_fifo_depth=16):
        self.tap = tap = RXTap(host_fifo_depth=host_fifo_depth)
        self.buf = buf = stream.Buffer([("data", 64)], pipe_valid=True, pipe_ready=True)
        self.comb += tap.source.connect(buf.sink)
        # The RFIC side: the words in order, each held until it is taken.
        rom = Memory(64, len(words), init=words)
        port = rom.get_port(async_read=True)
        self.specials += rom, port
        k = Signal(max=len(words) + 1)
        self.host_ready = Signal()
        self.go = Signal()
        phase = Signal(2)
        self.sync += If(self.go, phase.eq(phase + 1))
        self.comb += [
            port.adr.eq(k),
            # The RFIC delivers a word every few cycles, as the AD9361 does.
            tap.sink.valid.eq(self.go & (k < len(words)) & (phase == 0)),
            tap.sink.data.eq(port.dat_r),
            buf.source.ready.eq(self.host_ready),
            tap.tag_enable.eq(1),
        ]
        self.sync += If(tap.sink.valid & tap.sink.ready, k.eq(k + 1))
        self.took = Signal()
        self.engine_fire = Signal()
        self.engine_index = Signal(P.SAMPLE_INDEX_BITS)
        self.engine_data = Signal(64)
        self.host_fire = Signal()
        self.host_data = Signal(64)
        self.sync += [
            self.took.eq(tap.sink.valid & tap.sink.ready),
            self.engine_fire.eq(tap.valid),
            self.engine_index.eq(tap.index),
            self.engine_data.eq(tap.data),
            self.host_fire.eq(buf.source.valid & buf.source.ready),
            self.host_data.eq(buf.source.data),
        ]
        self.first_index = first_index


def decode_tags(words):
    """Host words -> device indices: low 12 bits from every word, the bits above
    from the first complete aligned block, carried along the low bits."""
    low = [((w >> 12) & 0xF) | ((w >> 28) & 0xF) << 4 | ((w >> 44) & 0xF) << 8 for w in words]
    hi_nib = [(w >> 60) & 0xF for w in words]
    # A complete aligned block: low runs 0..4095 without a hole.
    start = next(k for k in range(len(words) - 4095)
                 if low[k] == 0 and all(low[k + j] == j for j in range(4096)))
    high = 0
    for k in range(start, start + 4096):
        high |= hi_nib[k] << (4 * ((low[k] >> 8) & 0xF))
    idx = [None] * len(words)
    idx[start] = high << 12
    for k in range(start + 1, len(words)):
        idx[k] = idx[k - 1] + ((low[k] - low[k - 1]) % 4096)
    for k in range(start - 1, -1, -1):
        idx[k] = idx[k + 1] - ((low[k + 1] - low[k]) % 4096)
    return idx


class TestRXTap(unittest.TestCase):
    def test_the_correlator_sees_every_word_and_the_host_names_its_own(self):
        rng = random.Random(4)
        first_index = 0x3_1234_5000 - 37
        n_words = 12000
        sent = [rng.getrandbits(64) & 0x0FFF0FFF0FFF0FFF for _ in range(n_words)]
        dut = TapChain(first_index, sent)
        host, engine = [], []

        def bench():
            yield dut.tap.count.eq(first_index)
            yield
            yield dut.go.eq(1)
            cycle = 0
            drain = 0
            while drain < 100:
                cycle += 1
                drain = drain + 1 if len(engine) == n_words else 0
                # A host that keeps up, except for one long stall in the middle.
                ready = drain > 0 or (rng.random() < 0.9 and not (5000 <= cycle < 5100))
                yield dut.host_ready.eq(ready)
                yield
                if (yield dut.engine_fire):
                    engine.append(((yield dut.engine_index), (yield dut.engine_data)))
                if (yield dut.host_fire):
                    host.append((yield dut.host_data))
                if cycle > 4 * n_words:
                    break
            self.drops = (yield dut.tap._host_drops.status)

        run_simulation(dut, bench())
        # The correlator: every word, in order, on consecutive indices.
        self.assertEqual([d for _, d in engine], sent)
        self.assertEqual([i for i, _ in engine], list(range(first_index, first_index + n_words)))
        # The host: a subsequence, each word named by its own index.
        mask = 0x0FFF0FFF0FFF0FFF
        idx = decode_tags(host)
        by_index = dict(engine)
        for i, w in zip(idx, host):
            self.assertEqual(w & mask, by_index[i])
        self.assertGreater(self.drops, 0)
        self.assertEqual(len(host) + self.drops, n_words)


if __name__ == "__main__":
    unittest.main()
