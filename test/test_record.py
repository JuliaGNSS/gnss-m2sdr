#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Tests for the correlator-dump recorder / DMA record serializer."""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.record import CorrelatorRecorder
from gnss_m2sdr.record_format import RECORD_WORDS, unpack_record, FLAG_OVERFLOW


def drive_dump(port, vals):
    yield port.sample_index.eq(vals["sample_index"])
    yield port.integrated_samples.eq(vals["integrated_samples"])
    yield port.code_phase.eq(vals["code_phase"])
    yield port.prn.eq(vals["prn"])
    yield port.ie.eq(vals["i_early"]);  yield port.qe.eq(vals["q_early"])
    yield port.ip.eq(vals["i_prompt"]); yield port.qp.eq(vals["q_prompt"])
    yield port.il.eq(vals["i_late"]);   yield port.ql.eq(vals["q_late"])
    yield port.stb.eq(1)
    yield
    yield port.stb.eq(0)


def collect_records(dut, words_out, n_words, extra=200):
    yield dut.source.ready.eq(1)
    for _ in range(n_words * RECORD_WORDS + extra):
        if (yield dut.source.valid) and (yield dut.source.ready):
            words_out.append((yield dut.source.data))
        yield


class TestRecorder(unittest.TestCase):
    def test_single_channel_roundtrip(self):
        dut = CorrelatorRecorder(n_channels=1)
        vals = dict(sample_index=0x0011223344556677, integrated_samples=4092,
                    code_phase=0x00ABCDEF, prn=5,
                    i_early=111, q_early=-222, i_prompt=333333, q_prompt=-444444,
                    i_late=555, q_late=-666)
        words = []

        def bench():
            yield from drive_dump(dut.ports[0], vals)
            yield from collect_records(dut, words, n_words=1)

        run_simulation(dut, bench())
        self.assertEqual(len(words), RECORD_WORDS)
        rec = unpack_record(words)
        self.assertEqual(rec["sample_index"], vals["sample_index"])
        self.assertEqual(rec["integrated_samples"], 4092)
        self.assertEqual(rec["channel"], 0)
        self.assertEqual(rec["prn"], 5)
        self.assertEqual(rec["seq"], 0)
        self.assertEqual(rec["code_phase"], 0x00ABCDEF)
        for k in ("i_early", "q_early", "i_prompt", "q_prompt", "i_late", "q_late"):
            self.assertEqual(rec[k], vals[k], k)

    def test_two_channels_and_seq(self):
        dut = CorrelatorRecorder(n_channels=2)
        v0 = dict(sample_index=1000, integrated_samples=4092, code_phase=1, prn=3,
                  i_early=1, q_early=2, i_prompt=3, q_prompt=4, i_late=5, q_late=6)
        v1 = dict(sample_index=2000, integrated_samples=4092, code_phase=2, prn=9,
                  i_early=-1, q_early=-2, i_prompt=-3, q_prompt=-4, i_late=-5, q_late=-6)
        words = []

        def bench():
            yield dut.source.ready.eq(1)
            yield from drive_dump(dut.ports[0], v0)
            yield from drive_dump(dut.ports[1], v1)
            # Second dump on ch0 to check seq increments.
            for _ in range(60):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield
            yield from drive_dump(dut.ports[0], {**v0, "sample_index": 5092})
            for _ in range(60):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield

        run_simulation(dut, bench())
        recs = [unpack_record(words[i:i + RECORD_WORDS])
                for i in range(0, len(words), RECORD_WORDS)]
        self.assertGreaterEqual(len(recs), 3)
        by_ch = {}
        for r in recs:
            by_ch.setdefault(r["channel"], []).append(r)
        self.assertIn(0, by_ch); self.assertIn(1, by_ch)
        self.assertEqual(by_ch[1][0]["prn"], 9)
        self.assertEqual(by_ch[1][0]["i_prompt"], -3)
        # ch0 appears twice with increasing seq.
        self.assertEqual(by_ch[0][0]["seq"], 0)
        self.assertEqual(by_ch[0][1]["seq"], 1)
        self.assertEqual(by_ch[0][1]["sample_index"], 5092)

    def test_overflow_flag(self):
        dut = CorrelatorRecorder(n_channels=1)
        v = dict(sample_index=1, integrated_samples=1, code_phase=0, prn=1,
                 i_early=0, q_early=0, i_prompt=1, q_prompt=0, i_late=0, q_late=0)
        words = []

        def bench():
            # Stall output; dump1 is held pending, dump2 arrives before it is
            # serialized -> dump2 dropped, overflow latched.
            yield dut.source.ready.eq(0)
            yield from drive_dump(dut.ports[0], {**v, "sample_index": 1})
            yield from drive_dump(dut.ports[0], {**v, "sample_index": 2})
            # Drain dump1 (flags=0).
            yield dut.source.ready.eq(1)
            for _ in range(40):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield
            # dump3 is the next successful capture -> carries the overflow flag.
            yield from drive_dump(dut.ports[0], {**v, "sample_index": 3})
            for _ in range(40):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield

        run_simulation(dut, bench())
        recs = [unpack_record(words[i:i + RECORD_WORDS])
                for i in range(0, len(words), RECORD_WORDS)]
        idxs = [r["sample_index"] for r in recs]
        self.assertIn(1, idxs)          # dump1 emitted
        self.assertNotIn(2, idxs)       # dump2 was dropped
        flagged = [r for r in recs if r["flags"] & FLAG_OVERFLOW]
        self.assertTrue(flagged, "no record carried the overflow flag")
        self.assertEqual(flagged[0]["sample_index"], 3)  # dump3 signals the loss

    def test_dump_on_retire_cycle_is_not_dropped(self):
        """A dump landing on the cycle the FSM retires a record must survive.

        On the last EMIT beat the holding register is being freed, so a dump
        strobing on that very cycle is a fresh integration that must be
        captured -- not dropped.  `pend[]` also has to be written from a single
        block: with the capture logic setting it and the FSM's NextValue
        clearing it, Migen flattens the FSM fragment after the parent's and the
        clear silently wins on a colliding cycle.
        """
        dut = CorrelatorRecorder(n_channels=1)
        v = dict(sample_index=1, integrated_samples=4092, code_phase=7, prn=11,
                 i_early=1, q_early=2, i_prompt=3, q_prompt=4, i_late=5, q_late=6)
        words = []
        overflow = []

        def bench():
            def step():
                if (yield dut.source.valid) and (yield dut.source.ready):
                    words.append((yield dut.source.data))
                yield

            yield dut.source.ready.eq(1)
            yield from drive_dump(dut.ports[0], v)
            # Word 0 of the record is accepted by the FIFO on the cycle where
            # sink.first is high with valid & ready; the FIFO never backpressures
            # here, so the six words go out back-to-back and the retire beat
            # (sink.last) is RECORD_WORDS-1 cycles later.  Writes issued in a
            # bench take effect on the next cycle, hence the -2.
            for _ in range(60):
                if ((yield dut.fifo.sink.valid) and (yield dut.fifo.sink.ready)
                        and (yield dut.fifo.sink.first)):
                    break
                yield from step()
            else:
                self.fail("first record never started")
            for _ in range(RECORD_WORDS - 2):
                yield from step()
            yield dut.ports[0].sample_index.eq(2)
            yield dut.ports[0].stb.eq(1)
            yield from step()
            self.assertTrue((yield dut.fifo.sink.last) and (yield dut.fifo.sink.valid)
                            and (yield dut.fifo.sink.ready),
                            "stb did not land on the retire beat")
            yield dut.ports[0].stb.eq(0)
            for _ in range(3 * RECORD_WORDS + 20):
                yield from step()
            overflow.append((yield dut.overflow))

        run_simulation(dut, bench())
        recs = [unpack_record(words[i:i + RECORD_WORDS])
                for i in range(0, len(words), RECORD_WORDS)]
        self.assertEqual([r["sample_index"] for r in recs], [1, 2],
                         "dump colliding with the retire beat was dropped")
        self.assertEqual([r["seq"] for r in recs], [0, 1])
        self.assertEqual(recs[1]["prn"], 11)
        # The holding register was free on that cycle -> this is not an overflow.
        self.assertFalse(recs[1]["flags"] & FLAG_OVERFLOW)
        self.assertEqual(overflow, [0], "spurious sticky overflow flag")

    def test_retire_frees_only_the_serialized_channel(self):
        """The retire pulse is per-channel: it must not free another channel.

        While the FSM retires ch0, ch1's record is still pending, so a dump on
        ch1 on that cycle is a genuine overflow and must be dropped + flagged.
        """
        dut = CorrelatorRecorder(n_channels=2)
        v = dict(integrated_samples=4092, code_phase=0, prn=1,
                 i_early=0, q_early=0, i_prompt=0, q_prompt=0, i_late=0, q_late=0)
        words = []

        def bench():
            def step():
                if (yield dut.source.valid) and (yield dut.source.ready):
                    words.append((yield dut.source.data))
                yield

            yield dut.source.ready.eq(1)
            yield from drive_dump(dut.ports[0], {**v, "sample_index": 10})
            yield from drive_dump(dut.ports[1], {**v, "sample_index": 20})
            # ch0 is serialized first while ch1 stays pending behind it.
            for _ in range(60):
                if ((yield dut.fifo.sink.valid) and (yield dut.fifo.sink.ready)
                        and (yield dut.fifo.sink.first)):
                    break
                yield from step()
            else:
                self.fail("ch0 record never started")
            for _ in range(RECORD_WORDS - 2):
                yield from step()
            yield dut.ports[1].sample_index.eq(21)
            yield dut.ports[1].stb.eq(1)
            yield from step()
            yield dut.ports[1].stb.eq(0)
            for _ in range(2 * RECORD_WORDS + 20):
                yield from step()
            # Next accepted ch1 dump carries the overflow flag.
            yield from drive_dump(dut.ports[1], {**v, "sample_index": 22})
            for _ in range(2 * RECORD_WORDS + 20):
                yield from step()

        run_simulation(dut, bench())
        recs = [unpack_record(words[i:i + RECORD_WORDS])
                for i in range(0, len(words), RECORD_WORDS)]
        ch1 = [r for r in recs if r["channel"] == 1]
        self.assertEqual([r["sample_index"] for r in ch1], [20, 22],
                         "colliding dump on a *different* channel overwrote a pending record")
        self.assertFalse(ch1[0]["flags"] & FLAG_OVERFLOW)
        self.assertTrue(ch1[1]["flags"] & FLAG_OVERFLOW, "dropped dump was not flagged")


if __name__ == "__main__":
    unittest.main(verbosity=2)
