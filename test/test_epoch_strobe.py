#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Periodic epoch-strobe records: a timebase that does not need a satellite.

GNSSReceiver.jl#107 closes epoch *e* when a record with
`sample_index >= (e+1)*delta` arrives. With correlator dumps as the only
trigger the host loop has no clock while nothing is locked (cold start, all
channels lost lock) and the epoch boundary jitters with the code-period phase
of whichever satellite happens to dump next.

Pinned here:
  * with `epoch_period` set, the recorder emits a marker record every delta
    input samples, timestamped on the same free-running counter as the dumps
    and tagged `channel == STROBE_CHANNEL` / `FLAG_EPOCH_STROBE`,
  * it does so with no channel dumping at all, and while the bank is disabled,
  * correlator dumps are untouched: same channel id, own seq, no strobe flag,
  * `epoch_period = 0` (the reset value) emits nothing, so an unconfigured
    build puts exactly the same stream on DMA1 as before,
  * a dropped marker gets the *same* treatment a dropped dump gets (#6): sticky
    bit n_channels of `overflow`, a saturating `dropped` counter, FLAG_OVERFLOW
    on the next marker, and nothing clears either but `overflow_clear`.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.record import CorrelatorRecorder
from gnss_m2sdr.record_format import (
    FLAG_EPOCH_STROBE, FLAG_OVERFLOW, RECORD_WORDS, STROBE_CHANNEL,
    is_epoch_strobe, pack_record, unpack_record,
)


VALS = dict(sample_index=4091, integrated_samples=4092, code_phase=0x123456,
            prn=7, i_early=11, q_early=-12, i_prompt=13, q_prompt=-14,
            i_late=15, q_late=-16)


def records(words):
    return [unpack_record(words[i:i + RECORD_WORDS])
            for i in range(0, len(words) - RECORD_WORDS + 1, RECORD_WORDS)]


def set_dump(port, vals):
    """Present a dump on `port` without consuming a cycle (stb is left to the caller)."""
    yield port.sample_index.eq(vals["sample_index"])
    yield port.integrated_samples.eq(vals["integrated_samples"])
    yield port.code_phase.eq(vals["code_phase"])
    yield port.prn.eq(vals["prn"])
    yield port.ie.eq(vals["i_early"]);  yield port.qe.eq(vals["q_early"])
    yield port.ip.eq(vals["i_prompt"]); yield port.qp.eq(vals["q_prompt"])
    yield port.il.eq(vals["i_late"]);   yield port.ql.eq(vals["q_late"])


def sample_bench(dut, words, n_samples, origin=0, gap=4, drain=200, hook=None):
    """Feed `n_samples` *sparse* strobes and collect whatever DMA1 emits.

    Hardware strobes one sample every fs/sys_clk cycles, so the strobe is high
    for a single cycle out of `gap`; a continuous strobe would hide a divider
    that counted clock cycles instead of samples. `sample_count` mimics the
    bank: during a strobe cycle it reads the 0-based index of the sample being
    presented, which is what a dump latches too.
    """
    def step():
        if (yield dut.source.valid) and (yield dut.source.ready):
            words.append((yield dut.source.data))
        yield

    for k in range(n_samples):
        yield dut.sample_count.eq(origin + k)
        yield dut.sample_stb.eq(1)
        yield from step()
        yield dut.sample_stb.eq(0)
        for _ in range(gap - 1):
            yield from step()
        if hook is not None:
            yield from hook(k, step)
    for _ in range(drain):
        yield from step()


class TestEpochStrobeRecorder(unittest.TestCase):
    def test_strobe_emitted_with_no_dumps(self):
        period, origin = 8, 1_000_000
        dut = CorrelatorRecorder(n_channels=2)
        words = []

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.epoch_period.eq(period)
            yield
            yield from sample_bench(dut, words, n_samples=40, origin=origin)

        run_simulation(dut, bench())
        recs = records(words)
        self.assertGreaterEqual(len(recs), 5, "no epoch strobes were emitted")
        for r in recs:
            self.assertTrue(is_epoch_strobe(r))
            self.assertEqual(r["channel"], STROBE_CHANNEL)
            self.assertFalse(r["flags"] & FLAG_OVERFLOW)
        # One marker every `period` samples on the shared counter, starting at
        # the first sample seen after the period was configured.
        self.assertEqual([r["sample_index"] for r in recs[:5]],
                         [origin + i * period for i in range(5)])
        self.assertEqual([r["seq"] for r in recs[:5]], [0, 1, 2, 3, 4])
        # A marker carries no correlator payload.
        for k in ("integrated_samples", "prn", "code_phase",
                  "i_early", "q_early", "i_prompt", "q_prompt", "i_late", "q_late"):
            self.assertEqual(recs[0][k], 0, k)
        # ... and it is a *normal* record on the wire (magic included), so the
        # host parser needs no special case to resynchronise on one.
        self.assertEqual(
            words[:RECORD_WORDS],
            pack_record(sample_index=origin, integrated_samples=0,
                        channel=STROBE_CHANNEL, prn=0, seq=0,
                        flags=FLAG_EPOCH_STROBE,
                        i_early=0, q_early=0, i_prompt=0, q_prompt=0,
                        i_late=0, q_late=0, code_phase=0,
                        # A marker has no correlator payload at all, so it
                        # reports zero antenna blocks (see record_format.py).
                        num_ants=0))

    def test_period_zero_emits_nothing(self):
        dut = CorrelatorRecorder(n_channels=1)
        words = []

        def bench():
            yield dut.source.ready.eq(1)
            yield                                  # epoch_period keeps its reset value
            yield from sample_bench(dut, words, n_samples=40)

        run_simulation(dut, bench())
        self.assertEqual(words, [], "strobes emitted with epoch_period = 0")

    def test_dumps_are_not_disturbed_by_strobes(self):
        """A dump keeps its channel id and its own seq, and is not tagged a strobe."""
        period = 8
        dut = CorrelatorRecorder(n_channels=1)
        words = []

        def dump_at(k, step):
            if k in (5, 21):
                yield from set_dump(dut.ports[0], {**VALS, "sample_index": 4091 + k})
                yield dut.ports[0].stb.eq(1)
                yield from step()
                yield dut.ports[0].stb.eq(0)

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.epoch_period.eq(period)
            yield
            yield from sample_bench(dut, words, n_samples=40, hook=dump_at)

        run_simulation(dut, bench())
        recs    = records(words)
        dumps   = [r for r in recs if not is_epoch_strobe(r)]
        strobes = [r for r in recs if is_epoch_strobe(r)]
        self.assertEqual(len(dumps), 2, "correlator dumps were lost")
        self.assertGreaterEqual(len(strobes), 5)
        for r in dumps:
            self.assertEqual(r["channel"], 0)
            self.assertEqual(r["prn"], VALS["prn"])
            self.assertEqual(r["integrated_samples"], VALS["integrated_samples"])
            self.assertFalse(r["flags"] & FLAG_EPOCH_STROBE)
        self.assertEqual([r["seq"] for r in dumps],      [0, 1])  # per-channel seq
        self.assertEqual([r["seq"] for r in strobes[:2]], [0, 1])  # strobe's own seq

    def test_dropped_strobe_gets_the_same_treatment_as_a_dropped_dump(self):
        """A marker lost because the previous one is still being serialized.

        Serializing a record takes RECORD_WORDS cycles, so a period shorter
        than that (only reachable by misconfiguration) drops markers. The point
        of this test is that the strobe slot is not a second-class citizen: it
        gets the full #6 treatment -- sticky bit n_channels of `overflow`, a
        saturating `dropped` counter that only `overflow_clear` resets, and
        FLAG_OVERFLOW on the next marker that does get through.
        """
        n_channels  = 1
        strobe_slot = n_channels          # last recorder slot
        dut = CorrelatorRecorder(n_channels=n_channels)
        words = []
        seen  = {}

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.epoch_period.eq(2)           # < RECORD_WORDS cycles apart
            yield
            yield from sample_bench(dut, words, n_samples=6, gap=1, drain=0)
            seen["overflow"] = (yield dut.overflow)
            seen["dropped"]  = (yield dut.dropped[strobe_slot])
            seen["ch0"]      = (yield dut.dropped[0])
            yield dut.epoch_period.eq(0)           # stop and re-arm
            yield from sample_bench(dut, words, n_samples=0, drain=40)
            yield dut.epoch_period.eq(8)
            yield from sample_bench(dut, words, n_samples=20, origin=100)
            # Sticky: emitting the flagged marker must NOT clear the status.
            seen["after"] = (yield dut.overflow)
            seen["after_dropped"] = (yield dut.dropped[strobe_slot])
            # Only an explicit write-1-to-clear does.
            yield dut.overflow_clear.eq(1 << strobe_slot)
            yield
            yield dut.overflow_clear.eq(0)
            yield
            seen["cleared"] = (yield dut.overflow)
            seen["cleared_dropped"] = (yield dut.dropped[strobe_slot])

        run_simulation(dut, bench())
        recs = records(words)
        self.assertTrue(recs, "nothing came out at all")
        # The strobe's own bit, and only that bit: the idle channel is clean.
        self.assertEqual(seen["overflow"], 1 << strobe_slot)
        self.assertGreaterEqual(seen["dropped"], 1, "drops were not counted")
        self.assertEqual(seen["ch0"], 0, "an idle channel's counter moved")
        flagged = [r for r in recs if r["flags"] & FLAG_OVERFLOW]
        self.assertTrue(flagged, "no marker carried the overflow flag")
        self.assertTrue(is_epoch_strobe(flagged[0]))
        self.assertEqual(flagged[0]["sample_index"], 100)   # first marker after re-arm
        self.assertEqual(len(flagged), 1, "transient flag was not consumed by one record")
        # Sticky across the flagged record, cleared only by overflow_clear.
        self.assertEqual(seen["after"], 1 << strobe_slot,
                         "sticky bit self-cleared when the flagged marker was emitted")
        self.assertEqual(seen["after_dropped"], seen["dropped"])
        self.assertEqual(seen["cleared"], 0, "overflow_clear did not clear the strobe bit")
        self.assertEqual(seen["cleared_dropped"], 0)


class TestEpochStrobeBank(unittest.TestCase):
    def test_bank_strobes_while_no_channel_tracks(self):
        """The scenario from the issue: nothing acquired, host still gets a clock.

        The bank is left disabled, so no channel can ever dump, yet DMA1 must
        still deliver markers placed on the global free-running counter.
        """
        period = 16
        dut = GNSSTracking(n_channels=2, code_frac_bits=24)
        words = []

        def step():
            if (yield dut.source.valid) and (yield dut.source.ready):
                words.append((yield dut.source.data))
            yield

        def bench():
            yield dut.source.ready.eq(1)
            yield dut._control.storage.eq(0)       # bank disabled: no dumps possible
            yield dut._epoch_period.storage.eq(period)
            yield
            for _ in range(80):                    # 80 sparse samples
                yield dut.sample_stb.eq(1)
                yield from step()
                yield dut.sample_stb.eq(0)
                for _ in range(3):
                    yield from step()
            for _ in range(200):
                yield from step()

        run_simulation(dut, bench())
        recs = records(words)
        self.assertGreaterEqual(len(recs), 5,
                                "a silent receiver produced no timebase records")
        self.assertTrue(all(is_epoch_strobe(r) for r in recs))
        # The bank's counter starts at 0 and is ungated by `enable`.
        self.assertEqual([r["sample_index"] for r in recs[:5]],
                         [i * period for i in range(5)])


if __name__ == "__main__":
    unittest.main(verbosity=2)
