#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Tests for the host-visible overflow status: sticky CSR bit + drop counters.

The per-record FLAG_OVERFLOW is a transient marker carried by the next captured
dump (covered in test_record.py). These tests pin the *status* contract the host
polls instead: `overflow` stays set until the host clears it, and `dropped[]`
counts how many dumps were lost, so a poll at any realistic rate cannot miss a
drop that happened between two reads.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.record import CorrelatorRecorder
from gnss_m2sdr.record_format import RECORD_WORDS, unpack_record, FLAG_OVERFLOW


VALS = dict(sample_index=0, integrated_samples=4092, code_phase=0, prn=7,
            i_early=1, q_early=2, i_prompt=3, q_prompt=4, i_late=5, q_late=6)


def dump(port, sample_index):
    """One-cycle dump strobe on `port`."""
    yield port.sample_index.eq(sample_index)
    yield port.integrated_samples.eq(VALS["integrated_samples"])
    yield port.code_phase.eq(VALS["code_phase"])
    yield port.prn.eq(VALS["prn"])
    yield port.ie.eq(VALS["i_early"]);  yield port.qe.eq(VALS["q_early"])
    yield port.ip.eq(VALS["i_prompt"]); yield port.qp.eq(VALS["q_prompt"])
    yield port.il.eq(VALS["i_late"]);   yield port.ql.eq(VALS["q_late"])
    yield port.stb.eq(1)
    yield
    yield port.stb.eq(0)


def drop_one(port, sample_index):
    """Capture a dump, then strobe again on the next cycle -> one lost dump.

    The holding register is still pending one cycle after a capture (the FSM
    needs a SCAN cycle plus the record's EMIT beats to free it), so the second
    strobe is a genuine overflow.
    """
    yield from dump(port, sample_index)
    yield from dump(port, sample_index + 1)


def idle(n, words=None, dut=None):
    for _ in range(n):
        if words is not None and (yield dut.source.valid) and (yield dut.source.ready):
            words.append((yield dut.source.data))
        yield


def records(words):
    return [unpack_record(words[i:i + RECORD_WORDS])
            for i in range(0, len(words), RECORD_WORDS)]


class TestStickyOverflow(unittest.TestCase):
    def test_overflow_status_survives_later_dumps(self):
        """The status bit must not be cleared by the next successful capture.

        Dumps arrive at ~1 kHz, so a bit that self-clears on the following
        capture is visible for at most ~1 ms and no realistic host poll rate
        can observe it.  The bit stays set until the host clears it; the record
        flag stays transient (only the first record after the loss carries it).
        """
        dut = CorrelatorRecorder(n_channels=1)
        words = []

        def bench():
            yield dut.source.ready.eq(1)
            yield from drop_one(dut.ports[0], 1)     # dump 1 kept, dump 2 lost
            yield from idle(40, words, dut)
            yield from dump(dut.ports[0], 3)          # carries FLAG_OVERFLOW
            yield from idle(40, words, dut)
            yield from dump(dut.ports[0], 4)          # clean record
            yield from idle(40, words, dut)
            self.overflow = (yield dut.overflow)

        run_simulation(dut, bench())
        recs = records(words)
        idxs = [r["sample_index"] for r in recs]
        self.assertEqual(idxs, [1, 3, 4])
        flagged = [r["sample_index"] for r in recs if r["flags"] & FLAG_OVERFLOW]
        self.assertEqual(flagged, [3], "FLAG_OVERFLOW is per-dump, on the first record after the loss")
        self.assertEqual(self.overflow, 1,
                         "sticky overflow status was cleared by a later successful dump")

    def test_overflow_clear_is_write_one_to_clear_per_channel(self):
        dut = CorrelatorRecorder(n_channels=2)
        seen = []

        def bench():
            yield dut.source.ready.eq(1)
            yield from drop_one(dut.ports[0], 10)
            yield from idle(40)
            yield from drop_one(dut.ports[1], 20)
            yield from idle(40)
            seen.append((yield dut.overflow))          # both channels flagged
            # Clear ch0 only.
            yield dut.overflow_clear.eq(0b01)
            yield
            yield dut.overflow_clear.eq(0)
            yield
            seen.append((yield dut.overflow))
            # Clearing again must not resurrect anything; now clear ch1.
            yield dut.overflow_clear.eq(0b10)
            yield
            yield dut.overflow_clear.eq(0)
            yield
            seen.append((yield dut.overflow))

        run_simulation(dut, bench())
        self.assertEqual(seen, [0b11, 0b10, 0b00])

    def test_dropped_counter_counts_every_lost_dump_and_saturates(self):
        dut = CorrelatorRecorder(n_channels=1, drop_count_bits=2)   # saturates at 3
        counts = []

        def bench():
            yield dut.source.ready.eq(1)
            for k in range(5):
                yield from drop_one(dut.ports[0], 100 + 2 * k)
                yield from idle(30)
                counts.append((yield dut.dropped[0]))

        run_simulation(dut, bench())
        self.assertEqual(counts, [1, 2, 3, 3, 3],
                         "drop counter must count each lost dump, then saturate")

    def test_drop_landing_on_a_clear_is_not_lost(self):
        """A clear must not swallow a drop that happens on the same cycle."""
        dut = CorrelatorRecorder(n_channels=1, drop_count_bits=8)
        state = []

        def bench():
            yield dut.source.ready.eq(1)
            yield from drop_one(dut.ports[0], 1)
            yield from idle(30)
            yield from drop_one(dut.ports[0], 3)
            yield from idle(30)
            state.append(((yield dut.overflow), (yield dut.dropped[0])))
            # Capture a dump, then strobe again while the host clears: the drop
            # and the clear collide on one cycle.
            yield from dump(dut.ports[0], 5)
            yield dut.ports[0].sample_index.eq(6)
            yield dut.ports[0].stb.eq(1)
            yield dut.overflow_clear.eq(1)
            yield
            yield dut.ports[0].stb.eq(0)
            yield dut.overflow_clear.eq(0)
            yield from idle(30)
            state.append(((yield dut.overflow), (yield dut.dropped[0])))

        run_simulation(dut, bench())
        self.assertEqual(state[0], (1, 2))
        self.assertEqual(state[1], (1, 1),
                         "drop colliding with a host clear was lost")


class TestBankOverflowCSRs(unittest.TestCase):
    def test_clear_csr_pulses_the_recorder_clear(self):
        """gnss_overflow_clear is write-1-to-clear: one pulse, masked per channel."""
        dut = GNSSTracking(n_channels=2)
        pulses = []

        def bench():
            for _ in range(2):
                pulses.append((yield dut.recorder.overflow_clear))
                yield
            yield from dut._overflow_clear.write(0b10)
            for _ in range(4):
                pulses.append((yield dut.recorder.overflow_clear))
                yield

        run_simulation(dut, bench())
        self.assertEqual([p for p in pulses if p], [0b10],
                         "a CSR write must produce exactly one masked clear pulse")

    def test_dropped_csrs_are_per_channel_and_wired(self):
        dut = GNSSTracking(n_channels=2)
        self.assertEqual(len(dut._overflow.status), 2)
        self.assertEqual(len(dut._overflow_clear.storage), 2)
        for i in range(2):
            csr = getattr(dut, f"_dropped{i}")
            self.assertEqual(csr.name, f"dropped{i}")
            self.assertEqual(len(csr.status), len(dut.recorder.dropped[i]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
