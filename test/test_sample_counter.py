#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""The dump timestamp must come from ONE global free-running sample counter.

Tracking.jl's external-producer contract (and GNSSReceiver.jl#107) require every
channel -- and the raw DMA0 stream -- to report positions on a single sample
axis: the host subtracts a per-chunk origin, so a per-channel counter that is
zeroed by `restart` makes dumps from two channels incomparable.

Pinned here:
  * TrackingChannel latches the externally supplied counter (it owns no counter
    of its own, so `restart` cannot rebase the timestamp),
  * GNSSTracking's counter is free-running: it advances on every observed
    sample_stb, including while the bank is disabled, and survives `restart`,
  * two channels restarted N samples apart report sample_index values that
    differ by exactly N.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.record_format import RECORD_WORDS, unpack_record

FRAC = 24
# ~1 chip per sample: the code epoch (and hence the dump) lands on the sample
# after the last chip, i.e. on gated sample code_length+1, which keeps these
# simulations short.
FAST_CODE_STEP = (1 << FRAC) - 1


def epoch_sample(code_length):
    """1-based gated-sample number whose advance completes the code period."""
    return code_length + 1


class TestChannelUsesExternalCounter(unittest.TestCase):
    def test_sample_index_is_the_supplied_global_count(self):
        # The channel must report the *global* count of the last integrated
        # sample, not its own samples-since-restart. Feed a counter that starts
        # at a large origin (as the free-running hardware counter would after
        # hours of streaming) and check the dump reproduces it.
        code_length = 8
        origin      = 1_000_000
        dut = TrackingChannel(prn=1, code_frac_bits=FRAC, code_length=code_length)
        n_samples = epoch_sample(code_length) + 4
        got = {}

        def bench():
            yield dut.code_step.eq(FAST_CODE_STEP)
            yield dut.spacing.eq(1 << (FRAC - 1))
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            for k in range(n_samples):
                # Mimic the bank: the counter reads the 0-based index of the
                # current sample during its strobe cycle, then increments.
                yield dut.sample_count.eq(origin + k)
                yield dut.sample_stb.eq(1)
                yield
                yield dut.sample_stb.eq(0)
                for _ in range(3):              # sparse strobe, as on hardware
                    yield
                    if (yield dut.dump_stb):
                        got["sidx"] = (yield dut.sample_index)
                        got["n"]    = (yield dut.integrated_samples)
                if (yield dut.dump_stb):
                    got["sidx"] = (yield dut.sample_index)
                    got["n"]    = (yield dut.integrated_samples)

        run_simulation(dut, bench())
        self.assertTrue(got, "no correlator dump was produced")
        last = epoch_sample(code_length)                  # 1-based gated sample
        self.assertEqual(got["n"], last)
        self.assertEqual(got["sidx"], origin + last - 1)  # 0-based global index


class TestBankGlobalCounter(unittest.TestCase):
    def test_counter_runs_while_bank_disabled(self):
        # The counter observes the raw strobe, so it keeps a valid time axis
        # even when no channel is processing samples.
        n_strobes = 40
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        got = {}

        def bench():
            yield dut._control.storage.eq(0)   # bank disabled
            yield
            for _ in range(n_strobes):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            got["count"] = (yield dut.sample_count)
            got["csr"]   = (yield dut._sample_count.status)

        run_simulation(dut, bench())
        self.assertEqual(got["count"], n_strobes)
        self.assertEqual(got["csr"], n_strobes)

    def test_channels_restarted_apart_share_one_origin(self):
        # ch0 and ch1 are restarted CH1_OFFSET samples apart. With one global
        # counter their first dumps differ by exactly that offset; with
        # per-channel counters zeroed by restart both would report the same
        # value and the epoch grid would be meaningless across channels.
        CODE_LENGTH  = 1023
        PRE_STROBES  = 1500      # samples observed while the bank is disabled
        CH1_OFFSET   = 300       # gated samples between the two restarts
        dut = GNSSTracking(n_channels=2, prns=[5, 10], code_frac_bits=FRAC)
        words = []

        def pulse_restart(chan):
            # CSR restart is edge-triggered (0->1); keep it off strobe cycles.
            yield dut.sample_stb.eq(0)
            yield chan._control.storage.eq(0)
            yield
            yield chan._control.storage.eq(1)
            yield
            yield chan._control.storage.eq(0)
            yield

        def collect():
            if (yield dut.source.valid) and (yield dut.source.ready):
                words.append(((yield dut.source.data),
                              (yield dut.source.first),
                              (yield dut.source.last)))

        def bench():
            for chan in (dut.ch0, dut.ch1):
                yield chan._code_freq.storage.eq(FAST_CODE_STEP)
                yield chan._spacing.storage.eq(1 << (FRAC - 1))
            yield dut.source.ready.eq(1)
            yield
            # Strobes seen before the bank is enabled still advance the axis.
            for _ in range(PRE_STROBES):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield dut._control.storage.eq(1)      # enable bank
            yield
            yield from pulse_restart(dut.ch0)
            # Gated samples, restarting ch1 CH1_OFFSET samples into the run.
            n_gated = epoch_sample(CODE_LENGTH) + CH1_OFFSET + 4
            for k in range(n_gated):
                if k == CH1_OFFSET:
                    yield from pulse_restart(dut.ch1)
                yield dut.sample_stb.eq(1)
                yield
                yield from collect()
            yield dut.sample_stb.eq(0)
            for _ in range(60):                   # drain the record FIFO
                yield
                yield from collect()

        run_simulation(dut, bench())
        recs, cur = [], []
        for data, first, last in words:
            if first:
                cur = []
            cur.append(data)
            if last and len(cur) == RECORD_WORDS:
                recs.append(unpack_record(cur))
        first_dump = {}
        for r in recs:
            first_dump.setdefault(r["channel"], r)
        self.assertIn(0, first_dump, "channel 0 produced no dump")
        self.assertIn(1, first_dump, "channel 1 produced no dump")
        r0, r1 = first_dump[0], first_dump[1]

        n = epoch_sample(CODE_LENGTH)
        self.assertEqual(r0["integrated_samples"], n)
        self.assertEqual(r1["integrated_samples"], n)
        # Absolute position on the global axis: 0-based index of the last
        # integrated sample. ch0's integration starts at the first gated sample,
        # which is global sample PRE_STROBES.
        self.assertEqual(r0["sample_index"], PRE_STROBES + n - 1)
        # Companion invariant: first_sample = sample_index - integrated + 1.
        self.assertEqual(r0["sample_index"] - r0["integrated_samples"] + 1, PRE_STROBES)
        # Shared origin: the offset between the restarts is preserved exactly.
        self.assertEqual(r1["sample_index"] - r0["sample_index"], CH1_OFFSET)


if __name__ == "__main__":
    unittest.main(verbosity=2)
