#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Consecutive correlator dumps: period 2 must be as clean as period 1.

The existing lock tests stop at the first dump, so nothing pinned the *stateful*
half of the epoch/pacing design. Tracking runs for hours off one handover, so
what actually matters is that every code period looks like the first:

  * the accumulators are zeroed on dump, so dump N+1 carries only its own
    period's energy (a missing reset shows up as ~N x the prompt magnitude);
  * ``integrated_samples`` is the same on every period;
  * ``sample_index`` advances by exactly one code period, so the host's epoch
    grid (and the ``first_sample = sample_index - integrated_samples + 1``
    invariant it derives) holds across dumps, not just on the first one;
  * ``code_phase`` at the dump is the same on every period.

Setup: 2 samples/chip (``code_step`` = 2**23 divides 2**24 exactly, so a code
period is an integer 2046 samples and no NCO slip perturbs the comparison) and
``carrier_fw = 0`` with the phase set to 0, i.e. cos = 127, sin = 0. The carrier
replica is then constant, the signal is real (Q = 0), and the correlator input
repeats bit-exactly every code period -- so consecutive dumps must be *equal*,
which is a far tighter check on the reset than "roughly the same magnitude".
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH
from gnss_m2sdr.record_format import RECORD_WORDS, unpack_record

FRAC             = 24
SAMPLES_PER_CHIP = 2
CODE_STEP        = (1 << FRAC) // SAMPLES_PER_CHIP   # 0.5 chips/sample, exact
SPACING          = 1 << (FRAC - 1)                   # 0.5 chip E/L half spacing
PERIOD           = SAMPLES_PER_CHIP * CA_CODE_LENGTH  # 2046 samples / code period
AMP              = 1000                               # signal amplitude
CARRIER_AMP      = 127                                # cos(0) from the sin/cos LUT

ACCUMS = ("ie", "qe", "ip", "qp", "il", "ql")


def code_sample(prn, k, code_length=CA_CODE_LENGTH, _cache={}):
    """Signal sample k of a code-only (no carrier) replica-aligned signal."""
    code = _cache.get((prn, code_length))
    if code is None:
        code = [1 if b else -1 for b in ca_code_reference(prn)][:code_length]
        _cache[(prn, code_length)] = code
    return AMP * code[(k // SAMPLES_PER_CHIP) % code_length]


class TestChannelConsecutiveDumps(unittest.TestCase):
    def run_dumps(self, prn, n_dumps, stb_gap=0):
        # stb_gap>0 inserts idle cycles between samples (the sparse hardware
        # strobe): the epoch/reset must survive it on *every* period, not only
        # the first one.
        dut = TrackingChannel(prn=prn, code_frac_bits=FRAC, carrier_phase_bits=32)
        dumps = []

        def grab():
            if (yield dut.dump_stb):
                dumps.append(dict(
                    ie=(yield dut.ie), qe=(yield dut.qe),
                    ip=(yield dut.ip), qp=(yield dut.qp),
                    il=(yield dut.il), ql=(yield dut.ql),
                    n=(yield dut.integrated_samples),
                    sidx=(yield dut.sample_index),
                    cphase=(yield dut.dump_code_phase),
                ))

        def bench():
            yield dut.code_step.eq(CODE_STEP)
            yield dut.spacing.eq(SPACING)
            yield dut.carrier_fw.eq(0)       # constant replica: cos=127, sin=0
            yield dut.carrier_phase_in.eq(0)
            yield dut.carrier_set.eq(1)
            yield dut.restart.eq(1)
            yield
            yield dut.carrier_set.eq(0)
            yield dut.restart.eq(0)
            for k in range(n_dumps * PERIOD + 4):
                yield dut.sample_i.eq(code_sample(prn, k))
                yield dut.sample_q.eq(0)
                yield dut.sample_count.eq(k)  # global index of *this* sample
                yield dut.sample_stb.eq(1)
                yield
                yield from grab()
                for _ in range(stb_gap):
                    yield dut.sample_stb.eq(0)
                    yield
                    yield from grab()

        run_simulation(dut, bench())
        return dumps

    def assertConsecutive(self, dumps, n_dumps):
        self.assertEqual(len(dumps), n_dumps,
                         f"expected {n_dumps} dumps, got {len(dumps)}")
        first = dumps[0]

        # Aligned prompt, all energy in I (Q=0 signal, cos-only replica).
        self.assertEqual(first["ip"], CARRIER_AMP * AMP * PERIOD)
        self.assertEqual(first["qp"], 0)
        # 0.5-chip taps -> ~half the prompt correlation, and E/L balanced.
        self.assertEqual(first["ie"], first["il"])
        self.assertAlmostEqual(first["ie"] / first["ip"], 0.5, delta=0.1)

        for i, d in enumerate(dumps[1:], start=1):
            # The input repeats bit-exactly every period, so an accumulator that
            # was not zeroed (or a stale nsamp) shows up immediately here.
            for k in ACCUMS:
                self.assertEqual(d[k], first[k], f"dump {i} field {k} differs")
            self.assertEqual(d["n"], PERIOD, f"dump {i} integrated_samples")
            self.assertEqual(d["cphase"], first["cphase"], f"dump {i} code_phase")
            # Exactly one code period between dumps, on the global axis.
            self.assertEqual(d["sidx"] - dumps[i - 1]["sidx"], PERIOD,
                             f"dump {i} is not one code period after dump {i - 1}")

    def test_three_consecutive_dumps(self):
        n_dumps = 3
        dumps = self.run_dumps(prn=5, n_dumps=n_dumps)
        self.assertConsecutive(dumps, n_dumps)
        self.assertEqual(dumps[0]["n"], PERIOD)
        self.assertEqual(dumps[0]["sidx"], PERIOD - 1)  # 0-based last sample

    def test_consecutive_dumps_with_sparse_strobe(self):
        # One sample every 4 sys cycles: the epoch must keep landing on a strobe
        # cycle period after period (regression cover for the epoch timing).
        n_dumps = 2
        dumps = self.run_dumps(prn=5, n_dumps=n_dumps, stb_gap=3)
        self.assertConsecutive(dumps, n_dumps)


class TestBankConsecutiveRecords(unittest.TestCase):
    def test_record_stream_repeats_every_code_period(self):
        # Same invariants one level up, through the recorder and the DMA record
        # stream: successive records for a channel advance by one code period,
        # carry an incrementing seq, and repeat their payload exactly.
        n_records = 3
        prn = 5
        dut = GNSSTracking(n_channels=1, prns=[prn], code_frac_bits=FRAC)
        words = []

        def collect():
            if (yield dut.source.valid) and (yield dut.source.ready):
                words.append(((yield dut.source.data),
                              (yield dut.source.first),
                              (yield dut.source.last)))

        def bench():
            yield dut.ch0._code_freq.storage.eq(CODE_STEP)
            yield dut.ch0._spacing.storage.eq(SPACING)
            yield dut.ch0._carrier_freq.storage.eq(0)
            yield dut.ch0._carrier_phase.storage.eq(0)
            yield dut._control.storage.eq(1)          # enable bank
            yield dut.source.ready.eq(1)
            yield
            # Pulse restart + carrier_set (0 -> 0b11 -> 0), off strobe cycles.
            yield dut.ch0._control.storage.eq(0b11)
            yield
            yield dut.ch0._control.storage.eq(0)
            yield
            for k in range(n_records * PERIOD + 4):
                yield dut.sample_i.eq(code_sample(prn, k))
                yield dut.sample_q.eq(0)
                yield dut.sample_stb.eq(1)
                yield
                yield from collect()
            yield dut.sample_stb.eq(0)
            for _ in range(4 * RECORD_WORDS + 20):   # drain the record FIFO
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
        self.assertEqual(len(recs), n_records, "wrong number of records")

        r0 = recs[0]
        self.assertEqual(r0["prn"], prn)
        self.assertEqual(r0["integrated_samples"], PERIOD)
        self.assertEqual(r0["i_prompt"], CARRIER_AMP * AMP * PERIOD)
        # First gated sample is global sample 0 (the counter only sees the
        # strobes issued in this bench).
        self.assertEqual(r0["sample_index"], PERIOD - 1)
        self.assertEqual(r0["sample_index"] - r0["integrated_samples"] + 1, 0)

        for i, r in enumerate(recs):
            self.assertEqual(r["seq"], i, "seq must increment per record")
            self.assertEqual(r["flags"], 0, "unexpected overflow flag")
            self.assertEqual(r["integrated_samples"], PERIOD)
            self.assertEqual(r["sample_index"], PERIOD - 1 + i * PERIOD)
            for k in ("i_prompt", "q_prompt", "i_early", "q_early",
                      "i_late", "q_late", "code_phase", "prn", "channel"):
                self.assertEqual(r[k], r0[k], f"record {i} field {k} differs")


if __name__ == "__main__":
    unittest.main(verbosity=2)
