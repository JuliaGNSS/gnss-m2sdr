#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Bank integration test: configure channels via CSR storages, drive a
synthetic signal, and check the DMA record stream shows the right channel
locked. CSR .storage signals are poked directly (no bus needed in sim)."""

import math
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.record_format import RECORD_WORDS, unpack_record
from test.test_channel_lock import (
    synth_signal, FS, F_IF, CHIP_RATE, FRAC, PHASE_BITS, CA_CODE_LENGTH,
    AMP, CARRIER_AMP,
)


def mag(i, q):
    return math.hypot(i, q)


class TestBank(unittest.TestCase):
    def test_bank_locks_correct_channel(self):
        prns = [5, 10]
        dut = GNSSTracking(n_channels=2, prns=prns, code_frac_bits=FRAC)
        I, Q = synth_signal(5, code_offset_chips=0.0)  # PRN 5 signal

        carrier_fw = round(F_IF / FS * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
        code_step  = round(CHIP_RATE / FS * (1 << FRAC))
        spacing    = round(0.5 * (1 << FRAC))
        words = []

        def bench():
            # Configure both channels (PRN comes from the build-time code ROM).
            for chan in (dut.ch0, dut.ch1):
                yield chan._carrier_freq.storage.eq(carrier_fw)
                yield chan._carrier_phase.storage.eq(0)
                yield chan._code_freq.storage.eq(code_step)
                yield chan._spacing.storage.eq(spacing)
            yield dut._control.storage.eq(1)      # enable bank
            yield dut.source.ready.eq(1)
            yield
            # Pulse restart + carrier_set (0 -> 0b11) on both channels.
            for chan in (dut.ch0, dut.ch1):
                yield chan._control.storage.eq(0)
            yield
            for chan in (dut.ch0, dut.ch1):
                yield chan._control.storage.eq(0b11)
            yield
            for chan in (dut.ch0, dut.ch1):
                yield chan._control.storage.eq(0)
            yield
            # Feed samples; collect (data, first, last) beats, plus a tail so
            # both channels' records drain after the code-period epoch.
            for k in range(len(I) + 60):
                if k < len(I):
                    yield dut.sample_i.eq(I[k])
                    yield dut.sample_q.eq(Q[k])
                    yield dut.sample_stb.eq(1)
                else:
                    yield dut.sample_stb.eq(0)
                yield
                if (yield dut.source.valid) and (yield dut.source.ready):
                    words.append(((yield dut.source.data),
                                  (yield dut.source.first),
                                  (yield dut.source.last)))

        run_simulation(dut, bench())
        # Delimit records by first/last markers.
        recs, cur = [], []
        for data, first, last in words:
            if first:
                cur = []
            cur.append(data)
            if last and len(cur) == RECORD_WORDS:
                recs.append(unpack_record(cur))
        self.assertTrue(recs, "no records produced")
        by_ch = {}
        for r in recs:
            by_ch.setdefault(r["channel"], r)  # first dump per channel

        self.assertIn(0, by_ch)
        self.assertIn(1, by_ch)
        self.assertEqual(by_ch[0]["prn"], 5)
        self.assertEqual(by_ch[1]["prn"], 10)

        p0 = mag(by_ch[0]["i_prompt"], by_ch[0]["q_prompt"])  # PRN 5 replica vs PRN 5 signal
        p1 = mag(by_ch[1]["i_prompt"], by_ch[1]["q_prompt"])  # PRN 10 replica vs PRN 5 signal
        aligned = CARRIER_AMP * AMP * by_ch[0]["integrated_samples"]
        self.assertGreater(p0, 0.9 * aligned)   # ch0 locks
        self.assertLess(p1, 0.1 * aligned)      # ch1 (wrong PRN) does not


if __name__ == "__main__":
    unittest.main(verbosity=2)
