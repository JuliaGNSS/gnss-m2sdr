#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Tests for the carrier NCO / replica generator."""

import math
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.carrier_nco import CarrierNCO


class TestCarrierNCO(unittest.TestCase):
    def _run(self, freq_ratio, n, phase_bits=32, lut_addr_bits=8, amp_bits=8):
        dut = CarrierNCO(phase_bits, lut_addr_bits, amp_bits)
        fw = round(freq_ratio * (1 << phase_bits)) & ((1 << phase_bits) - 1)
        cos, sin = [], []

        def bench():
            yield dut.freq_word.eq(fw)
            yield dut.stb.eq(1)
            yield  # settle: stb applies this edge, phase advances from next
            for _ in range(n):
                cos.append((yield dut.cos))
                sin.append((yield dut.sin))
                yield

        run_simulation(dut, bench())
        return cos, sin

    def test_matches_ideal_within_quantization(self):
        peak = 127
        freq_ratio = 0.01
        n = 512
        cos, sin = self._run(freq_ratio, n)
        # Quantization budget: amplitude rounding (<=0.5) + phase truncation
        # to lut_addr_bits (a full LSB = 2*pi/256 worst case).
        tol = peak * (2 * math.pi / 256) + 1.0
        for i in range(n):
            ang = 2 * math.pi * freq_ratio * i
            self.assertLessEqual(abs(cos[i] - peak * math.cos(ang)), tol, f"cos[{i}]")
            self.assertLessEqual(abs(sin[i] - peak * math.sin(ang)), tol, f"sin[{i}]")

    def test_amplitude_bounds(self):
        cos, sin = self._run(0.003, 400)
        self.assertLessEqual(max(abs(x) for x in cos), 127)
        self.assertLessEqual(max(abs(x) for x in sin), 127)
        # Covers a full cycle -> should see near-peak positive and negative.
        self.assertGreater(max(cos), 120)
        self.assertLess(min(cos), -120)

    def test_frequency_via_cycle_count(self):
        # Count sin zero-up-crossings over n samples -> ~ freq_ratio * n cycles.
        freq_ratio = 0.02
        n = 2000
        _, sin = self._run(freq_ratio, n)
        ups = sum(1 for i in range(1, n) if sin[i - 1] < 0 <= sin[i])
        self.assertAlmostEqual(ups, freq_ratio * n, delta=2)

    def test_set_phase(self):
        dut = CarrierNCO(32, 8, 8)
        got = {}

        def bench():
            yield dut.phase_in.eq(1 << 30)  # quarter cycle -> cos ~ 0, sin ~ +peak
            yield dut.set_phase.eq(1)
            yield
            yield dut.set_phase.eq(0)
            yield
            got["phase"] = (yield dut.phase)
            got["cos"] = (yield dut.cos)
            got["sin"] = (yield dut.sin)

        run_simulation(dut, bench())
        self.assertEqual(got["phase"], 1 << 30)
        self.assertLessEqual(abs(got["cos"]), 2)
        self.assertGreater(got["sin"], 120)


if __name__ == "__main__":
    unittest.main(verbosity=2)
