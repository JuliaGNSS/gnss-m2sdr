#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Tests for the GPS L1 C/A code generator (software reference + Migen module)."""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.ca_code import (
    CACodeGenerator, ca_code_reference, ca_first10_octal, CA_CODE_LENGTH,
)

# IS-GPS-200 Table 3-Ia: first 10 chips, octal (PRN 1..10). ----------------------------------------
IS_GPS_200_FIRST10_OCTAL = {
     1: 0o1440,  2: 0o1620,  3: 0o1710,  4: 0o1744,  5: 0o1133,
     6: 0o1455,  7: 0o1131,  8: 0o1454,  9: 0o1626, 10: 0o1504,
}


class TestCACodeReference(unittest.TestCase):
    def test_first10_octal_matches_is_gps_200(self):
        for prn, expected in IS_GPS_200_FIRST10_OCTAL.items():
            got = int(ca_first10_octal(prn), 8)
            self.assertEqual(got, expected,
                f"PRN {prn}: got {oct(got)}, expected {oct(expected)}")

    def test_length_and_period(self):
        code = ca_code_reference(1)
        self.assertEqual(len(code), CA_CODE_LENGTH)
        self.assertTrue(all(c in (0, 1) for c in code))

    def test_balance(self):
        # A Gold code of length 1023 has 512 ones and 511 zeros (balanced).
        for prn in range(1, 33):
            ones = sum(ca_code_reference(prn))
            self.assertEqual(ones, 512, f"PRN {prn} not balanced: {ones} ones")

    def test_autocorrelation_peak(self):
        # Prompt autocorrelation peak = 1023; off-peak is small (<= 65 magnitude).
        prn = 5
        code = [1 - 2 * c for c in ca_code_reference(prn)]  # map to +/-1
        n = CA_CODE_LENGTH
        peak = sum(code[i] * code[i] for i in range(n))
        self.assertEqual(peak, n)
        worst = 0
        for shift in range(1, n):
            corr = sum(code[i] * code[(i + shift) % n] for i in range(n))
            worst = max(worst, abs(corr))
        self.assertLessEqual(worst, 65)


class TestCACodeGenerator(unittest.TestCase):
    def _run_module_vs_reference(self, prn, n_chips):
        dut = CACodeGenerator(prn=prn)
        ref = ca_code_reference(prn)
        got = []

        def bench():
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.shift.eq(1)
            # Settle: Migen applies the shift-enable write on this edge; the
            # LFSR advances only from the next edge, so no chip is skipped.
            yield
            for _ in range(n_chips):
                got.append((yield dut.chip))
                yield

        run_simulation(dut, bench())
        expected = [ref[i % CA_CODE_LENGTH] for i in range(n_chips)]
        self.assertEqual(got, expected)

    def test_module_matches_reference_prn1(self):
        self._run_module_vs_reference(prn=1, n_chips=CA_CODE_LENGTH)

    def test_module_matches_reference_prn5(self):
        self._run_module_vs_reference(prn=5, n_chips=CA_CODE_LENGTH)

    def test_module_wraps_after_1023(self):
        # Run past one full period to confirm the sequence repeats (epoch wrap).
        self._run_module_vs_reference(prn=7, n_chips=CA_CODE_LENGTH + 50)


if __name__ == "__main__":
    unittest.main(verbosity=2)
