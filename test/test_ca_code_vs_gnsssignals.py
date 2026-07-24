#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Cross-validate the C/A code reference against GNSSSignals.jl.

The golden file test/data/gps_l1ca_golden.json was produced by
GNSSSignals.jl (gen_code(1023, GPSL1CA(), prn, 1.023e6Hz)) with the +/-1
chips mapped to logical bits via 1->1, -1->0. This test needs no Julia; the
golden file is committed so CI can run it standalone.
"""

import json
import os
import unittest

from gnss_m2sdr.gateware.ca_code import ca_code_reference

GOLDEN = os.path.join(os.path.dirname(__file__), "data", "gps_l1ca_golden.json")


class TestCACodeVsGNSSSignals(unittest.TestCase):
    def test_all_prns_match_gnsssignals(self):
        with open(GOLDEN) as fp:
            codes = json.load(fp)["codes"]
        for prn in range(1, 33):
            golden = [int(b) for b in codes[str(prn)]]
            mine = ca_code_reference(prn)
            self.assertEqual(len(golden), 1023)
            self.assertEqual(mine, golden,
                f"PRN {prn} differs from GNSSSignals.jl reference")


if __name__ == "__main__":
    unittest.main(verbosity=2)
