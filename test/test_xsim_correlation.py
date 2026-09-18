#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""The LiteX-lowered channel correlates in Vivado's simulator.

Wraps scripts/xsim_correlation.py as a test, so a host with Vivado runs it with
the rest of the suite and a host without one (CI) skips it by name rather than
pretending. Two faults reached silicon through the gap this covers -- the Verilog
the build writes is not what the Migen simulator runs (docs/gateware_builds.md
5.6e and 5.6g) -- so it is worth the two minutes on any machine that can.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest

REPO   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(REPO, "scripts", "xsim_correlation.py")


def _have_xsim():
    root = os.environ.get("XILINX_VIVADO")
    return all(shutil.which(t) or (root and os.path.exists(os.path.join(root, "bin", t)))
               for t in ("xvlog", "xelab", "xsim"))


@unittest.skipUnless(_have_xsim(), "Vivado's xvlog/xelab/xsim are not on PATH")
class TestXsimCorrelation(unittest.TestCase):

    def _run(self, *args):
        with tempfile.TemporaryDirectory() as out:
            r = subprocess.run(
                [sys.executable, SCRIPT, "--out", out, *args],
                cwd=REPO, env={**os.environ, "PYTHONPATH": REPO},
                capture_output=True, text=True, timeout=1800)
        self.assertEqual(r.returncode, 0,
                         f"xsim_correlation.py {' '.join(args)} failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
        self.assertIn("RESULT: PASS", r.stdout)

    def test_gps_l1ca_three_taps(self):
        self._run("--periods", "2", "--stb-gap", "1", "--noise", "30")

    def test_galileo_e1b_cboc_five_taps(self):
        # Amplitude 3 on sigma 30 is a ~51 dBHz satellite at 24.552 MS/s. Not
        # louder: the CBOC replica's RMS is 19.9 and the carrier LUT's peak 127,
        # so a matched prompt sums 19.9 x 127 x amp x 98 209 samples per 4 ms
        # period -- 7.4e8 here, and past the 32-bit rail from amp ~ 9 (see
        # docs/gateware_builds.md 5.6g on the accumulator range at E1 rates).
        self._run("--signal", "GalileoE1B", "--periods", "1", "--stb-gap", "0",
                  "--noise", "30", "--amp", "3", "--doppler", "-2500")


if __name__ == "__main__":
    unittest.main(verbosity=2)
