#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""TDM correlator: the LiteX-lowered Verilog in Vivado's xsim against the
bit-exact model, plus static checks on that Verilog. Skipped by name where the
simulator is unavailable (CI)."""

import os
import re
import tempfile
import unittest

from gnss_m2sdr.tdm import model as M
from test import tdm_xsim
from test.tdm_harness import random_code, parse_stream
from test.test_tdm_engine import build_scenario, strobe_scenario, CODE_LEN


@unittest.skipIf(tdm_xsim.tool_prefix() is None, "Vivado xsim not available")
class TestTDMVerilogInXsim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.mkdtemp(prefix="tdm_xsim_")
        cls.sc = build_scenario(random_code(CODE_LEN, seed=7))
        cls.words, cls.verilog = tdm_xsim.run(cls.sc, cls.dir, n_channels=4, code_words=256,
                                              ready_pattern=lambda k: (k // 5) % 4 != 0)

    def test_records_match_the_model(self):
        got = parse_stream(self.words)
        self.assertEqual(len(got), len(self.sc.records))
        for g, w in zip(got, self.sc.records):
            self.assertEqual(g, w)

    def test_no_negative_literal(self):
        # A unary-minus literal is how a signed constant reached silicon as an
        # unsigned one before (docs/tdm_correlator.md, "Two lowering faults").
        self.assertIsNone(re.search(r"(?:^|[(=,?:&|~!<>]\s*)-\s*\d+'", self.verilog, re.M))

    def test_memory_init_files_are_unsigned_hex(self):
        for fn in os.listdir(self.dir):
            if fn.endswith(".init"):
                for line in open(os.path.join(self.dir, fn)).read().split():
                    self.assertRegex(line, r"^[0-9a-fA-F]+$", fn)


@unittest.skipIf(tdm_xsim.tool_prefix() is None, "Vivado xsim not available")
class TestTDMVerilogScenarios(unittest.TestCase):
    """The other engine scenarios on the lowered Verilog: rounds back to back
    (the state RAMs' read-after-write margin), and the epoch strobes and the
    lost-sample flag over gaps and period changes."""
    def run_scenario(self, sc):
        words, _ = tdm_xsim.run(sc, tempfile.mkdtemp(prefix="tdm_xsim_"), n_channels=4, code_words=256)
        self.assertEqual(parse_stream(words), sc.records)

    def test_back_to_back_rounds(self):
        self.run_scenario(build_scenario(random_code(CODE_LEN, seed=7), spacing=7, drain=600))

    def test_strobes_and_lost_samples(self):
        self.run_scenario(strobe_scenario(random_code(CODE_LEN, seed=9)))


if __name__ == "__main__":
    unittest.main()
