#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""SoC wiring guards that do not need a synthesised design.

Two failure modes that used to be silent or confusing:
  * the litex_m2sdr checkout defaulted to one developer's worktree path, so
    everyone else got a FileNotFoundError for a directory they never had;
  * a missing pcie_dma1 left the record stream unconnected, which shows up as
    "every channel overflows" rather than as a build error.

gnss_m2sdr/soc.py loads the real litex_m2sdr.py build script at import time, so
these tests point LITEX_M2SDR_DIR at a stub tree (a litex_m2sdr.py defining a
bare BaseSoC) and load soc.py under a private module name. That keeps the tests
independent of which litex_m2sdr checkout happens to be installed -- and it
exercises load_base_module()/find_litex_m2sdr_dir() on the way in.
"""

import importlib.util
import os
import shutil
import sys
import tempfile
import unittest

REPO     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOC_PATH = os.path.join(REPO, "gnss_m2sdr", "soc.py")

STUB_BASE_SOC = """
class BaseSoC:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
"""


def make_stub_tree(body=STUB_BASE_SOC):
    """A directory that looks like a litex_m2sdr checkout."""
    d = tempfile.mkdtemp()
    with open(os.path.join(d, "litex_m2sdr.py"), "w") as fp:
        fp.write(body)
    return d


def load_soc_module(litex_dir):
    """Import gnss_m2sdr/soc.py against `litex_dir`, under a private name."""
    old = os.environ.get("LITEX_M2SDR_DIR")
    os.environ["LITEX_M2SDR_DIR"] = litex_dir
    try:
        spec = importlib.util.spec_from_file_location("gnss_soc_under_test", SOC_PATH)
        mod  = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        if old is None:
            os.environ.pop("LITEX_M2SDR_DIR", None)
        else:
            os.environ["LITEX_M2SDR_DIR"] = old


class SocTestCase(unittest.TestCase):
    def setUp(self):
        self.stub = make_stub_tree()
        self.addCleanup(shutil.rmtree, self.stub, True)
        self.addCleanup(lambda: self.stub in sys.path and sys.path.remove(self.stub))
        self.soc = load_soc_module(self.stub)

    def make_tree(self):
        d = make_stub_tree()
        self.addCleanup(shutil.rmtree, d, True)
        return d


class TestLitexM2SDRDirResolution(SocTestCase):
    def test_env_var_wins(self):
        d = self.make_tree()
        self.assertEqual(
            self.soc.find_litex_m2sdr_dir(env={"LITEX_M2SDR_DIR": d}, candidates=()),
            d)

    def test_falls_back_to_conventional_candidates(self):
        d = self.make_tree()
        self.assertEqual(
            self.soc.find_litex_m2sdr_dir(env={}, candidates=("/nonexistent/x", d)),
            d)

    def test_wrong_env_var_is_not_silently_replaced_by_a_default(self):
        good = self.make_tree()
        with self.assertRaises(FileNotFoundError) as cm:
            self.soc.find_litex_m2sdr_dir(env={"LITEX_M2SDR_DIR": "/nonexistent/oops"},
                                          candidates=(good,))
        self.assertIn("/nonexistent/oops", str(cm.exception))

    def test_error_names_every_path_tried(self):
        with self.assertRaises(FileNotFoundError) as cm:
            self.soc.find_litex_m2sdr_dir(env={}, candidates=("/nonexistent/a",
                                                              "/nonexistent/b"))
        msg = str(cm.exception)
        self.assertIn("LITEX_M2SDR_DIR", msg)
        self.assertIn("/nonexistent/a", msg)
        self.assertIn("/nonexistent/b", msg)

    def test_default_candidates_are_not_developer_specific(self):
        # Every default must be derived from the user's home or from where this
        # repo actually sits -- never a path baked in from someone's machine.
        cands = self.soc.LITEX_M2SDR_DIR_CANDIDATES
        self.assertEqual(cands[0], os.path.expanduser("~/litex_m2sdr"))
        self.assertIn(os.path.join(os.path.dirname(REPO), "litex_m2sdr"), cands)
        for cand in cands:
            self.assertTrue(
                cand.startswith(os.path.expanduser("~")) or
                cand.startswith(os.path.dirname(REPO)),
                f"{cand} is not relative to $HOME or to this repo")

    def test_load_base_module_uses_the_resolved_directory(self):
        self.assertTrue(hasattr(self.soc.BaseSoC, "__init__"))
        self.assertIn(self.stub, sys.path)


class TestRecordDMARequired(SocTestCase):
    def test_missing_pcie_dma1_raises(self):
        class NoDMA:
            pass
        with self.assertRaises(AttributeError) as cm:
            self.soc.require_record_dma(NoDMA())
        self.assertIn("pcie_dma1", str(cm.exception))

    def test_present_pcie_dma1_accepted(self):
        class WithDMA:
            pcie_dma1 = object()
        self.soc.require_record_dma(WithDMA())   # must not raise

    def test_gnss_soc_forces_two_dmas(self):
        soc = self.soc.GNSSSoC()
        self.assertEqual(soc.kwargs["pcie_dmas"], 2)
        self.assertTrue(soc.kwargs["with_pcie"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
