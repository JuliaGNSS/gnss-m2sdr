#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Tests for the code NCO + E/P/L code replica."""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH


class TestCodeReplica(unittest.TestCase):
    FRAC = 20

    def _run(self, prn, samples_per_chip, spacing_chips, n):
        dut = CodeReplica(prn=prn, frac_bits=self.FRAC)
        step = (1 << self.FRAC) // samples_per_chip
        sp   = int(round(spacing_chips * (1 << self.FRAC)))
        rec = {"e": [], "p": [], "l": [], "epoch": [], "idx": []}

        def bench():
            yield dut.code_step.eq(step)
            yield dut.spacing.eq(sp)
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield  # settle
            for _ in range(n):
                rec["e"].append((yield dut.early))
                rec["p"].append((yield dut.prompt))
                rec["l"].append((yield dut.late))
                rec["idx"].append((yield dut.chip_index))
                rec["epoch"].append((yield dut.epoch))
                yield

        run_simulation(dut, bench())
        return rec

    def test_prompt_reproduces_code(self):
        prn, spc = 5, 4
        rec = self._run(prn, samples_per_chip=spc, spacing_chips=0.5,
                        n=spc * CA_CODE_LENGTH)
        code_pm = [1 if b else -1 for b in ca_code_reference(prn)]
        # Chip i is held for `spc` samples; sample the middle of each chip.
        for chip in range(CA_CODE_LENGTH):
            s = chip * spc + spc // 2
            self.assertEqual(rec["p"][s], code_pm[chip], f"chip {chip}")

    def test_epoch_period(self):
        spc = 4
        rec = self._run(1, samples_per_chip=spc, spacing_chips=0.5,
                        n=2 * spc * CA_CODE_LENGTH + 10)
        epochs = [i for i, e in enumerate(rec["epoch"]) if e]
        # Exactly one epoch per full code period (spc*1023 samples).
        self.assertEqual(len(epochs), 2)
        self.assertEqual(epochs[1] - epochs[0], spc * CA_CODE_LENGTH)

    def test_early_leads_late_trails(self):
        # 4 samples/chip, 0.5-chip spacing -> E leads P by 2 samples, L trails by 2.
        spc, lead = 4, 2
        rec = self._run(5, samples_per_chip=spc, spacing_chips=0.5,
                        n=spc * CA_CODE_LENGTH)
        n = len(rec["p"])
        for i in range(lead, n - lead):
            self.assertEqual(rec["e"][i], rec["p"][i + lead], f"early[{i}]")
            self.assertEqual(rec["l"][i], rec["p"][i - lead], f"late[{i}]")

    def test_spacing_zero_collapses_epl(self):
        rec = self._run(7, samples_per_chip=4, spacing_chips=0.0, n=400)
        for i in range(len(rec["p"])):
            self.assertEqual(rec["e"][i], rec["p"][i])
            self.assertEqual(rec["l"][i], rec["p"][i])


if __name__ == "__main__":
    unittest.main(verbosity=2)
