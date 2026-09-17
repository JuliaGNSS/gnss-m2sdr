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
            yield dut.tap_offset[0].eq(sp)          # early: +spacing
            yield dut.tap_offset[2].eq(-sp)         # late:  -spacing
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


class TestRegisteredChipWindow(unittest.TestCase):
    """The three chip words the taps read are registers, not asynchronous RAM
    reads (see code_replica.py). That is a timing fix, so what these tests pin
    is that it changed *nothing* about the replica: the window has to be exact
    on every sample, including the first one after an arbitrary rebase and the
    one where an early tap reaches back across the wrap.
    """

    FRAC = 20

    def _dut(self, code, max_code_length=64):
        return CodeReplica(frac_bits=self.FRAC,
                           max_code_length=max_code_length, code_init=code)

    @staticmethod
    def _pm(bit):
        return 1 if bit else -1

    def _taps_from(self, code, length, start_chip, spc, n, max_code_length=64):
        """Rebase onto `start_chip` and collect (early, prompt, late) per sample."""
        dut  = self._dut(code, max_code_length)
        step = (1 << self.FRAC) // spc
        off  = (1 << self.FRAC) // spc          # one input sample of offset
        got  = []

        def bench():
            yield dut.code_step.eq(step)
            yield dut.code_length.eq(length)
            yield dut.tap_offset[0].eq(off)     # early: one sample ahead
            yield dut.tap_offset[2].eq(-off)    # late:  one sample behind
            yield dut.restart_chip.eq(start_chip)
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield
            for _ in range(n):
                got.append(((yield dut.early), (yield dut.prompt),
                            (yield dut.late), (yield dut.chip_index)))
                yield

        run_simulation(dut, bench())
        return got

    def test_first_sample_after_a_rebase_is_already_right(self):
        # The window reloads in the restart cycle itself, so there is no dead
        # sample: chip 37's word must be on the prompt tap immediately.
        code = [(i * 7 + 3) % 2 for i in range(64)]
        got  = self._taps_from(code, length=64, start_chip=37, spc=4, n=1)
        self.assertEqual(got[0][3], 37)
        self.assertEqual(got[0][1], self._pm(code[37]))

    def test_early_tap_reaches_across_the_wrap_at_chip_zero(self):
        # At chip 0 the late tap reads code[length-1] -- the one word the
        # shifting window has to have brought round with it. Rebase one chip
        # short of the wrap and walk through it.
        length, spc = 16, 4
        code = [(i * 5 + 1) % 2 for i in range(64)]
        got  = self._taps_from(code, length=length, start_chip=length - 1,
                               spc=spc, n=3 * spc)
        seen_wrap = False
        for early, prompt, late, idx in got:
            self.assertEqual(prompt, self._pm(code[idx]))
            # Taps sit one input sample either side; with spc samples per chip
            # they only ever leave the chip on its first/last sample.
            self.assertIn(early, (self._pm(code[idx]),
                                  self._pm(code[(idx + 1) % length])))
            self.assertIn(late, (self._pm(code[idx]),
                                 self._pm(code[(idx - 1) % length])))
            if idx == 0:
                seen_wrap = True
        self.assertTrue(seen_wrap, "never reached chip 0")

    def test_window_matches_a_software_walk_of_the_code(self):
        # The strong form: every tap of every sample against a direct
        # floor(phase)-based evaluation of the same code, across two wraps at a
        # non-integer number of samples per chip (so the taps change chip at
        # phases the integer case never visits).
        length = 23
        code   = [(i * 11 + 4) % 2 for i in range(64)]
        step   = int(round((1 << self.FRAC) / 2.13))
        off    = step                              # one input sample
        dut    = self._dut(code)
        got, want = [], []

        def bench():
            yield dut.code_step.eq(step)
            yield dut.code_length.eq(length)
            yield dut.tap_offset[0].eq(off)
            yield dut.tap_offset[2].eq(-off)
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield
            for _ in range(2 * length * 3):
                idx  = (yield dut.chip_index)
                frac = (yield dut.code_frac)
                got.append(((yield dut.early), (yield dut.prompt), (yield dut.late)))
                # floor(phase +/- offset) in chips, wrapped into the code.
                want.append(tuple(
                    self._pm(code[(idx + ((frac + d) >> self.FRAC)) % length])
                    for d in (off, 0, -off)))
                yield

        run_simulation(dut, bench())
        self.assertEqual(got, want)


if __name__ == "__main__":
    unittest.main(verbosity=2)
