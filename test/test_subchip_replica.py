#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""The sub-chip replica, checked against GNSSSignals.jl.

`test/data/l1_subchip_golden.json` was produced by GNSSSignals.jl v4.1.0 (the
generator is committed next to it as `gen_l1_subchip_golden.jl`) and holds, for
each L1 signal in scope:

  * the primary chips of a 528-chip excerpt (33 x 16, a whole number of TMBOC
    pattern periods, so the pattern still tiles a code that short);
  * the replica -- primary chip times `get_subcarrier_code` -- at 1600
    consecutive phases of *this gateware's own* code NCO: an exactly
    representable `code_step` accumulated in `frac_bits` fixed point from phase
    0, so a sample of the simulation and an entry of the golden file are the
    same phase with no interpolation on either side. The secondary code is not
    in it, because the gateware replicates the primary code only and the host
    removes the overlay (GNSSReceiver.jl#132).

That grid also makes the tap offsets checkable against GNSSSignals: a tap `s`
input samples early sits at exactly the phase of sample `k + s`, so the whole
tap-placement path is compared against the same reference rather than against
the gateware's own prompt.

The CBOC comparison carries a tolerance and the others do not: GNSSSignals'
resampled code table uses the integer amplitudes (19, 6) where the float
`get_subcarrier_code` uses sqrt(10/11) and sqrt(1/11), and the gateware holds
the same integers -- so the *only* difference is that 5e-4 approximation, which
is asserted as a bound rather than waved at.
"""

import json
import os
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.record_format import TAPS_EPL, TAPS_VEPL
from gnss_m2sdr.subcarrier import (
    CBOC_A1, CBOC_A2, TMBOC_L1CP_PATTERN, boc_cos_lut, boc_sin_lut, cboc_lut,
    lut_rms, replica_shape, signal_replica_shape, subchip_factor,
    tmboc_select_bits, SIGNAL_MODULATIONS,
)

GOLDEN = os.path.join(os.path.dirname(__file__), "data", "l1_subchip_golden.json")

with open(GOLDEN) as fp:
    G = json.load(fp)

FRAC      = G["frac_bits"]
CODE_STEP = G["code_step"]
NCHIP     = G["n_chips"]
NSAMPLE   = G["n_samples"]
MAX_SUB   = 12                   # covers every L1 modulation GNSSSignals exposes

# The integer CBOC table is an approximation of the sqrt-power amplitudes; this
# is how far it can be off, relative to the replica's own RMS amplitude.
CBOC_TOL = 1e-3


def golden_signal(key):
    return G["signals"][key]


def shape_for(key):
    """The gateware replica shape for a golden entry, with its select bits."""
    g = golden_signal(key)
    return signal_replica_shape(g["signal"], code_length=NCHIP)


def code_init(key, shape):
    """Code-RAM init words: chip in bit 0, subcarrier select in bit 1."""
    chips  = [int(c) for c in golden_signal(key)["chips"]]
    select = shape.select if shape.select is not None else [0] * len(chips)
    return [c | (s << 1) for c, s in zip(chips, select)]


def run_replica(shape, init, tap_shifts, n_samples, code_step=CODE_STEP,
                frac_bits=FRAC, max_subchips=MAX_SUB):
    """Capture every tap of a configured CodeReplica for `n_samples` samples.

    `tap_shifts` are whole input samples, earliest first (the gateware's own
    order); they are turned into offsets exactly as the host does, by
    multiplying the code step.
    """
    dut = CodeReplica(frac_bits=frac_bits, max_code_length=len(init),
                      code_init=init, num_taps=len(tap_shifts),
                      max_subchips=max_subchips)
    taps = [[] for _ in tap_shifts]

    def bench():
        for sel, lut in ((0, shape.lut_a), (1, shape.lut_b)):
            if lut is None:
                continue
            for adr, val in enumerate(lut):
                yield dut.lut_adr.eq(adr)
                yield dut.lut_sel.eq(sel)
                yield dut.lut_dat.eq(val)
                yield dut.lut_we.eq(1)
                yield
        yield dut.lut_we.eq(0)
        yield dut.subchips.eq(shape.subchips)
        yield dut.code_length.eq(len(init))
        yield dut.code_step.eq(code_step)
        for t, shift in enumerate(tap_shifts):
            yield dut.tap_offset[t].eq(shift * code_step)
        yield dut.restart.eq(1)
        yield
        yield dut.restart.eq(0)
        yield dut.stb.eq(1)
        yield                                   # settle
        for _ in range(n_samples):
            for t in range(len(tap_shifts)):
                taps[t].append((yield dut.replica[t]))
            yield

    run_simulation(dut, bench())
    return taps


class TestSubcarrierTablesMatchGNSSSignals(unittest.TestCase):
    """The tables the host programs are GNSSSignals' subcarrier, exactly."""

    def _check(self, name, lut, subchips, select_pattern=None, tol=0.0):
        g   = G["subcarrier"][name]
        amp = lut_rms(lut[0] if select_pattern else lut)
        npos, nsub = g["chip_positions"], g["sub_phases"]
        worst = 0.0
        for pos in range(npos):
            sel = bool(select_pattern and select_pattern[pos % len(select_pattern)])
            for k in range(nsub):
                # Sub-chip the phase pos + k/nsub falls in.
                idx  = (k * subchips) // nsub
                got  = (lut[1][idx] if sel else lut[0][idx]) if select_pattern else lut[idx]
                want = g["values"][pos * nsub + k]
                self.assertEqual(got > 0, want > 0,
                                 f"{name} sign at position {pos} sub-phase {k}")
                worst = max(worst, abs(got / amp - want))
        self.assertLessEqual(worst, tol, f"{name} amplitude error {worst}")
        return worst

    def test_loc_is_the_unit_table(self):
        self._check("LOC", [1], 1)

    def test_boc_sin_tables_are_exact(self):
        self._check("BOCsin_1_1", boc_sin_lut(1), subchip_factor("BOCsin", 1))
        self._check("BOCsin_6_1", boc_sin_lut(6), subchip_factor("BOCsin", 6))

    def test_boc_cos_table_is_exact(self):
        # The quarter-cycle shift straddles the sine grid, which is why BOCcos
        # needs P = 4m and not 2m; on that grid it is exact, not rounded.
        self._check("BOCcos_1_1", boc_cos_lut(1), subchip_factor("BOCcos", 1))

    def test_cboc_keeps_its_amplitudes(self):
        # Four levels, not two. The error bound is the integer approximation
        # GNSSSignals' own table uses -- so this is not "close enough to a sign",
        # it is the same table.
        for name, sign in (("CBOC_E1B", +1), ("CBOC_E1C", -1)):
            lut = cboc_lut(boc2_sign=sign)
            self.assertEqual(sorted(set(abs(v) for v in lut)),
                             [CBOC_A1 - CBOC_A2, CBOC_A1 + CBOC_A2])
            worst = self._check(name, lut, subchip_factor("CBOC", 1, 6),
                                tol=CBOC_TOL)
            self.assertGreater(worst, 0.0, "a sign-only table would read 0 here")

    def test_cboc_amplitude_is_gnsssignals_code_amplitude(self):
        # What GNSSReceiver divides out: get_code_amplitude(GalileoE1B). If the
        # device's replica RMS were 1 (a sign-only stand-in) the same satellite
        # would read ~26 dB off in C/N0.
        shape = signal_replica_shape("GalileoE1B")
        want  = golden_signal("GalileoE1B_prn1")["code_amplitude"]
        self.assertAlmostEqual(shape.code_amplitude, want, places=9)
        self.assertEqual(G["cboc_int_amplitudes"], [CBOC_A1, CBOC_A2])
        self.assertAlmostEqual(lut_rms(boc_sin_lut(1)), 1.0, places=12)

    def test_tmboc_tables_and_pattern(self):
        self.assertEqual([int(b) for b in TMBOC_L1CP_PATTERN], G["tmboc_pattern"])
        lut_a, lut_b = replica_shape("TMBOC").lut_a, replica_shape("TMBOC").lut_b
        self._check("TMBOC_L1CP", (lut_a, lut_b), subchip_factor("TMBOC", 1, 6),
                    select_pattern=TMBOC_L1CP_PATTERN)


class TestGatewareReplicaMatchesGNSSSignals(unittest.TestCase):
    """The prompt tap, sample for sample, against GNSSSignals' own replica."""

    def _compare(self, key, tol):
        shape = shape_for(key)
        taps  = run_replica(shape, code_init(key, shape), [0, 0, 0], NSAMPLE)
        want  = golden_signal(key)["replica"]
        amp   = shape.code_amplitude
        worst = 0.0
        for k, w in enumerate(want):
            got = taps[0][k] / amp
            self.assertEqual(got > 0, w > 0, f"{key} sign at sample {k}")
            worst = max(worst, abs(got - w))
        self.assertLessEqual(worst, tol, f"{key} worst error {worst}")

    def test_gps_l1ca_is_still_plain_bpsk(self):
        self._compare("GPSL1CA_prn1", 0.0)

    def test_boc11_signals(self):
        for key in ("GalileoE1B_BOC11_prn1", "GalileoE1C_BOC11_prn1",
                    "GPSL1C_D_prn1", "BeiDouB1C_D_prn1", "BeiDouB1C_P_prn1"):
            with self.subTest(key):
                self._compare(key, 0.0)

    def test_galileo_e1_cboc_including_the_anti_phase_pilot(self):
        # E1C's CBOC(-) is not E1B's CBOC(+) with a different code: the BOC(6,1)
        # component is subtracted, which swaps which sub-chips are the tall ones.
        for key in ("GalileoE1B_prn1", "GalileoE1B_prn7", "GalileoE1C_prn1"):
            with self.subTest(key):
                self._compare(key, CBOC_TOL)

    def test_gps_l1c_pilot_tmboc(self):
        for key in ("GPSL1C_P_prn1", "GPSL1C_P_prn19"):
            with self.subTest(key):
                self._compare(key, 0.0)

    def test_a_sign_only_cboc_would_fail_this_comparison(self):
        # The trap this test exists for: reducing the CBOC table to +/-1 keeps
        # every sign right and every amplitude wrong, so a sign-only check
        # passes and the C/N0 is ~26 dB out.
        key   = "GalileoE1B_prn1"
        shape = signal_replica_shape("GalileoE1B", code_length=NCHIP)
        signs = replica_shape("BOCsin", m=1, name="sign-only")
        taps  = run_replica(signs, code_init(key, signs), [0, 0, 0], 64)
        want  = golden_signal(key)["replica"]
        err   = max(abs(taps[0][k] / signs.code_amplitude - want[k])
                    for k in range(64))
        self.assertGreater(err, 0.2, "a sign-only replica must not pass as CBOC")
        self.assertAlmostEqual(shape.code_amplitude / signs.code_amplitude,
                               19.9248588, places=6)


class TestTapPlacement(unittest.TestCase):
    """Five independently placed taps, checked against GNSSSignals as well."""

    SHIFTS = (4, 1, 0, -1, -4)         # VE, E, P, L, VL, earliest first

    def _run(self, key, shifts=SHIFTS):
        shape = shape_for(key)
        n     = NSAMPLE
        taps  = run_replica(shape, code_init(key, shape), shifts, n)
        return shape, taps, golden_signal(key)["replica"]

    def test_every_tap_is_the_reference_replica_at_its_own_phase(self):
        # A tap `s` samples early is the replica at the phase of sample k + s.
        # Comparing against GNSSSignals (not against the gateware's own prompt)
        # is what makes this a check of the tap *and* the subcarrier together.
        for key in ("GalileoE1B_prn1", "GPSL1C_P_prn1", "GPSL1CA_prn1"):
            with self.subTest(key):
                shape, taps, want = self._run(key)
                tol = CBOC_TOL if shape.kind == "CBOC" else 0.0
                lo, hi = max(self.SHIFTS), NSAMPLE - max(self.SHIFTS)
                for t, shift in enumerate(self.SHIFTS):
                    for k in range(lo, hi):
                        got = taps[t][k] / shape.code_amplitude
                        self.assertLessEqual(abs(got - want[k + shift]), tol,
                                             f"{key} tap {t} sample {k}")

    def test_taps_need_not_be_symmetric(self):
        # GNSSReceiver hands over the whole shift array precisely because a
        # five-tap layout is not describable by one spacing number.
        shifts = (4, 3, 0, -1, -2)
        shape, taps, want = self._run("GalileoE1B_BOC11_prn1", shifts)
        lo, hi = 4, NSAMPLE - 4
        for t, shift in enumerate(shifts):
            for k in range(lo, hi):
                self.assertEqual(taps[t][k], round(want[k + shift]),
                                 f"tap {t} (shift {shift}) sample {k}")

    def test_a_three_tap_layout_is_unchanged(self):
        # GPS L1 C/A on three taps must be bit-for-bit what it always was.
        shape = shape_for("GPSL1CA_prn1")
        init  = code_init("GPSL1CA_prn1", shape)
        five  = run_replica(shape, init, (4, 1, 0, -1, -4), 400)
        three = run_replica(shape, init, (1, 0, -1), 400, max_subchips=1)
        self.assertEqual(five[1], three[0])
        self.assertEqual(five[2], three[1])
        self.assertEqual(five[3], three[2])


class TestReplicaShapeRefusals(unittest.TestCase):
    """Shapes the gateware cannot evaluate are reported, never approximated."""

    def _status(self, subchips, offsets, max_subchips=MAX_SUB):
        dut = CodeReplica(frac_bits=FRAC, max_code_length=64,
                          code_init=[1] * 64, num_taps=len(offsets),
                          max_subchips=max_subchips)
        out = {}

        def bench():
            yield dut.subchips.eq(subchips)
            for t, off in enumerate(offsets):
                yield dut.tap_offset[t].eq(off)
            yield
            out["bad"] = (yield dut.replica_unsupported)

        run_simulation(dut, bench())
        return out["bad"]

    def test_a_valid_shape_is_accepted(self):
        self.assertEqual(self._status(12, (1 << (FRAC - 1), 0, -(1 << (FRAC - 1)))), 0)

    def test_subchips_past_the_table_is_refused(self):
        # The index would run off the table and read whatever the default is --
        # a replica that looks like a replica.
        self.assertEqual(self._status(13, (0, 0, 0)), 1)
        self.assertEqual(self._status(0, (0, 0, 0)), 1)

    def test_a_whole_chip_tap_offset_is_refused(self):
        # -1.0 chip is the one out-of-range value the signed register can hold;
        # it would land on idx-1 with a zero fraction instead of idx-2.
        self.assertEqual(self._status(2, (0, 0, -(1 << FRAC))), 1)

    def test_a_loc_only_build_declares_no_subcarrier(self):
        self.assertEqual(self._status(1, (0, 0, 0), max_subchips=1), 0)
        self.assertEqual(self._status(0, (0, 0, 0), max_subchips=1), 1)


class TestSignalCoverageIsHonest(unittest.TestCase):
    """Claim exactly the GNSSSignals types whose replica is implemented."""

    def test_every_claimed_signal_has_a_golden_reference(self):
        covered = {g["signal"] for g in G["signals"].values()}
        self.assertEqual(set(SIGNAL_MODULATIONS), covered)

    def test_modulation_matches_what_gnsssignals_reports(self):
        for g in G["signals"].values():
            shape = signal_replica_shape(g["signal"])
            self.assertEqual(shape.kind, g["modulation"], g["signal"])
            self.assertEqual(shape.subchips, g["subchip_factor"], g["signal"])
            self.assertAlmostEqual(shape.code_amplitude, g["code_amplitude"],
                                   places=9, msg=g["signal"])

    def test_boc11_approximations_are_separate_signals(self):
        # Not a substitution: GNSSSignals gives the approximation its own type
        # and its own amplitude, and so does this module.
        full = signal_replica_shape("GalileoE1B")
        appr = signal_replica_shape("GalileoE1B_BOC11")
        self.assertNotEqual(full.kind, appr.kind)
        self.assertEqual(appr.code_amplitude, 1.0)
        self.assertGreater(full.code_amplitude, 19.0)

    def test_tmboc_select_bits_tile_the_code(self):
        bits = tmboc_select_bits(10230)
        self.assertEqual(len(bits), 10230)
        self.assertEqual(sum(bits), 4 * (10230 // 33))
        self.assertEqual(bits[:33], [int(b) for b in TMBOC_L1CP_PATTERN])
        self.assertEqual(bits[33:66], bits[:33])


if __name__ == "__main__":
    unittest.main(verbosity=2)
