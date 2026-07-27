#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Tests for the host-side E/L spacing quantisation.

Tracking.jl quantises the preferred Early/Late chip shift to a whole number of
input samples (``calc_preferred_code_shift_to_sample_shift``) and ``dll_disc``
normalises with the *quantised* spacing. The host must therefore program the
FPGA with that same grid, otherwise the discriminator gain is wrong by up to a
few percent. These tests pin both the CSR word the host computes and the tap
positions the gateware then produces.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.gps_ca import GPS_CA_CHIP_RATE
from software.gnss_tracking import GNSSChannel

FS   = 4e6          # bring-up sample rate (fs/f_code = 3.9101 samples/chip)
FRAC = 24


class FakeCSR:
    """Records CSR writes by name instead of touching /dev/m2sdr0."""
    def __init__(self):
        self.written = {}

    def write(self, name, value):
        self.written[name] = value

    def read(self, name):
        return self.written.get(name, 0)


def channel(fs=FS):
    csr = FakeCSR()
    return GNSSChannel(csr, fs, index=0, code_frac_bits=FRAC), csr


def replica_taps(code_step, spacing, n, prn=5, stb_gap=0):
    """Run CodeReplica for `n` enabled samples; return the E/P/L chip streams.

    `stb_gap` idle cycles between strobes reproduce the sparse hardware strobe.
    Samples are recorded on the cycles where `stb` actually reads high, so the
    stream is the per-sample replica state for either strobe density.
    """
    dut = CodeReplica(prn=prn, frac_bits=FRAC)
    rec = {"e": [], "p": [], "l": []}

    def bench():
        yield dut.code_step.eq(code_step)
        yield dut.spacing.eq(spacing)
        yield dut.restart.eq(1)
        yield
        yield dut.restart.eq(0)
        k = 0
        while len(rec["p"]) < n:
            yield dut.stb.eq(1 if k % (stb_gap + 1) == 0 else 0)
            if (yield dut.stb):
                rec["e"].append((yield dut.early))
                rec["p"].append((yield dut.prompt))
                rec["l"].append((yield dut.late))
            yield
            k += 1

    run_simulation(dut, bench())
    return rec


class TestSpacingQuantisation(unittest.TestCase):
    def test_half_chip_snaps_to_two_samples(self):
        # 0.5 chips at fs = 4 MHz is 1.955 samples; Tracking.jl rounds that to 2
        # samples = 0.5115 chips, and dll_disc normalises with (2 - 1.023)/2.
        ch, csr = channel()
        ch.set_spacing_chips(0.5)
        word = csr.written["gnss_ch0_spacing"]
        self.assertEqual(word, 2 * ch.code_word(0.0))
        self.assertEqual(ch.sample_shift(0.5), 2)
        # The old behaviour (raw preferred shift) is a different, wrong word.
        self.assertNotEqual(word, int(round(0.5 * (1 << FRAC))))
        self.assertAlmostEqual(word / (1 << FRAC),
                               2 * GPS_CA_CHIP_RATE / FS, places=6)

    def test_el_spacing_matches_tracking_jl_normalisation(self):
        # dll_disc's (2 - d)/2 uses d = 2*sample_shift*code_phase_delta chips;
        # the FPGA's actual E-L spacing (2*spacing) must equal that exactly.
        for fs in (4e6, 2.046e6, 5e6, 3.3e6, 8.184e6):
            ch, csr = channel(fs)
            ch.set_spacing_chips(0.5)
            word  = csr.written["gnss_ch0_spacing"]
            shift = ch.sample_shift(0.5)
            fpga_el   = 2 * word / (1 << FRAC)                    # chips
            julia_el  = 2 * shift * GPS_CA_CHIP_RATE / fs         # chips
            self.assertAlmostEqual(fpga_el, julia_el, places=5, msg=f"fs={fs}")

    def test_spacing_word_is_whole_number_of_samples(self):
        # spacing = sample_shift * code_step keeps the E tap exactly on a sample
        # boundary of the same NCO, with no rounding drift between the words.
        # (fs, preferred shift) pairs whose quantised spacing stays below a chip.
        for fs, pref in ((4e6, 0.25), (4e6, 0.5), (4e6, 0.75),
                         (2.046e6, 0.25), (2.046e6, 0.5),
                         (5e6, 0.5), (3.3e6, 0.5), (8.184e6, 0.75)):
            ch, csr = channel(fs)
            ch.set_spacing_chips(pref)
            word = csr.written["gnss_ch0_spacing"]
            self.assertEqual(word % ch.code_word(0.0), 0,
                             msg=f"fs={fs} pref={pref}")

    def test_sub_sample_shift_clamps_to_one_sample(self):
        # Tracking.jl's max(1, sample_shift): never collapse E/P/L onto one tap.
        ch, csr = channel(fs=2.046e6)       # 2 samples/chip
        ch.set_spacing_chips(0.1)           # 0.2 samples -> rounds to 0
        self.assertEqual(ch.sample_shift(0.1), 1)
        self.assertEqual(csr.written["gnss_ch0_spacing"], ch.code_word(0.0))

    def test_spacing_of_one_chip_or_more_rejected(self):
        # The E/L taps only reach idx +/- 1, so spacing must stay below a chip.
        ch, csr = channel()
        for pref in (0.9, 1.0, 2.0):
            with self.assertRaises(ValueError):
                ch.set_spacing_chips(pref)
        self.assertNotIn("gnss_ch0_spacing", csr.written)

    def test_configure_quantises_with_the_programmed_doppler(self):
        ch, csr = channel()
        ch.configure(prn=1, carrier_hz=1200.0, code_doppler_hz=1200.0, spacing=0.5)
        self.assertEqual(csr.written["gnss_ch0_spacing"],
                         2 * ch.code_word(1200.0))


class TestReplicaTapAlignment(unittest.TestCase):
    """The programmed word must put E/L exactly `sample_shift` samples away."""

    def _check(self, stb_gap, n):
        ch, csr = channel()
        ch.set_spacing_chips(0.5)
        step  = ch.code_word(0.0)
        shift = ch.sample_shift(0.5)
        rec   = replica_taps(step, csr.written["gnss_ch0_spacing"], n,
                             stb_gap=stb_gap)
        for i in range(shift, n - shift):
            self.assertEqual(rec["e"][i], rec["p"][i + shift], f"early[{i}]")
            self.assertEqual(rec["l"][i], rec["p"][i - shift], f"late[{i}]")

    def test_taps_land_on_whole_samples(self):
        self._check(stb_gap=0, n=2000)

    def test_taps_land_on_whole_samples_sparse_strobe(self):
        self._check(stb_gap=3, n=600)

    def test_raw_chip_spacing_is_not_sample_aligned(self):
        # Why the quantisation matters: the raw 0.5-chip word makes the E tap
        # advance 1.955 samples ahead, so it wobbles between 1 and 2 samples.
        ch, _ = channel()
        step = ch.code_word(0.0)
        rec  = replica_taps(step, int(round(0.5 * (1 << FRAC))), 2000)
        mismatches = sum(1 for i in range(2, 2000 - 2)
                         if rec["e"][i] != rec["p"][i + 2])
        self.assertGreater(mismatches, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
