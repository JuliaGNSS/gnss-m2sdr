#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Host side of the sub-chip replica and the per-tap offsets.

These are plain functions of their inputs -- no board, no simulation -- but they
are the half of the contract that decides what the gateware is told. The tap
array in particular has to arrive exactly as GNSSReceiver hands it over (whole
input samples, latest first, prompt at zero); reversing it or collapsing it to a
single spacing is the failure mode the five-tap layout exists to rule out.
"""

import unittest

from gnss_m2sdr.record_format import TAPS_EPL, TAPS_VEPL, tap_layouts_mask
from gnss_m2sdr.subcarrier import (
    CBOC_A1, CBOC_A2, TMBOC_L1CP_PATTERN, lut_rms, signal_replica_shape,
)
from software.gnss_tracking import GNSSBank, GNSSChannel

FS   = 20e6          # 19.55 samples/chip: room for a 5-tap layout under a chip
FRAC = 24


class FakeCSR:
    def __init__(self, regs=None):
        self.written = {}
        self.writes  = []
        self.regs    = regs or {}

    def write(self, name, value):
        self.written[name] = value
        self.writes.append((name, value))

    def read(self, name):
        return self.regs.get(name, self.written.get(name, 0))


def channel(fs=FS, **kwargs):
    csr = FakeCSR()
    kwargs.setdefault("num_taps", TAPS_VEPL)
    kwargs.setdefault("max_subchips", 12)
    kwargs.setdefault("replica_bits", 8)
    return GNSSChannel(csr, fs, index=0, code_frac_bits=FRAC, **kwargs), csr


def signed(word, bits=FRAC + 1):
    return word - (1 << bits) if word & (1 << (bits - 1)) else word


class TestTapOffsetArray(unittest.TestCase):
    def test_the_contracts_array_reaches_the_right_registers(self):
        # tap_sample_shifts is latest first: [-VL, -L, 0, +E, +VE].
        ch, csr = channel()
        ch.set_tap_offsets([-12, -3, 0, 3, 12])
        step = ch.code_word(0.0)
        self.assertEqual(signed(csr.written["gnss_ch0_tap_offset_ve"]),  12 * step)
        self.assertEqual(signed(csr.written["gnss_ch0_tap_offset_e"]),    3 * step)
        self.assertEqual(signed(csr.written["gnss_ch0_tap_offset_l"]),   -3 * step)
        self.assertEqual(signed(csr.written["gnss_ch0_tap_offset_vl"]), -12 * step)
        self.assertNotIn("gnss_ch0_tap_offset_p", csr.written)

    def test_an_asymmetric_layout_is_programmed_as_given(self):
        ch, csr = channel()
        ch.set_tap_offsets([-10, -2, 0, 3, 14])
        step = ch.code_word(0.0)
        self.assertEqual(signed(csr.written["gnss_ch0_tap_offset_vl"]), -10 * step)
        self.assertEqual(signed(csr.written["gnss_ch0_tap_offset_ve"]),  14 * step)

    def test_a_three_tap_array_leaves_ve_vl_alone(self):
        ch, csr = channel()
        ch.set_tap_offsets([-3, 0, 3])
        self.assertEqual(set(csr.written),
                         {"gnss_ch0_tap_offset_e", "gnss_ch0_tap_offset_l"})

    def test_the_prompt_must_be_zero_and_in_the_middle(self):
        ch, _ = channel()
        with self.assertRaises(ValueError):
            ch.set_tap_offsets([-3, -1, 1, 1, 3])    # no prompt at the centre
        with self.assertRaises(ValueError):
            ch.set_tap_offsets([0, -3, 3])           # early-first, not latest
        with self.assertRaises(ValueError):
            ch.set_tap_offsets([-3, -1, 0, 1])       # not a layout the wire has

    def test_a_tap_a_whole_chip_out_is_refused(self):
        # The taps reach chip index +/-1 only; the gateware makes the same
        # refusal (code_status.replica_unsupported) rather than landing on the
        # wrong chip.
        ch, csr = channel(fs=4e6)                    # 3.91 samples/chip
        with self.assertRaises(ValueError):
            ch.set_tap_offsets([-4, -1, 0, 1, 4])
        self.assertNotIn("gnss_ch0_tap_offset_ve", csr.written)

    def test_tracking_default_shifts_fit(self):
        # VeryEarlyPromptLateCorrelator prefers 0.15 chips for E/L and 0.6 for
        # VE/VL; both stay inside max_tap_offset_chips = 1.0 at every rate the
        # code NCO can serve L1 at.
        for fs in (4e6, 5e6, 10e6, 20e6, 30.72e6):
            ch, csr = channel(fs)
            e  = ch.sample_shift(0.15)
            ve = ch.sample_shift(0.6)
            ch.set_tap_offsets([-ve, -e, 0, e, ve])
            self.assertLess(abs(signed(csr.written["gnss_ch0_tap_offset_ve"])),
                            1 << FRAC, msg=f"fs={fs}")


class TestReplicaProgramming(unittest.TestCase):
    def test_a_cboc_channel_writes_the_amplitude_bearing_table(self):
        ch, csr = channel()
        shape = ch.load_replica_shape("GalileoE1B")
        writes = [v for n, v in csr.writes if n == "gnss_ch0_subcarrier_load"]
        self.assertEqual(len(writes), 12)
        vals = [(w & 0xFF) - 256 if (w & 0x80) else (w & 0xFF) for w in writes]
        self.assertEqual(vals, shape.lut_a)
        self.assertEqual(sorted(set(abs(v) for v in vals)),
                         [CBOC_A1 - CBOC_A2, CBOC_A1 + CBOC_A2])
        # Five taps and 12 sub-chips staged for the next restart.
        self.assertEqual(csr.written["gnss_ch0_replica"], 12 | (1 << 4))

    def test_tmboc_writes_both_tables_and_the_per_chip_select(self):
        ch, csr = channel()
        shape = signal_replica_shape("GPSL1C_P", code_length=66)
        ch.load_replica_shape(shape)
        writes = [v for n, v in csr.writes if n == "gnss_ch0_subcarrier_load"]
        self.assertEqual(len(writes), 24)                      # table A and B
        self.assertEqual([(w >> 12) & 1 for w in writes], [0] * 12 + [1] * 12)
        # The select bits ride with the chips, one per chip.
        ch.load_code([1] * 66, select=shape.select)
        loads = [v for n, v in csr.writes if n == "gnss_ch0_code_load"][1:]
        self.assertEqual([(v >> 3) & 1 for v in loads], shape.select)
        self.assertEqual([(v >> 3) & 1 for v in loads][:33],
                         [int(b) for b in TMBOC_L1CP_PATTERN])

    def test_a_loc_channel_stays_three_taps_and_one_subchip(self):
        ch, csr = channel()
        ch.load_replica_shape("GPSL1CA")
        self.assertEqual(csr.written["gnss_ch0_replica"], 1)
        self.assertEqual(len([n for n, _ in csr.writes
                              if n == "gnss_ch0_subcarrier_load"]), 1)

    def test_five_taps_are_refused_on_a_three_tap_build(self):
        ch, csr = channel(num_taps=TAPS_EPL)
        with self.assertRaises(ValueError):
            ch.set_replica(subchips=12, num_taps=TAPS_VEPL)
        self.assertNotIn("gnss_ch0_replica", csr.written)

    def test_an_amplitude_that_does_not_fit_the_table_is_refused(self):
        ch, _ = channel(replica_bits=8)
        with self.assertRaises(ValueError):
            ch.write_subcarrier(0, 200)

    def test_select_bits_must_match_the_code_length(self):
        ch, _ = channel()
        with self.assertRaises(ValueError):
            ch.load_code([1] * 64, select=[0] * 33)

    def test_the_declared_amplitude_is_gnsssignals_own(self):
        # What the host divides out. Leaving replica_code_amplitude at its
        # default is correct *because* this equals get_code_amplitude.
        e1b = signal_replica_shape("GalileoE1B")
        self.assertAlmostEqual(e1b.code_amplitude, 19.924858845171276, places=9)
        self.assertEqual(lut_rms(signal_replica_shape("GalileoE1B_BOC11").lut_a),
                         1.0)


class TestCapabilityDecoding(unittest.TestCase):
    def test_a_five_tap_build_is_read_back_whole(self):
        csr = FakeCSR({
            "gnss_version": 3 | (2 << 8),
            "gnss_capabilities": (4 | (2 << 8) | (5 << 16) | (24 << 24)
                                  | (32 << 32) | (32 << 40) | (10230 << 48)),
            "gnss_signal_caps": (0b11111 | (1 << 8) | (1 << 16)
                                 | (tap_layouts_mask(TAPS_VEPL) << 17)
                                 | (12 << 21) | (8 << 29)),
        })
        caps = GNSSBank(csr).capabilities(fs=20e6)
        self.assertEqual(caps["num_taps"], TAPS_VEPL)
        self.assertEqual(caps["tap_layouts"], [TAPS_EPL, TAPS_VEPL])
        self.assertEqual(caps["max_subchips"], 12)
        self.assertEqual(caps["replica_bits"], 8)

    def test_the_bank_hands_a_channel_the_builds_limits(self):
        csr = FakeCSR({
            "gnss_version": 3 | (2 << 8),
            "gnss_capabilities": (4 | (1 << 8) | (5 << 16) | (24 << 24)
                                  | (32 << 32) | (32 << 40) | (4092 << 48)),
            "gnss_signal_caps": (0b11111 | (1 << 8) | (1 << 16)
                                 | (tap_layouts_mask(TAPS_VEPL) << 17)
                                 | (12 << 21) | (8 << 29)),
        })
        ch = GNSSBank(csr).channel(0, FS)
        self.assertEqual(ch.num_taps, TAPS_VEPL)
        self.assertEqual(ch.max_subchips, 12)
        self.assertEqual(ch.replica_bits(), 8)
        ch.set_replica(subchips=12, num_taps=TAPS_VEPL)      # must not raise

    def test_a_three_tap_build_refuses_a_five_tap_channel(self):
        csr = FakeCSR({
            "gnss_version": 3 | (2 << 8),
            "gnss_capabilities": (4 | (1 << 8) | (3 << 16) | (24 << 24)
                                  | (32 << 32) | (32 << 40) | (1023 << 48)),
            "gnss_signal_caps": (1 | (1 << 8) | (1 << 16)
                                 | (tap_layouts_mask(TAPS_EPL) << 17)
                                 | (1 << 21) | (2 << 29)),
        })
        ch = GNSSBank(csr).channel(0, FS)
        self.assertEqual(ch.num_taps, TAPS_EPL)
        with self.assertRaises(ValueError):
            ch.set_replica(subchips=1, num_taps=TAPS_VEPL)


if __name__ == "__main__":
    unittest.main(verbosity=2)
