#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Correlator accumulators must clamp, not wrap.

A wrapped accumulator is indistinguishable from a plausible correlator value on
the host, so a strong in-band interferer / badly set AD9361 gain silently
produces garbage that tracking happily consumes. These tests drive a channel
into overflow deterministically and pin the clamped value plus the saturation
flags.

The drive trick: freeze the code NCO (``code_freq``/``code_step`` = 0) so the
prompt chip stays at chip 0, and feed full-scale samples whose sign matches that
chip. Every product then adds the same positive (or negative) amount, which
walks the accumulator into the rail in a few hundred samples. Only afterwards is
the code NCO started, with zero samples, so the accumulators hold their value
until the code-period epoch dumps them.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH

FRAC        = 24
PHASE_BITS  = 32
ACCUM_BITS  = 32
PRN         = 1
AMP         = 32767                 # full-scale 16-bit sample
CARRIER_AMP = 127                   # cos(0) with carrier_amp_bits = 8

ACC_MAX =  (1 << (ACCUM_BITS - 1)) - 1
ACC_MIN = -(1 << (ACCUM_BITS - 1))

# Samples needed to walk a 32-bit accumulator past the positive rail.
STEP        = AMP * CARRIER_AMP
N_SATURATE  = ACC_MAX // STEP + 64   # comfortably past
N_HEADROOM  = 100                    # comfortably below

CODE_STEP_MAX = (1 << FRAC) - 1      # ~1 chip per sample: reach the epoch fast


def chip0(prn=PRN):
    """The +/-1 prompt chip the frozen code NCO sits on."""
    return 1 if ca_code_reference(prn)[0] else -1


def run_channel_to_dump(n_drive, sign=+1):
    """Drive `n_drive` sign-matched full-scale samples, then dump.

    Returns the dump fields plus the sticky saturation flag.
    """
    dut = TrackingChannel(prn=PRN, code_frac_bits=FRAC,
                          carrier_phase_bits=PHASE_BITS, accum_bits=ACCUM_BITS)
    out = {}

    def bench():
        yield dut.carrier_fw.eq(0)          # cos = +127, sin = 0
        yield dut.carrier_phase_in.eq(0)
        yield dut.spacing.eq(1 << (FRAC - 1))
        yield dut.code_step.eq(0)           # freeze the code phase
        yield dut.carrier_set.eq(1)
        yield dut.restart.eq(1)
        yield
        yield dut.carrier_set.eq(0)
        yield dut.restart.eq(0)
        # Phase 1: every product contributes sign * AMP * CARRIER_AMP.
        yield dut.sample_i.eq(sign * AMP * chip0())
        yield dut.sample_q.eq(0)
        yield dut.sample_stb.eq(1)
        for _ in range(n_drive):
            yield
        out["saturated"] = (yield dut.saturated)
        # Phase 2: zero samples (accumulators hold) and run the code NCO out to
        # the code-period epoch so the accumulators are dumped.
        yield dut.sample_i.eq(0)
        yield dut.code_step.eq(CODE_STEP_MAX)
        for _ in range(CA_CODE_LENGTH + 64):
            yield
            if (yield dut.dump_stb):
                out.update(ip=(yield dut.ip), qp=(yield dut.qp),
                           dump_saturated=(yield dut.dump_saturated))
                break
        yield dut.sample_stb.eq(0)
        yield

    run_simulation(dut, bench())
    return out


class TestAccumulatorSaturation(unittest.TestCase):
    def test_no_flag_and_exact_sum_within_range(self):
        out = run_channel_to_dump(N_HEADROOM)
        self.assertIn("ip", out, "no dump produced")
        self.assertEqual(out["saturated"], 0)
        self.assertEqual(out["dump_saturated"], 0)
        self.assertEqual(out["qp"], 0)
        # Exact: the frozen chip makes every product identical.
        self.assertEqual(out["ip"], N_HEADROOM * STEP)
        self.assertLess(out["ip"], ACC_MAX)

    def test_positive_overflow_clamps_instead_of_wrapping(self):
        out = run_channel_to_dump(N_SATURATE, sign=+1)
        self.assertIn("ip", out, "no dump produced")
        wrapped = N_SATURATE * STEP
        self.assertGreater(wrapped, ACC_MAX, "test drive does not overflow")
        self.assertEqual(out["ip"], ACC_MAX)
        self.assertEqual(out["saturated"], 1)
        self.assertEqual(out["dump_saturated"], 1)

    def test_negative_overflow_clamps_instead_of_wrapping(self):
        out = run_channel_to_dump(N_SATURATE, sign=-1)
        self.assertIn("ip", out, "no dump produced")
        self.assertEqual(out["ip"], ACC_MIN)
        self.assertEqual(out["saturated"], 1)
        self.assertEqual(out["dump_saturated"], 1)

    def test_restart_clears_the_sticky_flag(self):
        dut = TrackingChannel(prn=PRN, code_frac_bits=FRAC,
                              carrier_phase_bits=PHASE_BITS, accum_bits=ACCUM_BITS)
        seen = {}

        def bench():
            yield dut.carrier_fw.eq(0)
            yield dut.carrier_phase_in.eq(0)
            yield dut.spacing.eq(1 << (FRAC - 1))
            yield dut.code_step.eq(0)
            yield dut.carrier_set.eq(1)
            yield dut.restart.eq(1)
            yield
            yield dut.carrier_set.eq(0)
            yield dut.restart.eq(0)
            yield dut.sample_i.eq(AMP * chip0())
            yield dut.sample_stb.eq(1)
            for _ in range(N_SATURATE):
                yield
            seen["before"] = (yield dut.saturated)
            yield dut.sample_stb.eq(0)
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield
            seen["after"] = (yield dut.saturated)

        run_simulation(dut, bench())
        self.assertEqual(seen["before"], 1)
        self.assertEqual(seen["after"], 0)


class TestBankSaturationCSR(unittest.TestCase):
    """The sticky saturation bit must reach the host over CSR, per channel."""

    def test_saturation_csr_sets_and_clears(self):
        dut = GNSSTracking(n_channels=1, prns=[PRN], code_frac_bits=FRAC)
        chan = dut.ch0
        seen = {}

        def bench():
            yield chan._carrier_freq.storage.eq(0)
            yield chan._carrier_phase.storage.eq(0)
            yield chan._code_freq.storage.eq(0)      # freeze the code phase
            yield chan._spacing.storage.eq(1 << (FRAC - 1))
            yield dut._control.storage.eq(1)          # enable the bank
            yield dut.source.ready.eq(1)
            yield
            yield chan._control.storage.eq(0)
            yield
            yield chan._control.storage.eq(0b11)      # restart + carrier_set
            yield
            yield chan._control.storage.eq(0)
            yield
            seen["idle"] = (yield dut._saturation.status)
            yield dut.sample_i.eq(AMP * chip0())
            yield dut.sample_q.eq(0)
            yield dut.sample_stb.eq(1)
            for _ in range(N_SATURATE):
                yield
            seen["saturated"] = (yield dut._saturation.status)
            # Run the code NCO out to the epoch on zero samples so the clamped
            # accumulators are dumped, and check the per-dump flag too.
            yield dut.sample_i.eq(0)
            yield chan._code_freq.storage.eq(CODE_STEP_MAX)
            for _ in range(CA_CODE_LENGTH + 64):
                yield
                if (yield chan.channel.dump_stb):
                    break
            yield
            seen["dump_saturated"] = (yield chan._dump_saturated.status)
            seen["dump_ip"] = (yield chan._ip.status)
            yield dut.sample_stb.eq(0)
            yield
            yield chan._control.storage.eq(0b01)      # restart pulse
            yield
            yield chan._control.storage.eq(0)
            yield
            seen["after_restart"] = (yield dut._saturation.status)

        run_simulation(dut, bench())
        self.assertEqual(seen["idle"], 0)
        self.assertEqual(seen["saturated"], 1)
        self.assertEqual(seen["dump_saturated"], 1)
        # The readback CSR is unsigned; the clamped accumulator is +ACC_MAX.
        self.assertEqual(seen["dump_ip"], ACC_MAX)
        self.assertEqual(seen["after_restart"], 0)


class TestAccumWidthGuard(unittest.TestCase):
    """`record.py`'s s32() takes the low 32 bits unconditionally, and the CSR
    readback registers are 32 bits wide, so anything but accum_bits=32 loses
    the top bits silently. Fail the build instead."""

    def test_wider_accumulator_is_rejected(self):
        with self.assertRaises(AssertionError):
            GNSSTracking(n_channels=1, prns=[PRN], accum_bits=48)

    def test_narrower_accumulator_is_rejected(self):
        with self.assertRaises(AssertionError):
            GNSSTracking(n_channels=1, prns=[PRN], accum_bits=24)

    def test_default_accumulator_is_accepted(self):
        GNSSTracking(n_channels=1, prns=[PRN], accum_bits=32)


if __name__ == "__main__":
    unittest.main(verbosity=2)
