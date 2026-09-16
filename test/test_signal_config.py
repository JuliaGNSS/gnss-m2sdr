#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Runtime signal configuration: code length, code rate, and what a dump says.

The gateware was a GPS L1 C/A correlator with the constants inlined: 1023 chips,
1.023 Mcps, a code phase reported as a fraction whose integer chip "must be"
1022. None of that survives contact with the rest of GNSSSignals -- Galileo E1 is
4092 chips, GPS L5 is 10230 at 10.23 Mcps, BeiDou B1I is 2046 -- and none of it
failed loudly when it was wrong. This file pins the parameterisation
(gnss-m2sdr#29, step 2 of GNSSReceiver.jl#130):

  * **code length is runtime state.** One build's channel replicates 330, 1023,
    2046, 4092, 5115 or 10230 chips, wrapping and dumping on its own period.
  * **arming is atomic.** A code load suppresses the channel's dumps until the
    next restart, which is also when the staged length is committed, so no
    record ever describes half of one satellite and half of another.
  * **an unrepresentable code rate is refused, not truncated.** The step word is
    one bit wider than the fraction so that >= 1 chip/sample *arrives*; the
    channel then reports `rate_error` and stops dumping. The host driver raises
    on the same condition. The old code masked the word, which turned 10.23 Mcps
    at fs = 4.092 MHz into a perfectly plausible 0.5 chips/sample.
  * **a record describes itself.** Format version, tap count, the complete code
    phase (chip *and* fraction), the code length and the code step it ran at.
  * **the build describes itself.** `gnss_version` / `gnss_capabilities` /
    `gnss_signal_caps` are what an adapter builds GNSSReceiver's
    `HardwareCorrelatorCapabilities` from.

The correlator arithmetic is checked against an exact software reference
(`software_channel` below) rather than a tolerance: it reproduces the gateware's
fixed-point NCOs, its sin/cos LUT and its E/P/L tap selection sample by sample,
so an off-by-one chip or a mis-scaled step shows up as an inequality rather than
as a slightly worse correlation.
"""

import math
import unittest

from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.carrier_nco import _sincos_tables
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.record_format import (
    CSR_LAYOUT_VERSION, MAX_SECONDARY_CODE_LENGTH, MODULATIONS, MOD_LOC,
    NUM_TAPS, RECORD_FORMAT_VERSION, RECORD_WORDS, code_chip_rate,
    code_phase_chips, pack_record, unpack_record,
)
from software.gnss_tracking import (
    AcquisitionResult, GNSSBank, GNSSChannel, peak_code_phase,
)
from test.test_bank_csr import (
    CTL_RESTART, CTL_CARRIER_SET, LOAD_DAT, LOAD_RESET, LOAD_WE,
    csr_write, pulse_control,
)

FRAC        = 24          # code_frac_bits
PHASE_BITS  = 32          # carrier_phase_bits
LUT_BITS    = 8
AMP_BITS    = 8
CARRIER_AMP = (1 << (AMP_BITS - 1)) - 1      # 127

# The primary-code lengths this step has to serve, ahead of the long-code work
# in GNSSReceiver.jl#133. 330 is the shortest, 10230 the longest.
SUPPORTED_LENGTHS = (330, 1023, 2046, 4092, 5115, 10230)


def pseudo_code(length, seed=0xACE1):
    """Deterministic 0/1 chip sequence of `length` chips.

    A stand-in for a primary code, not a model of one: what is under test here
    is the length/rate parameterisation, and the gateware replicates whatever
    chips the host wrote, so any balanced sequence exercises the same paths. The
    chips themselves are cross-checked against GNSSSignals.jl for GPS L1 C/A in
    test_ca_code_vs_gnsssignals.py, which stays the reference for code content.
    """
    x, out = seed, []
    for _ in range(length):
        bit = (x ^ (x >> 2) ^ (x >> 3) ^ (x >> 5)) & 1
        x = (x >> 1) | (bit << 15)
        out.append(x & 1)
    return out


def pm(bit):
    return 1 if bit else -1


# --- exact software model of one tracking channel ----------------------------

def software_channel(code, samples, code_step, spacing, carrier_fw,
                     restart_chip=0, restart_frac=0, frac_bits=FRAC,
                     phase_bits=PHASE_BITS, lut_bits=LUT_BITS, amp_bits=AMP_BITS):
    """Bit-exact reference for TrackingChannel: the dumps it must produce.

    Mirrors the gateware sample by sample -- carrier phase accumulator and LUT
    lookup, code accumulator, chip index wrap at `len(code)`, the Early/Late tap
    selection from the fractional phase, and the dump on the wrapping sample,
    which is *included* in the integration it closes.

    `samples` is a list of (I, Q) per antenna position, i.e. [[(i, q), ...], ...].
    Returns one dict per dump: the six accumulators per antenna plus the
    metadata the record carries.
    """
    sin_t, cos_t = _sincos_tables(lut_bits, amp_bits)
    n_ants = len(samples)
    n      = len(code)
    last   = n - 1
    frac, idx, phase = restart_frac, restart_chip, 0
    acc    = [dict(ie=0, qe=0, ip=0, qp=0, il=0, ql=0) for _ in range(n_ants)]
    nsamp  = 0
    dumps  = []
    for k in range(len(samples[0])):
        addr = phase >> (phase_bits - lut_bits)
        cos, sin = cos_t[addr], sin_t[addr]
        idx_next = 0 if idx >= last else idx + 1
        idx_prev = last if idx == 0 else idx - 1
        early_adv = (frac + spacing) >= (1 << frac_bits)
        late_ret  = frac < spacing
        e = pm(code[idx_next if early_adv else idx])
        p = pm(code[idx])
        l = pm(code[idx_prev if late_ret else idx])
        for a in range(n_ants):
            i, q = samples[a][k]
            i_bb = i * cos + q * sin
            q_bb = q * cos - i * sin
            acc[a]["ie"] += e * i_bb; acc[a]["qe"] += e * q_bb
            acc[a]["ip"] += p * i_bb; acc[a]["qp"] += p * q_bb
            acc[a]["il"] += l * i_bb; acc[a]["ql"] += l * q_bb
        nsamp += 1

        acc_next = frac + code_step
        carry    = acc_next >> frac_bits
        if carry and idx >= last:                    # epoch: this sample wraps
            dumps.append(dict(
                ants=[dict(a) for a in acc], n=nsamp,
                sample_index=k, code_chip=idx, code_phase=frac))
            acc  = [dict(ie=0, qe=0, ip=0, qp=0, il=0, ql=0) for _ in range(n_ants)]
            nsamp = 0
        frac = acc_next & ((1 << frac_bits) - 1)
        if carry:
            idx = idx_next
        phase = (phase + carrier_fw) & ((1 << phase_bits) - 1)
    return dumps


# Signal amplitude: small enough that a 10230-chip integration at 2.13
# samples/chip stays inside the 32-bit accumulators (127 * amp * N < 2**31).
# Saturation is a separate contract, covered by test_accum_saturation.py.
SIGNAL_AMP = 300


def synth_signal(code, n_samples, code_step, carrier_fw, amp=SIGNAL_AMP,
                 code_offset_chips=0.0, frac_bits=FRAC, phase_bits=PHASE_BITS,
                 gain=1.0, spatial_phase=0.0):
    """A BPSK satellite: `code` at `code_step` chips/sample on `carrier_fw`."""
    n = len(code)
    I, Q = [], []
    for k in range(n_samples):
        cp = (k * code_step / (1 << frac_bits) + code_offset_chips) % n
        chip = pm(code[int(cp)])
        theta = 2 * math.pi * carrier_fw * k / (1 << phase_bits) + spatial_phase
        I.append(int(round(amp * gain * chip * math.cos(theta))))
        Q.append(int(round(amp * gain * chip * math.sin(theta))))
    return list(zip(I, Q))


def run_channel(code, samples, code_step, spacing, carrier_fw, max_code_length,
                n_ants=1, code_length=None):
    """Drive a TrackingChannel over `samples`; return every dump it produced."""
    dut = TrackingChannel(code_frac_bits=FRAC, carrier_phase_bits=PHASE_BITS,
                          max_code_length=max_code_length, num_ants=n_ants,
                          code_init=code)
    dumps = []

    def bench():
        yield dut.code_step.eq(code_step)
        yield dut.code_length.eq(len(code) if code_length is None else code_length)
        yield dut.spacing.eq(spacing)
        yield dut.carrier_fw.eq(carrier_fw)
        yield dut.carrier_phase_in.eq(0)
        yield dut.carrier_set.eq(1)
        yield dut.restart.eq(1)
        yield
        yield dut.carrier_set.eq(0)
        yield dut.restart.eq(0)
        for k in range(len(samples[0])):
            for a in range(n_ants):
                yield dut.sample_i_ants[a].eq(samples[a][k][0])
                yield dut.sample_q_ants[a].eq(samples[a][k][1])
            yield dut.sample_stb.eq(1)
            yield
            if (yield dut.dump_stb):
                ants = []
                for x in dut.acc:
                    ants.append(dict(
                        ie=(yield x["ie"]), qe=(yield x["qe"]),
                        ip=(yield x["ip"]), qp=(yield x["qp"]),
                        il=(yield x["il"]), ql=(yield x["ql"])))
                dumps.append(dict(
                    ants=ants,
                    n=(yield dut.integrated_samples),
                    code_chip=(yield dut.dump_code_chip),
                    code_phase=(yield dut.dump_code_phase),
                    code_length=(yield dut.dump_code_length),
                    code_step=(yield dut.dump_code_step),
                ))

    run_simulation(dut, bench())
    return dumps


class TestRuntimeCodeLength(unittest.TestCase):
    """The replica wraps on the length it is told, not on 1023."""

    def _epochs(self, max_code_length, code_length, samples_per_chip, periods=2):
        dut = CodeReplica(frac_bits=FRAC, max_code_length=max_code_length,
                          code_init=pseudo_code(code_length))
        step = (1 << FRAC) // samples_per_chip
        n    = periods * samples_per_chip * code_length + 4
        rec  = []

        def bench():
            yield dut.code_step.eq(step)
            yield dut.code_length.eq(code_length)
            yield dut.spacing.eq(1 << (FRAC - 1))
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield
            for k in range(n):
                if (yield dut.epoch):
                    rec.append((k, (yield dut.chip_index)))
                yield

        run_simulation(dut, bench())
        return rec

    def test_epoch_period_follows_the_programmed_length(self):
        # One epoch per code period, for every length in scope. 2 samples/chip
        # keeps the longest case affordable in simulation.
        for length in SUPPORTED_LENGTHS:
            with self.subTest(code_length=length):
                epochs = self._epochs(length, length, samples_per_chip=2)
                self.assertGreaterEqual(len(epochs), 2, "no code wrap")
                self.assertEqual(epochs[1][0] - epochs[0][0], 2 * length)
                # ... and it wraps at the last chip of *this* code.
                self.assertEqual(epochs[0][1], length - 1)

    def test_one_build_serves_several_lengths(self):
        # The real claim: a single build (code RAM sized once) wraps wherever it
        # is told to, so channels of one bank can track different signals.
        for length in (330, 1023, 2046):
            with self.subTest(code_length=length):
                epochs = self._epochs(2046, length, samples_per_chip=2)
                self.assertEqual(epochs[1][0] - epochs[0][0], 2 * length)

    def test_prompt_reproduces_the_loaded_code_at_its_own_length(self):
        length, spc = 330, 4
        code = pseudo_code(length)
        dut  = CodeReplica(frac_bits=FRAC, max_code_length=2046, code_init=code)
        step = (1 << FRAC) // spc
        got  = []

        def bench():
            yield dut.code_step.eq(step)
            yield dut.code_length.eq(length)
            yield dut.spacing.eq(1 << (FRAC - 1))
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield
            for _ in range(spc * length):
                got.append((yield dut.prompt))
                yield

        run_simulation(dut, bench())
        for chip in range(length):
            self.assertEqual(got[chip * spc + spc // 2], pm(code[chip]),
                             f"chip {chip}")

    def test_shortening_the_length_wraps_on_the_next_chip(self):
        # A chip index left beyond the new last chip (a length change under a
        # running index, or a handover phase from the previous signal) must wrap
        # immediately. An `==` comparison would run a whole lap of the RAM
        # first: one code period of lost lock instead of one chip.
        length, spc = 16, 2
        dut = CodeReplica(frac_bits=FRAC, max_code_length=64,
                          code_init=pseudo_code(64))
        step = (1 << FRAC) // spc
        seen = []

        def bench():
            yield dut.code_step.eq(step)
            yield dut.code_length.eq(length)
            yield dut.spacing.eq(0)
            yield dut.restart_chip.eq(40)          # far past chip 15
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield
            for k in range(8):
                seen.append(((yield dut.chip_index), (yield dut.epoch)))
                yield

        run_simulation(dut, bench())
        idxs = [i for i, _ in seen]
        self.assertEqual(idxs[0], 40)
        self.assertIn(0, idxs[:4], f"index never wrapped: {idxs}")
        self.assertTrue(any(e for _, e in seen[:4]), "no epoch on the wrap")


class TestAgainstSoftwareReference(unittest.TestCase):
    """Complete E/P/L sums, for each supported length, at a non-integer number
    of samples per chip and a nonzero Doppler."""

    # 2.13 samples/chip: deliberately not an integer, so the fractional code
    # phase advances through every value and the E/L taps change chip on
    # different samples than the prompt does.
    SAMPLES_PER_CHIP = 2.13
    DOPPLER_FW = 0x0400_0000        # carrier phase increment / sample

    def _case(self, length, max_code_length=None, n_ants=1, offset=0.0):
        code      = pseudo_code(length)
        code_step = round((1 << FRAC) / self.SAMPLES_PER_CHIP)
        spacing   = code_step                     # E/L one NCO sample out
        n_samples = int(length * self.SAMPLES_PER_CHIP) + 8
        samples   = [synth_signal(code, n_samples, code_step, self.DOPPLER_FW,
                                  code_offset_chips=offset,
                                  gain=1.0 if a == 0 else 0.5,
                                  spatial_phase=0.0 if a == 0 else 0.7)
                     for a in range(n_ants)]
        want = software_channel(code, samples, code_step, spacing, self.DOPPLER_FW)
        got  = run_channel(code, samples, code_step, spacing, self.DOPPLER_FW,
                           max_code_length or length, n_ants=n_ants)
        self.assertTrue(want, "the reference produced no dump")
        self.assertEqual(len(got), len(want), "wrong number of dumps")
        return code, code_step, got, want

    def test_every_supported_length_matches_the_reference(self):
        for length in SUPPORTED_LENGTHS:
            with self.subTest(code_length=length):
                code, code_step, got, want = self._case(length)
                for g, w in zip(got, want):
                    self.assertEqual(g["ants"][0], w["ants"][0])
                    self.assertEqual(g["n"], w["n"])
                    self.assertEqual(g["code_chip"], w["code_chip"])
                    self.assertEqual(g["code_phase"], w["code_phase"])
                    self.assertEqual(g["code_length"], length)
                    self.assertEqual(g["code_step"], code_step)
                # The dump also has to be a *correlation*: aligned prompt above
                # both shifted taps, or the arithmetic agrees on nonsense.
                a = got[0]["ants"][0]
                power = lambda i, q: i * i + q * q
                self.assertGreater(power(a["ip"], a["qp"]), power(a["ie"], a["qe"]))
                self.assertGreater(power(a["ip"], a["qp"]), power(a["il"], a["ql"]))

    def test_two_antennas_match_the_reference_on_a_non_1023_code(self):
        _, _, got, want = self._case(2046, n_ants=2)
        for g, w in zip(got, want):
            self.assertEqual(g["ants"], w["ants"])
        # Shared replicas, independent accumulators: antenna 1 sees the same
        # satellite at half the amplitude and a different spatial phase.
        a0, a1 = got[0]["ants"]
        self.assertNotEqual(a0, a1)

    def test_fractional_code_phase_offset_matches_the_reference(self):
        # A satellite that is not chip-aligned with the replica: E/P/L split.
        _, _, got, want = self._case(1023, offset=0.4)
        self.assertEqual(got[0]["ants"][0], want[0]["ants"][0])
        a = got[0]["ants"][0]
        self.assertNotEqual(a["ie"], a["il"], "E/L balanced on a shifted code")

    def test_handover_restarts_on_the_measured_phase_of_a_short_code(self):
        # The acquisition handover for a non-1023 code: restart at chip 97.25 of
        # a 330-chip code, and the first epoch must land 330 - 97.25 chips later.
        length, chip, frac = 330, 97, 1 << (FRAC - 2)
        code      = pseudo_code(length)
        code_step = round((1 << FRAC) / self.SAMPLES_PER_CHIP)
        n_samples = int(length * self.SAMPLES_PER_CHIP) + 8
        samples   = [synth_signal(code, n_samples, code_step, self.DOPPLER_FW)]
        want = software_channel(code, samples, code_step, code_step,
                                self.DOPPLER_FW, restart_chip=chip,
                                restart_frac=frac)

        dut = TrackingChannel(code_frac_bits=FRAC, carrier_phase_bits=PHASE_BITS,
                              max_code_length=length, code_init=code)
        got = []

        def bench():
            yield dut.code_step.eq(code_step)
            yield dut.code_length.eq(length)
            yield dut.spacing.eq(code_step)
            yield dut.carrier_fw.eq(self.DOPPLER_FW)
            yield dut.code_phase_chip.eq(chip)
            yield dut.code_phase_frac.eq(frac)
            yield dut.carrier_set.eq(1)
            yield dut.restart.eq(1)
            yield
            yield dut.carrier_set.eq(0)
            yield dut.restart.eq(0)
            for k in range(n_samples):
                yield dut.sample_i_ants[0].eq(samples[0][k][0])
                yield dut.sample_q_ants[0].eq(samples[0][k][1])
                yield dut.sample_stb.eq(1)
                yield
                if (yield dut.dump_stb):
                    got.append(dict(n=(yield dut.integrated_samples),
                                    code_chip=(yield dut.dump_code_chip)))

        run_simulation(dut, bench())
        self.assertTrue(want and got, "no dump after the handover")
        self.assertEqual(got[0]["n"], want[0]["n"])
        self.assertEqual(got[0]["code_chip"], length - 1)
        # The first integration is the *tail* of the code -- the 232.75 chips
        # from the handover phase to the wrap -- not a whole period. Starting at
        # chip 0 regardless (the old behaviour of a restart) would make it 330.
        tail = (length - chip - frac / float(1 << FRAC)) * self.SAMPLES_PER_CHIP
        self.assertLess(got[0]["n"], int(length * self.SAMPLES_PER_CHIP))
        self.assertLessEqual(abs(got[0]["n"] - tail), 1.0)


class TestAtomicArming(unittest.TestCase):
    """A code load is armed by `restart`, and produces no records before it."""

    LENGTH = 330
    SPC    = 2

    def _bank(self):
        return GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC,
                            max_code_length=2046)

    def _run(self, restart_after_load):
        length, spc = self.LENGTH, self.SPC
        code = pseudo_code(length)
        dut  = self._bank()
        step = (1 << FRAC) // spc
        out  = {}

        def bench():
            yield dut.ch0._code_freq.storage.eq(step)
            yield dut.ch0._spacing.storage.eq(step)
            yield dut.ch0._carrier_freq.storage.eq(0)   # cos = 127, sin = 0
            yield dut._control.storage.eq(1)            # enable bank
            yield from pulse_control(dut.ch0, CTL_RESTART | CTL_CARRIER_SET)
            # Let a full period run with the built-in (1023-chip) code, so the
            # channel is demonstrably dumping before the load starts.
            for _ in range(spc * CA_CODE_LENGTH + 4):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            out["before"] = (yield dut.ch0._dump_count.status)

            # Re-assign: stage the new length, stream the new code in.
            yield from csr_write(dut.ch0._code_length, length)
            yield from csr_write(dut.ch0._code_load, LOAD_RESET)
            out["loading"] = (yield dut.ch0._code_status.status) & 1
            for bit in code:
                yield from csr_write(dut.ch0._code_load,
                                     LOAD_WE | (LOAD_DAT if bit else 0))
            # Run long enough that the *old* configuration would have dumped.
            for _ in range(spc * CA_CODE_LENGTH + 4):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            out["during"] = (yield dut.ch0._dump_count.status)
            out["length_before_restart"] = (yield dut.ch0._code_length_active.status)

            if restart_after_load:
                yield from pulse_control(dut.ch0, CTL_RESTART)
                out["length_after_restart"] = (yield dut.ch0._code_length_active.status)
                out["loading_after"] = (yield dut.ch0._code_status.status) & 1
                for _ in range(spc * length + 4):
                    yield dut.sample_stb.eq(1)
                    yield
                yield dut.sample_stb.eq(0)
                out["after"] = (yield dut.ch0._dump_count.status)
                out["n"] = (yield dut.ch0._integrated_samples.status)
                out["chip"] = (yield dut.ch0._dump_code_chip.status)

        run_simulation(dut, bench())
        return out

    def test_a_code_load_suppresses_dumps_until_the_restart_arms_it(self):
        out = self._run(restart_after_load=True)
        self.assertGreater(out["before"], 0, "channel never dumped to begin with")
        self.assertTrue(out["loading"], "code_status.loading not raised by the load")
        self.assertEqual(out["during"], out["before"],
                         "a record was emitted from a half-written code")
        # The staged length is not in force until the restart commits it.
        self.assertEqual(out["length_before_restart"], CA_CODE_LENGTH)
        self.assertEqual(out["length_after_restart"], self.LENGTH)
        self.assertFalse(out["loading_after"], "restart did not clear loading")
        self.assertGreater(out["after"], out["during"],
                           "no record after the channel was armed")
        self.assertEqual(out["n"], self.SPC * self.LENGTH)
        self.assertEqual(out["chip"], self.LENGTH - 1)

    def test_without_the_restart_the_channel_stays_silent(self):
        out = self._run(restart_after_load=False)
        self.assertEqual(out["during"], out["before"])


class TestCodeRateLimits(unittest.TestCase):
    """>= 1 chip per input sample is refused on both sides of the CSR."""

    def test_gateware_reports_rate_error_and_stops_dumping(self):
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC,
                           max_code_length=1023)
        good = (1 << FRAC) // 2                   # 0.5 chips/sample
        bad  = 1 << FRAC                          # exactly 1 chip/sample
        out  = {}

        def bench():
            yield dut.ch0._code_freq.storage.eq(bad)
            yield dut.ch0._spacing.storage.eq(good)
            yield dut.ch0._carrier_freq.storage.eq(0)
            yield dut._control.storage.eq(1)
            yield from pulse_control(dut.ch0, CTL_RESTART)
            for _ in range(4 * CA_CODE_LENGTH):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            out["dumps_bad"]  = (yield dut.ch0._dump_count.status)
            out["status_bad"] = (yield dut.ch0._code_status.status)
            out["mask_bad"]   = (yield dut._rate_error.status)

            # Program a representable rate and re-arm: the sticky bit clears and
            # records come back.
            yield dut.ch0._code_freq.storage.eq(good)
            yield from pulse_control(dut.ch0, CTL_RESTART)
            out["mask_after_restart"] = (yield dut._rate_error.status)
            for _ in range(2 * CA_CODE_LENGTH + 4):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            out["dumps_good"]  = (yield dut.ch0._dump_count.status)
            out["status_good"] = (yield dut.ch0._code_status.status)

        run_simulation(dut, bench())
        self.assertEqual(out["dumps_bad"], 0,
                         "a dump was produced at an unrepresentable code rate")
        self.assertEqual(out["status_bad"] >> 1 & 1, 1, "rate_unsupported not set")
        self.assertEqual(out["mask_bad"] & 1, 1, "sticky rate_error not set")
        self.assertEqual(out["mask_after_restart"] & 1, 0,
                         "rate_error survived the restart that fixed it")
        self.assertEqual(out["status_good"] >> 1 & 1, 0)
        self.assertGreater(out["dumps_good"], 0, "no dump after the rate was fixed")

    def test_host_refuses_an_unrepresentable_chip_rate(self):
        # GPS L5 (10.23 Mcps) at the L1 C/A sample rate. The old code masked the
        # word, yielding 0.5 chips/sample -- a channel that would have armed,
        # correlated noise and never locked, with nothing to point at.
        chan = GNSSChannel(_NullCSR(), fs=4.092e6, code_length=10230,
                           chip_rate=10.23e6, carrier_freq=1176.45e6)
        with self.assertRaises(ValueError) as cm:
            chan.code_word()
        self.assertIn("chips/sample", str(cm.exception))
        # ... and accepts it once the board is sampling fast enough.
        fast = GNSSChannel(_NullCSR(), fs=30.72e6, code_length=10230,
                           chip_rate=10.23e6, carrier_freq=1176.45e6)
        self.assertEqual(fast.code_word(),
                         round(10.23e6 / 30.72e6 * (1 << FRAC)))

    def test_host_scales_code_doppler_with_the_signals_own_carrier(self):
        # fc = chip_rate * (1 + fd / f_carrier): using L1's 1575.42 MHz for a
        # 1176.45 MHz L5 channel mis-scales the code Doppler by 34 %.
        chan = GNSSChannel(_NullCSR(), fs=30.72e6, code_length=10230,
                           chip_rate=10.23e6, carrier_freq=1176.45e6)
        want = 10.23e6 * (1.0 + 3000.0 / 1176.45e6) / 30.72e6 * (1 << FRAC)
        self.assertEqual(chan.code_word(3000.0), round(want))
        self.assertNotEqual(chan.code_word(3000.0), chan.code_word(0.0))

    def test_host_code_word_still_matches_gps_l1ca(self):
        chan = GNSSChannel(_NullCSR(), fs=4.092e6)
        self.assertEqual(chan.code_word(), round(1.023e6 / 4.092e6 * (1 << FRAC)))


class _NullCSR:
    """CSR stand-in for driver arithmetic that touches no register."""
    regs = {}

    def write(self, name, value):
        self.regs[name] = value

    def read(self, name):
        return self.regs.get(name, 0)


class TestRecordDescribesItsSignal(unittest.TestCase):
    """The DMA record carries the configuration the dump came from."""

    def test_round_trip_through_pack_unpack(self):
        words = pack_record(
            sample_index=1234, integrated_samples=4092, channel=2, prn=17,
            seq=5, flags=0, i_early=1, q_early=2, i_prompt=3, q_prompt=4,
            i_late=5, q_late=6, code_phase=0x00ABCDEF,
            code_phase_chip=4091, code_length=4092, code_step=0x00400000)
        rec = unpack_record(words)
        self.assertEqual(rec["version"], RECORD_FORMAT_VERSION)
        self.assertEqual(rec["num_taps"], NUM_TAPS)
        self.assertEqual(rec["code_phase_chip"], 4091)
        self.assertEqual(rec["code_length"], 4092)
        self.assertEqual(rec["code_step"], 0x00400000)
        # The complete code phase, with no assumption about where the dump fell.
        self.assertAlmostEqual(code_phase_chips(rec, FRAC),
                               4091 + 0x00ABCDEF / float(1 << FRAC), places=9)
        self.assertAlmostEqual(code_chip_rate(rec, FRAC, 4.092e6),
                               0.25 * 4.092e6, places=3)

    def test_version_1_records_still_parse_and_refuse_to_be_guessed(self):
        # A pre-#29 record: the new fields read zero. It must keep parsing (the
        # layout only grew into reserved words), and the code phase must raise
        # rather than silently report chip 0.
        words = pack_record(
            sample_index=7, integrated_samples=4092, channel=0, prn=1, seq=0,
            flags=0, i_early=0, q_early=0, i_prompt=0, q_prompt=0, i_late=0,
            q_late=0, code_phase=42, version=1, num_taps=0)
        rec = unpack_record(words)
        self.assertEqual(rec["version"], 1)
        self.assertEqual(rec["code_phase"], 42)
        self.assertEqual(rec["sample_index"], 7)
        with self.assertRaises(ValueError):
            code_phase_chips(rec, FRAC)

    def test_gateware_record_carries_the_live_configuration(self):
        length, spc = 330, 2
        code = pseudo_code(length)
        dut  = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC,
                            max_code_length=2046)
        step = (1 << FRAC) // spc
        words = []

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.ch0._code_freq.storage.eq(step)
            yield dut.ch0._spacing.storage.eq(step)
            yield dut.ch0._carrier_freq.storage.eq(0)
            yield dut._control.storage.eq(1)
            yield from csr_write(dut.ch0._code_length, length)
            yield from csr_write(dut.ch0._code_load, LOAD_RESET)
            for bit in code:
                yield from csr_write(dut.ch0._code_load,
                                     LOAD_WE | (LOAD_DAT if bit else 0))
            yield from pulse_control(dut.ch0, CTL_RESTART | CTL_CARRIER_SET)
            for _ in range(spc * length + 40):
                yield dut.sample_stb.eq(1)
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield
            yield dut.sample_stb.eq(0)
            for _ in range(2 * RECORD_WORDS):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield

        run_simulation(dut, bench())
        recs = [unpack_record(words[i:i + RECORD_WORDS])
                for i in range(0, len(words) - RECORD_WORDS + 1, RECORD_WORDS)]
        self.assertTrue(recs, "no record reached the DMA stream")
        rec = recs[0]
        self.assertEqual(rec["version"], RECORD_FORMAT_VERSION)
        self.assertEqual(rec["num_taps"], NUM_TAPS)
        self.assertEqual(rec["code_length"], length)
        self.assertEqual(rec["code_step"], step)
        self.assertEqual(rec["code_phase_chip"], length - 1)
        self.assertEqual(rec["integrated_samples"], spc * length)
        # ... and the chip index is the *code's* last chip, not 1022.
        self.assertNotEqual(rec["code_phase_chip"], CA_CODE_LENGTH - 1)


class TestCapabilityCSRs(unittest.TestCase):
    """The build describes itself, so the adapter does not have to assume."""

    def test_capability_csrs_report_the_build(self):
        dut = GNSSTracking(n_channels=3, prns=[1, 2, 3], code_frac_bits=20,
                           num_ants=2, max_code_length=4092)
        got = {}

        def bench():
            got["version"]  = (yield dut._version.status)
            got["caps"]     = (yield dut._capabilities.status)
            got["sig"]      = (yield dut._signal_caps.status)

        run_simulation(dut, bench())
        self.assertEqual(got["version"] & 0xFF, CSR_LAYOUT_VERSION)
        self.assertEqual((got["version"] >> 8) & 0xFF, RECORD_FORMAT_VERSION)

        caps = got["caps"]
        field = lambda v, sh, w: (v >> sh) & ((1 << w) - 1)
        self.assertEqual(field(caps, 0, 8), 3)          # n_channels
        self.assertEqual(field(caps, 8, 8), 2)          # num_ants_max
        self.assertEqual(field(caps, 16, 8), NUM_TAPS)
        self.assertEqual(field(caps, 24, 8), 20)        # code_frac_bits
        self.assertEqual(field(caps, 32, 8), 32)        # carrier_phase_bits
        self.assertEqual(field(caps, 40, 8), 32)        # accum_bits
        self.assertEqual(field(caps, 48, 16), 4092)     # max_code_length

        sig = got["sig"]
        self.assertEqual(field(sig, 0, 8), MODULATIONS)
        self.assertEqual(field(sig, 0, 8) & MOD_LOC, MOD_LOC)
        self.assertEqual(field(sig, 8, 8), MAX_SECONDARY_CODE_LENGTH)
        self.assertEqual(field(sig, 16, 1), 1)          # reports_code_phase

    def test_the_csr_names_the_adapter_addresses_exist(self):
        # GNSSM2SDR.jl#8 addresses these by name out of csr.csv. A rename is a
        # silent break there, so the set is pinned here (the SoC adds the
        # `gnss_` prefix of the bank submodule).
        dut = GNSSTracking(n_channels=2, prns=[1, 2], code_frac_bits=FRAC,
                           max_code_length=2046)
        names = {c.name for c in dut.get_csrs()}
        for name in ("version", "capabilities", "signal_caps", "rate_error",
                     "num_ants", "saturation", "overflow", "epoch_period",
                     "sample_count",
                     "ch0_code_freq", "ch0_code_freq_next", "ch0_code_length",
                     "ch0_code_length_active", "ch0_code_status",
                     "ch0_code_load", "ch0_code_phase", "ch0_spacing",
                     "ch0_dump_code_phase", "ch0_dump_code_chip",
                     "ch1_code_length"):
            self.assertIn(name, names)
        # The code-rate register has to carry the overflow bit, or an
        # unrepresentable rate cannot even be written, let alone rejected.
        width = {c.name: c.size for c in dut.get_csrs()}
        self.assertEqual(width["ch0_code_freq"], FRAC + 1)
        self.assertEqual(width["ch0_code_freq_next"], FRAC + 1)

    def test_declared_modulations_stay_honest(self):
        # An over-declared capability is a channel that arms and never locks, so
        # the BOC/CBOC/TMBOC bits must stay clear until #30 implements them.
        self.assertEqual(MODULATIONS, MOD_LOC)

    def test_host_reads_the_capabilities_back(self):
        csr = _NullCSR()
        csr.regs = {
            "gnss_version": CSR_LAYOUT_VERSION | (RECORD_FORMAT_VERSION << 8),
            "gnss_capabilities": (4 | (2 << 8) | (3 << 16) | (24 << 24)
                                  | (32 << 32) | (32 << 40) | (10230 << 48)),
            "gnss_signal_caps": MOD_LOC | (1 << 8) | (1 << 16),
        }
        caps = GNSSBank(csr).capabilities(fs=30.72e6)
        self.assertEqual(caps["n_channels"], 4)
        self.assertEqual(caps["num_ants_max"], 2)
        self.assertEqual(caps["num_taps"], 3)
        self.assertEqual(caps["max_code_length"], 10230)
        self.assertEqual(caps["modulations"], MOD_LOC)
        self.assertEqual(caps["max_secondary_code_length"], 1)
        self.assertTrue(caps["reports_code_phase"])
        self.assertEqual(caps["max_tap_offset_chips"], 1.0)
        lo, hi = caps["code_frequency_limits"]
        self.assertAlmostEqual(lo, 30.72e6 / (1 << 24))
        self.assertLess(hi, 30.72e6)
        self.assertGreater(hi, 30.72e6 * 0.999)

    def test_host_refuses_a_newer_csr_layout(self):
        csr = _NullCSR()
        csr.regs = {"gnss_version": (CSR_LAYOUT_VERSION + 1)
                                    | (RECORD_FORMAT_VERSION << 8)}
        with self.assertRaises(RuntimeError):
            GNSSBank(csr).capabilities()


class TestHostCodePhase(unittest.TestCase):
    """No 1022 anywhere on the host side either."""

    def test_peak_code_phase_uses_the_reported_chip(self):
        dump = dict(code_chip=329, code_phase=1 << (FRAC - 2))
        self.assertAlmostEqual(peak_code_phase(dump, FRAC), 329.25, places=9)

    def test_peak_code_phase_refuses_to_invent_a_chip(self):
        with self.assertRaises(KeyError):
            peak_code_phase(dict(code_phase=0), FRAC)
        # Explicitly asking for the legacy inference still works, for a v1 board.
        self.assertAlmostEqual(
            peak_code_phase(dict(code_phase=0), FRAC, code_length=1023), 1022.0)

    def test_code_phase_word_wraps_on_the_configured_length(self):
        chan = GNSSChannel(_NullCSR(), fs=8.184e6, code_length=4092,
                           chip_rate=1.023e6)
        word = chan.code_phase_word(4092.5)      # one full code past the start
        self.assertEqual(word >> FRAC, 0)
        self.assertEqual(word & ((1 << FRAC) - 1), 1 << (FRAC - 1))

    def test_acquisition_result_propagates_on_its_own_signal(self):
        res = AcquisitionResult(metric=50.0, doppler_hz=1000.0, power=1.0,
                                code_phase=10.0, sample_index=0, detected=True,
                                code_length=4092, chip_rate=1.023e6,
                                carrier_freq=1575.42e6)
        fs = 8.184e6
        fc = 1.023e6 * (1 + 1000.0 / 1575.42e6)
        n  = 100_000
        self.assertAlmostEqual(res.code_phase_at(n, fs),
                               (10.0 + n * fc / fs) % 4092, places=6)
        # The historical positional order is untouched.
        self.assertEqual(res[:3], (50.0, 1000.0, 1.0))

    def test_load_code_stages_the_length_it_loaded(self):
        csr  = _NullCSR()
        csr.regs = {}
        chan = GNSSChannel(csr, fs=4.092e6, index=0)
        chan.load_code(pseudo_code(2046), prn=7)
        self.assertEqual(csr.regs["gnss_ch0_code_length"], 2046)
        self.assertEqual(csr.regs["gnss_ch0_prn"], 7)
        self.assertEqual(chan.code_length, 2046)
        # A bare PRN still means GPS L1 C/A, 1023 chips.
        chan.load_code(11)
        self.assertEqual(csr.regs["gnss_ch0_code_length"], CA_CODE_LENGTH)


if __name__ == "__main__":
    unittest.main(verbosity=2)
