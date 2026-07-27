#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""NCO updates and handover must land on a sample the host picked, not on
"whatever sample was in flight when the PCIe write landed".

GNSSReceiver.jl's NCOUpdate carries `apply_at_epoch` so the tracking loop's
transport delay is a known number of epochs, and Tracking.jl's "Transport delay"
section makes scheduling each update to a known future epoch the caller's
responsibility. Plain CSR writes cannot do that: each write takes effect at an
arbitrary sample offset, and the four writes of an acquisition handover (carrier
frequency/phase, code frequency/phase) land at four *different* offsets, so the
handover is neither atomic nor placeable on the sample axis.

Pinned here, for bank.py's staged `*_next` registers + `apply_at`/`apply`:
  * `apply_at` is the first input sample whose NCO advance uses the staged
    frequency words, and the first sample that sees a scheduled restart's loaded
    code phase / carrier phase -- all committed in one cycle,
  * the apply point is independent of when the host wrote (the immediate CSR
    path, kept for bring-up and sweeps, is not -- shown side by side),
  * a target that has already passed commits on the next sample and reports it
    (`late`, `applied_at`), instead of hanging on a compare that cannot match,
  * the selects decide what a commit takes, so a carrier-only update leaves the
    code rate alone,
  * staged values do not reach the channel before their apply point, and
    immediate writes still take effect immediately,
  * the host driver's `code_phase` word decodes to the phase the gateware loads.

Strobes are sparse here (one per `STROBE_GAP + 1` sys cycles) because that is
how the RFIC drives them on hardware: the commit compare has to hit a strobe
cycle, not just any cycle.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.code_replica import CodeReplica
from software.gnss_tracking import GNSSChannel      # host side; no device needed

FRAC       = 24
PHASE_BITS = 32
STROBE_GAP = 3          # idle sys cycles between sample strobes

CARRIER_A = 0x0100_0000
CARRIER_B = 0x0200_0000
CARRIER_C = 0x0300_0000
CODE_A    = 0x0040_0000  # 0.25 chip / sample
CODE_B    = 0x0050_0000

# apply bits (see bank.py).
A_ARM          = 1 << 0
A_RESTART      = 1 << 1
A_CARRIER_SET  = 1 << 2
A_CARRIER_FREQ = 1 << 3
A_CODE_FREQ    = 1 << 4

ARM_FREQS = A_ARM | A_CARRIER_FREQ | A_CODE_FREQ          # NCO update
ARM_ALL   = ARM_FREQS | A_RESTART | A_CARRIER_SET         # acquisition handover


def poke(csr, value):
    """One host CSR write: `storage` plus the one-cycle `re` strobe the bus gives."""
    yield csr.storage.eq(value)
    yield csr.re.eq(1)
    yield
    yield csr.re.eq(0)


def drive(dut, n_samples, actions, trace):
    """Feed `n_samples` sparse strobes, recording what each sample was processed with.

    `actions[k]` (a generator function) runs in the idle gap *before* the strobe
    carrying sample k, i.e. it models a host write completing between samples.
    """
    ch = dut.ch0.channel
    for k in range(n_samples):
        if k in actions:
            yield from actions[k]()
        for _ in range(STROBE_GAP):
            yield
        yield dut.sample_stb.eq(1)
        yield                                   # the strobe cycle for sample k
        # NCO state during that cycle: what sample k is actually processed with
        # (the counter reads k during its own strobe cycle).
        trace.append(dict(
            sample  = (yield dut.sample_count),
            carrier = (yield ch.carrier.phase),
            # One number for the code NCO position: chip_index.frac.
            code    = ((yield ch.code.chip_index) << FRAC) | (yield ch.code.code_frac),
        ))
        yield dut.sample_stb.eq(0)


def carrier_step(trace, k):
    """Phase increment that took sample k-1 to sample k."""
    return (trace[k]["carrier"] - trace[k - 1]["carrier"]) % (1 << PHASE_BITS)


def code_step(trace, k):
    return trace[k]["code"] - trace[k - 1]["code"]


def first_changed(trace, step, old):
    """First sample index whose advance no longer used the old frequency word."""
    for k in range(1, len(trace)):
        if step(trace, k) != old:
            return k
    return None


def configure(dut, carrier_fw=CARRIER_A, code_fw=CODE_A):
    """Static configuration through the immediate CSRs, then enable the bank."""
    yield from poke(dut.ch0._carrier_freq, carrier_fw)
    yield from poke(dut.ch0._code_freq, code_fw)
    yield from poke(dut.ch0._spacing, 1 << (FRAC - 1))
    yield from poke(dut._control, 1)


class TestScheduledApply(unittest.TestCase):
    N_SAMPLES = 20
    APPLY_AT  = 12

    def stage_freqs(self, dut, apply_at=None, flags=ARM_FREQS):
        def action():
            yield from poke(dut.ch0._carrier_freq_next, CARRIER_B)
            yield from poke(dut.ch0._code_freq_next, CODE_B)
            yield from poke(dut.ch0._apply_at, self.APPLY_AT if apply_at is None else apply_at)
            yield from poke(dut.ch0._apply, flags)
        return action

    def test_staged_update_takes_effect_exactly_at_apply_at(self):
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace, got = [], {}

        def bench():
            yield from configure(dut)
            yield from drive(dut, self.N_SAMPLES, {3: self.stage_freqs(dut)}, trace)
            got["status"]     = (yield dut.ch0._apply_status.status)
            got["applied_at"] = (yield dut.ch0._applied_at.status)

        run_simulation(dut, bench())
        self.assertEqual([t["sample"] for t in trace], list(range(self.N_SAMPLES)))
        for k in range(1, self.N_SAMPLES):
            new = k >= self.APPLY_AT
            self.assertEqual(carrier_step(trace, k), CARRIER_B if new else CARRIER_A,
                             f"carrier advance into sample {k}")
            self.assertEqual(code_step(trace, k), CODE_B if new else CODE_A,
                             f"code advance into sample {k}")
        # Both NCOs switched on the same sample: the commit is atomic.
        self.assertEqual(got["applied_at"], self.APPLY_AT)
        self.assertEqual(got["status"], 0)      # not armed any more, not late

    def test_apply_point_is_independent_of_when_the_host_wrote(self):
        # The defect, side by side with the fix: an immediate CSR write applies
        # one sample after it lands (so PCIe timing decides the apply point),
        # while a staged+armed update applies at apply_at whenever it was armed.
        def run(write_before_sample, scheduled):
            dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
            trace = []

            def immediate():
                yield from poke(dut.ch0._carrier_freq, CARRIER_B)

            def bench():
                yield from configure(dut)
                action = self.stage_freqs(dut) if scheduled else immediate
                yield from drive(dut, self.N_SAMPLES, {write_before_sample: action}, trace)

            run_simulation(dut, bench())
            return first_changed(trace, carrier_step, CARRIER_A)

        self.assertEqual(run(4, scheduled=False), 5)
        self.assertEqual(run(9, scheduled=False), 10)   # jitter: follows the write
        self.assertEqual(run(4, scheduled=True), self.APPLY_AT)
        self.assertEqual(run(9, scheduled=True), self.APPLY_AT)

    def test_scheduled_handover_loads_code_and_carrier_phase(self):
        # Acquisition handover: carrier frequency + phase and code frequency +
        # phase must all take hold on sample apply_at.
        CHIP, CHIP_FRAC = 511, 0x123456
        CARRIER_PHASE   = 0x1234_5678
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace = []

        def stage():
            yield from poke(dut.ch0._code_phase, (CHIP << FRAC) | CHIP_FRAC)
            yield from poke(dut.ch0._carrier_phase, CARRIER_PHASE)
            yield from poke(dut.ch0._carrier_freq_next, CARRIER_B)
            yield from poke(dut.ch0._code_freq_next, CODE_B)
            yield from poke(dut.ch0._apply_at, self.APPLY_AT)
            yield from poke(dut.ch0._apply, ARM_ALL)

        def bench():
            yield from configure(dut)
            yield from drive(dut, self.N_SAMPLES, {3: stage}, trace)

        run_simulation(dut, bench())
        # Sample apply_at-1 is still on the pre-handover grid ...
        self.assertEqual(trace[self.APPLY_AT - 1]["code"], (self.APPLY_AT - 1) * CODE_A)
        # ... and apply_at is the first sample on the acquired phase.
        self.assertEqual(trace[self.APPLY_AT]["code"], (CHIP << FRAC) | CHIP_FRAC)
        self.assertEqual(trace[self.APPLY_AT]["carrier"], CARRIER_PHASE)
        # From there both NCOs run at the staged rates.
        for k in range(self.APPLY_AT + 1, self.N_SAMPLES):
            self.assertEqual(carrier_step(trace, k), CARRIER_B)
            self.assertEqual(code_step(trace, k), CODE_B)

    def test_target_already_passed_commits_at_once_and_reports_late(self):
        # A target in the past must not wait for a 64-bit compare that can never
        # match again; it commits on the next sample and flags the violated
        # feedback delay so the loop filter can be told.
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace, got = [], {}

        def bench():
            yield from configure(dut)
            yield from drive(dut, self.N_SAMPLES,
                             {10: self.stage_freqs(dut, apply_at=3)}, trace)
            got["status"]     = (yield dut.ch0._apply_status.status)
            got["applied_at"] = (yield dut.ch0._applied_at.status)

        run_simulation(dut, bench())
        applied = first_changed(trace, carrier_step, CARRIER_A)
        self.assertEqual(got["status"], 0b10)          # not armed, late
        self.assertEqual(got["applied_at"], applied)
        self.assertGreater(applied, 3)

    def test_selects_decide_which_staged_values_the_commit_takes(self):
        # A carrier-only update must leave the code NCO alone -- otherwise every
        # arm would drag whatever is in code_freq_next (here: nothing, i.e. 0)
        # into the code rate and kill the channel.
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace = []

        def stage():
            yield from poke(dut.ch0._carrier_freq_next, CARRIER_B)
            yield from poke(dut.ch0._apply_at, self.APPLY_AT)
            yield from poke(dut.ch0._apply, A_ARM | A_CARRIER_FREQ)

        def bench():
            yield from configure(dut)
            yield from drive(dut, self.N_SAMPLES, {3: stage}, trace)

        run_simulation(dut, bench())
        self.assertEqual(first_changed(trace, carrier_step, CARRIER_A), self.APPLY_AT)
        self.assertIsNone(first_changed(trace, code_step, CODE_A))

    def test_host_code_phase_word_decodes_to_the_requested_phase(self):
        # The host driver and the gateware have to agree on the chip|frac layout
        # of code_phase; a mismatch is an E/L-swap-class bug (silently tracks the
        # wrong phase) rather than an obvious error.
        CHIPS = 511.25
        host  = GNSSChannel(None, fs=4.092e6, code_frac_bits=FRAC)
        dut   = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace = []

        def stage():
            yield from poke(dut.ch0._code_phase, host.code_phase_word(CHIPS))
            yield from poke(dut.ch0._apply_at, self.APPLY_AT)
            yield from poke(dut.ch0._apply, A_ARM | A_RESTART)

        def bench():
            yield from configure(dut)
            yield from drive(dut, self.N_SAMPLES, {3: stage}, trace)

        run_simulation(dut, bench())
        self.assertEqual(trace[self.APPLY_AT]["code"], round(CHIPS * (1 << FRAC)))

    def test_unarmed_staging_never_reaches_the_channel(self):
        # Preparing the next update must not perturb the loop: without an arm,
        # the staged words are invisible.
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace, got = [], {}

        def stage_only():
            yield from poke(dut.ch0._carrier_freq_next, CARRIER_B)
            yield from poke(dut.ch0._code_freq_next, CODE_B)
            yield from poke(dut.ch0._apply_at, self.APPLY_AT)

        def bench():
            yield from configure(dut)
            yield from drive(dut, self.N_SAMPLES, {3: stage_only}, trace)
            got["status"] = (yield dut.ch0._apply_status.status)

        run_simulation(dut, bench())
        self.assertEqual(got["status"], 0)
        self.assertIsNone(first_changed(trace, carrier_step, CARRIER_A))
        self.assertIsNone(first_changed(trace, code_step, CODE_A))

    def test_immediate_write_after_a_commit_takes_effect_at_once(self):
        # The committed word stays in force until the host writes the immediate
        # CSR again -- which then applies immediately, as it always did, and
        # staging a further update on top still waits for its own apply point.
        N = 24
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        trace = []

        def restage():
            yield from poke(dut.ch0._carrier_freq, CARRIER_C)     # immediate
            yield from poke(dut.ch0._carrier_freq_next, CARRIER_A)  # staged, unarmed

        def bench():
            yield from configure(dut)
            yield from drive(dut, N, {3: self.stage_freqs(dut), 16: restage}, trace)

        run_simulation(dut, bench())
        for k in range(1, N):
            if k < self.APPLY_AT:
                want = CARRIER_A
            elif k < 17:
                want = CARRIER_B
            else:
                want = CARRIER_C
            self.assertEqual(carrier_step(trace, k), want, f"advance into sample {k}")


class TestCodePhasePreload(unittest.TestCase):
    def test_restart_rebases_onto_the_staged_code_phase(self):
        # Handover needs restart to land on the acquired phase, not on chip 0:
        # loading chip 5 of an 8-chip code leaves 3 chips to the next epoch.
        CODE_LENGTH = 8
        CHIP, CHIP_FRAC = 5, 1 << (FRAC - 2)     # 5.25 chips
        STEP = 1 << (FRAC - 1)                   # 0.5 chip / sample
        dut = CodeReplica(prn=1, frac_bits=FRAC, code_length=CODE_LENGTH)
        got = {"phase": None, "epoch_after": None}

        def bench():
            yield dut.code_step.eq(STEP)
            yield dut.spacing.eq(1 << (FRAC - 1))
            yield dut.restart_chip.eq(CHIP)
            yield dut.restart_frac.eq(CHIP_FRAC)
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield
            for k in range(16):
                yield dut.stb.eq(1)
                yield                              # strobe cycle for sample k
                if got["phase"] is None:
                    got["phase"] = ((yield dut.chip_index) << FRAC) | (yield dut.code_frac)
                if (yield dut.epoch) and got["epoch_after"] is None:
                    got["epoch_after"] = k         # 0-based sample that wrapped
                yield dut.stb.eq(0)
                for _ in range(STROBE_GAP):        # sparse strobe, as on hardware
                    yield

        run_simulation(dut, bench())
        self.assertEqual(got["phase"], (CHIP << FRAC) | CHIP_FRAC)
        # (8 - 5.25) chips at 0.5 chip/sample: the 6th sample completes the code.
        self.assertEqual(got["epoch_after"], 5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
