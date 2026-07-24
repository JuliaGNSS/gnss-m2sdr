#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""End-to-end lock test: drive TrackingChannel with a synthetic GPS L1 C/A
signal and check the correlators behave (prompt peaks, E/L balanced at
alignment, and the E-L discriminator responds correctly to a code offset)."""

import math
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH

# Signal / sampling parameters.
CHIP_RATE   = 1.023e6
SAMPLES_PER_CHIP = 4
FS          = SAMPLES_PER_CHIP * CHIP_RATE   # 4.092 MHz
F_IF        = FS / 16                          # residual carrier (Hz)
AMP         = 1000                             # signal amplitude (12-bit safe)
CARRIER_AMP = 127
FRAC        = 24
PHASE_BITS  = 32


def synth_signal(prn, code_offset_chips=0.0, carrier_phase0=0.0, n=None):
    """Complex baseband GPS L1 C/A samples: I=A*code*cos, Q=A*code*sin."""
    if n is None:
        # A few extra samples so the code-period epoch (and its dump) is
        # captured despite the one-cycle simulation-harness offset.
        n = SAMPLES_PER_CHIP * CA_CODE_LENGTH + 16
    code = [1 if b else -1 for b in ca_code_reference(prn)]
    I, Q = [], []
    for k in range(n):
        cp = (k * CHIP_RATE / FS + code_offset_chips) % CA_CODE_LENGTH
        chip = code[int(math.floor(cp))]
        theta = 2 * math.pi * F_IF * k / FS + carrier_phase0
        I.append(int(round(AMP * chip * math.cos(theta))))
        Q.append(int(round(AMP * chip * math.sin(theta))))
    return I, Q


def run_channel(prn, I, Q, spacing_chips=0.5):
    dut = TrackingChannel(prn=prn, code_frac_bits=FRAC, carrier_phase_bits=PHASE_BITS)
    carrier_fw = round(F_IF / FS * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
    code_step  = round(CHIP_RATE / FS * (1 << FRAC))
    spacing    = round(spacing_chips * (1 << FRAC))
    dump = {}

    def bench():
        yield dut.carrier_fw.eq(carrier_fw)
        yield dut.code_step.eq(code_step)
        yield dut.spacing.eq(spacing)
        yield dut.carrier_phase_in.eq(0)
        yield dut.carrier_set.eq(1)
        yield dut.restart.eq(1)
        yield
        yield dut.carrier_set.eq(0)
        yield dut.restart.eq(0)
        for k in range(len(I)):
            yield dut.sample_i.eq(I[k])
            yield dut.sample_q.eq(Q[k])
            yield dut.sample_stb.eq(1)
            yield
            if (yield dut.dump_stb):
                dump.update(
                    ie=(yield dut.ie), qe=(yield dut.qe),
                    ip=(yield dut.ip), qp=(yield dut.qp),
                    il=(yield dut.il), ql=(yield dut.ql),
                    n=(yield dut.integrated_samples),
                )
                break

    run_simulation(dut, bench())
    return dump


def mag(i, q):
    return math.hypot(i, q)


class TestChannelLock(unittest.TestCase):
    def test_locks_at_alignment(self):
        prn = 5
        I, Q = synth_signal(prn, code_offset_chips=0.0)
        d = self.assertDump(run_channel(prn, I, Q))
        n = d["n"]
        p, e, l = mag(d["ip"], d["qp"]), mag(d["ie"], d["qe"]), mag(d["il"], d["ql"])

        # Prompt in-phase energy ~ CARRIER_AMP * AMP * N (matched carrier, phi=0).
        expected_ip = CARRIER_AMP * AMP * n
        self.assertGreater(d["ip"], 0.9 * expected_ip)
        self.assertLess(abs(d["qp"]), 0.05 * d["ip"])   # energy in I, not Q

        # Prompt dominates; Early/Late balanced at perfect alignment.
        self.assertGreater(p, e)
        self.assertGreater(p, l)
        self.assertLess(abs(e - l) / p, 0.05)
        # 0.5-chip taps -> ~half the prompt correlation (triangular autocorr).
        self.assertAlmostEqual(e / p, 0.5, delta=0.1)

    def test_discriminator_sign(self):
        prn = 5
        # +0.25 chip: signal code phase advanced -> signal leads the replica ->
        # the (leading) Early correlator matches better -> Early > Late.
        Ip, Qp = synth_signal(prn, code_offset_chips=+0.25)
        dp = self.assertDump(run_channel(prn, Ip, Qp))
        e_p = mag(dp["ie"], dp["qe"]); l_p = mag(dp["il"], dp["ql"])

        # -0.25 chip: signal retarded -> lags the replica -> Late > Early.
        Im, Qm = synth_signal(prn, code_offset_chips=-0.25)
        dm = self.assertDump(run_channel(prn, Im, Qm))
        e_m = mag(dm["ie"], dm["qe"]); l_m = mag(dm["il"], dm["ql"])

        self.assertGreater(e_p, l_p)   # +advance: early wins
        self.assertGreater(l_m, e_m)   # -retard: late wins

    def test_wrong_prn_no_lock(self):
        # Correlate PRN 5 signal with a PRN 10 replica -> no prompt peak.
        I, Q = synth_signal(5, code_offset_chips=0.0)
        d = self.assertDump(run_channel(10, I, Q))
        p = mag(d["ip"], d["qp"])
        aligned = CARRIER_AMP * AMP * d["n"]
        self.assertLess(p, 0.1 * aligned)

    def assertDump(self, d):
        self.assertTrue(d, "no correlator dump was produced")
        return d


if __name__ == "__main__":
    unittest.main(verbosity=2)
