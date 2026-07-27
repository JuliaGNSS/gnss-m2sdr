#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""CSR control paths of the channel bank: code loader, carrier_set, restart.

These are the paths the host actually drives at handover, and they were only
covered indirectly (or not at all):

  * **runtime code loader** (`_code_load`: dat / we / reset_addr + the
    auto-incrementing load address) -- the mechanism that makes runtime PRN
    selection work. Tested end to end: a channel whose ROM was built for PRN 1
    is loaded with PRN 20's code and then has to lock on a PRN 20 signal, which
    it cannot do unless all 1023 chips landed at the right addresses.
  * **`carrier_set` / `carrier_phase`** through the CSR edge detector, not just
    `set_phase` on the bare NCO: a level-sensitive load would keep clobbering
    the NCO every cycle the host leaves the bit set.
  * **`restart` mid-stream**, not only at t=0: the aborted partial integration
    must be discarded (accumulators *and* `integrated_samples`) while the dump
    timestamp stays on the global sample axis.

All three read the dump back through the per-channel readback CSRs, which is how
the host validates a loop over RemoteClient (and which was itself untested).

Simulation setup as in test_consecutive_dumps: 2 samples/chip (exact code period
of 2046 samples) and a constant carrier replica (cos = 127, sin = 0), so an
aligned prompt accumulator is exactly CARRIER_AMP * AMP * PERIOD -- any stale
chip or leftover accumulation breaks the equality.
"""

import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.ca_code import ca_code_reference, CA_CODE_LENGTH
from test.test_consecutive_dumps import (
    FRAC, CODE_STEP, SPACING, PERIOD, AMP, CARRIER_AMP, code_sample,
)

# _code_load field offsets (bank.py): bit0=dat, bit1=we, bit2=reset_addr.
LOAD_DAT   = 1 << 0
LOAD_WE    = 1 << 1
LOAD_RESET = 1 << 2

# _control field offsets: bit0=restart, bit1=carrier_set.
CTL_RESTART     = 1 << 0
CTL_CARRIER_SET = 1 << 1


def s32(x):
    """Interpret a 32-bit readback CSR as a signed accumulator."""
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def csr_write(csr, value):
    """One CSR bus write: `storage` stable while `re` pulses for one cycle."""
    yield csr.storage.eq(value)
    yield csr.re.eq(1)
    yield
    yield csr.re.eq(0)
    yield


def pulse_control(chan, bits):
    """Edge-trigger `_control` bits (0 -> bits -> 0), off any strobe cycle."""
    yield chan._control.storage.eq(0)
    yield
    yield chan._control.storage.eq(bits)
    yield
    yield chan._control.storage.eq(0)
    yield


def read_dump(chan):
    """The per-channel dump readback CSRs, as the host reads them."""
    return dict(
        count  = (yield chan._dump_count.status),
        ip     = s32((yield chan._ip.status)), qp = s32((yield chan._qp.status)),
        ie     = s32((yield chan._ie.status)), qe = s32((yield chan._qe.status)),
        il     = s32((yield chan._il.status)), ql = s32((yield chan._ql.status)),
        n      = (yield chan._integrated_samples.status),
        sidx   = (yield chan._sample_index.status),
        cphase = (yield chan._dump_code_phase.status),
    )


def configure(chan):
    """Code/carrier words for a code-only, replica-aligned signal."""
    yield chan._code_freq.storage.eq(CODE_STEP)
    yield chan._spacing.storage.eq(SPACING)
    yield chan._carrier_freq.storage.eq(0)     # constant replica: cos=127, sin=0
    yield chan._carrier_phase.storage.eq(0)


class TestRuntimeCodeLoad(unittest.TestCase):
    def test_loaded_prn_locks_on_a_channel_built_for_another(self):
        rom_prn, load_prn = 1, 20
        dut = GNSSTracking(n_channels=1, prns=[rom_prn], code_frac_bits=FRAC)
        chips = ca_code_reference(load_prn)
        self.assertEqual(len(chips), CA_CODE_LENGTH)
        got = {}

        def bench():
            yield from configure(dut.ch0)
            yield dut._control.storage.eq(1)       # enable bank
            yield
            # Five writes at addresses 0..4 first, so `reset_addr` has something
            # to undo: without it the code would land shifted by five chips and
            # the replica would not correlate.
            for _ in range(5):
                yield from csr_write(dut.ch0._code_load, LOAD_WE)
            yield from csr_write(dut.ch0._code_load, LOAD_RESET)
            for bit in chips:
                yield from csr_write(dut.ch0._code_load,
                                     LOAD_WE | (LOAD_DAT if bit else 0))
            # Start the integration and feed one code period of PRN 20.
            yield from pulse_control(dut.ch0, CTL_RESTART | CTL_CARRIER_SET)
            for k in range(PERIOD + 4):
                yield dut.sample_i.eq(code_sample(load_prn, k))
                yield dut.sample_q.eq(0)
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            got.update((yield from read_dump(dut.ch0)))

        run_simulation(dut, bench())
        self.assertEqual(got["count"], 1, "no dump was latched")
        self.assertEqual(got["n"], PERIOD)
        # Exact: only a replica equal to PRN 20's code, chip for chip, reaches
        # the full aligned correlation.
        self.assertEqual(got["ip"], CARRIER_AMP * AMP * PERIOD,
                         "loaded code did not reproduce the PRN 20 replica")
        self.assertEqual(got["qp"], 0)
        self.assertEqual(got["ie"], got["il"])      # E/L balanced at alignment
        # The load must not have disturbed the timestamp: CSR writes carry no
        # sample strobe, so the first gated sample is still global sample 0.
        self.assertEqual(got["sidx"], PERIOD - 1)


class TestCarrierSetCSR(unittest.TestCase):
    def test_carrier_set_loads_phase_on_the_rising_edge_only(self):
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC)
        fw   = 1 << 20
        phase0, phase1 = 1 << 30, 0x1234
        mask = (1 << 32) - 1
        got  = {}

        def bench():
            yield dut._control.storage.eq(1)          # enable bank
            yield dut.ch0._carrier_freq.storage.eq(fw)
            yield dut.ch0._carrier_phase.storage.eq(phase0)
            yield
            yield dut.ch0._control.storage.eq(CTL_CARRIER_SET)   # 0 -> 1 edge
            yield                                     # pulse cycle
            yield                                     # phase loaded here
            got["loaded"] = (yield dut.ch0.channel.carrier.phase)
            got["cos"]    = (yield dut.ch0.channel.carrier.cos)
            got["sin"]    = (yield dut.ch0.channel.carrier.sin)
            # carrier_set stays high while the CSR changes: a level-sensitive
            # load would pin the phase to phase1 instead of letting it run.
            yield dut.ch0._carrier_phase.storage.eq(phase1)
            for _ in range(6):
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            got["advanced"] = (yield dut.ch0.channel.carrier.phase)
            got["strobes"]  = (yield dut.sample_count)
            # A fresh 0 -> 1 edge does load the new value.
            yield from pulse_control(dut.ch0, CTL_CARRIER_SET)
            got["reloaded"] = (yield dut.ch0.channel.carrier.phase)

        run_simulation(dut, bench())
        self.assertEqual(got["loaded"], phase0)
        self.assertLessEqual(abs(got["cos"]), 2)      # quarter cycle
        self.assertGreater(got["sin"], 120)
        # One increment per observed strobe, from the loaded phase.
        self.assertGreaterEqual(got["strobes"], 6)
        self.assertEqual(got["advanced"], (phase0 + got["strobes"] * fw) & mask)
        self.assertEqual(got["reloaded"], phase1)


class TestRestartMidStream(unittest.TestCase):
    def test_restart_discards_the_partial_integration(self):
        # Stream a *wrong* PRN for a while (so the accumulators hold garbage and
        # the code NCO sits at an arbitrary phase), then restart and feed one
        # aligned code period. The dump must look exactly like a clean start.
        PRE = 300                       # gated samples before the restart
        prn, wrong_prn = 5, 10
        dut = GNSSTracking(n_channels=1, prns=[prn], code_frac_bits=FRAC)
        mid, got = {}, {}

        def bench():
            yield from configure(dut.ch0)
            yield dut._control.storage.eq(1)       # enable bank
            yield
            yield from pulse_control(dut.ch0, CTL_RESTART | CTL_CARRIER_SET)
            for k in range(PRE):
                yield dut.sample_i.eq(code_sample(wrong_prn, k))
                yield dut.sample_q.eq(0)
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            mid["accum"] = s32((yield dut.ch0.channel.ip))
            mid.update(count=(yield dut.ch0._dump_count.status))
            # Mid-stream restart. The CSR pulse is asynchronous to the sparse
            # sample strobe on hardware; keep it off a strobe cycle here so the
            # sample in flight is retired before the reset.
            yield from pulse_control(dut.ch0, CTL_RESTART)
            for k in range(PERIOD + 4):
                yield dut.sample_i.eq(code_sample(prn, k))
                yield dut.sample_q.eq(0)
                yield dut.sample_stb.eq(1)
                yield
            yield dut.sample_stb.eq(0)
            yield
            got.update((yield from read_dump(dut.ch0)))

        run_simulation(dut, bench())
        self.assertEqual(mid["count"], 0, "a dump escaped before the restart")
        self.assertEqual(got["count"], 1, "no dump after the restart")
        # The PRE samples of the wrong PRN are gone: nsamp counts only the new
        # integration and the prompt is the full aligned correlation.
        self.assertEqual(got["n"], PERIOD, "partial integration leaked into nsamp")
        self.assertEqual(got["ip"], CARRIER_AMP * AMP * PERIOD,
                         "accumulators were not cleared by the mid-stream restart")
        self.assertEqual(got["qp"], 0)
        # restart rebases the code phase and the integration, never the shared
        # timestamp: the dump is still placed on the global sample axis.
        self.assertEqual(got["sidx"], PRE + PERIOD - 1)
        self.assertEqual(got["sidx"] - got["n"] + 1, PRE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
