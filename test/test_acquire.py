#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Acquisition must hand a code phase over to tracking, not just a Doppler.

A sliding-correlator run knows *where* the satellite's code is only because of
*when* the peak dump arrived: a dump fires on the code epoch, so the replica's
code phase at the peak dump is the signal's code phase, and `sample_index`
places it on the bank's one global sample axis. Drop that pair and the result
cannot start a tracking channel -- half the acquisition answer is missing.

Pinned here:
  * `acquire()` returns the peak's code phase and the global sample index it
    refers to, accurate to the per-dump slide,
  * the phase propagates to any other sample index at the acquired code rate
    (the handover arithmetic a caller has to do),
  * the detection threshold is applied by `acquire()` itself, so a caller can
    tell "acquired" from "noise" without re-deriving it,
  * the first three fields keep their historical order (metric, doppler, power).

The CSR layer is replaced by a model of one channel: a code NCO stepping at the
`code_freq` word the host wrote, a dump every simulated 1 ms on the code epoch,
and prompt power from the replica-vs-signal misalignment of a simulated
satellite at a known Doppler and code phase. `time` is faked as well, so the
sweep is deterministic and costs no wall-clock time.
"""

import math
import unittest
from unittest import mock

from software import gnss_tracking
from software.gnss_tracking import GNSSBank, GNSSChannel, acquire
from gnss_m2sdr.gps_ca import CA_CODE_LENGTH, GPS_CA_CHIP_RATE, GPS_L1_HZ

FS            = 4_000_000.0
FRAC_BITS     = 24
PREFIX        = "gnss_ch0_"
DUMP_PERIOD   = 1e-3        # simulated seconds between code epochs (~1 code period)
CLOCK_STEP    = 5e-5        # simulated seconds per time.time() call (20 polls/dump)
SAMPLE_ORIGIN = 1_234_567   # global counter at t=0; deliberately not 0, so a
                            # restart-relative index cannot pass as a global one.
TRUE_DOPPLER  = 500.0       # must land exactly on a swept Doppler bin
TRUE_EPOCH    = 3_000_111.25    # global (fractional) sample of a signal code epoch

# Sweep parameters for the bench: 0.5 chips per dump over 3.2 simulated seconds
# sweeps ~1280 chips even with the ~20% of dumps this bench's polling misses
# (as on hardware, reading is slower than dumping), so the peak is always inside
# the swept window whatever the satellite's code phase.
SWEEP = dict(doppler_range=1000.0, doppler_step=500.0, slide_chips=500.0,
             dwell=3.2, verbose=False)


def true_code_phase(sample_index, doppler=TRUE_DOPPLER):
    """Code phase (chips) of the simulated satellite at a global sample index."""
    fc = GPS_CA_CHIP_RATE * (1.0 + doppler / GPS_L1_HZ)
    return ((sample_index - TRUE_EPOCH) * fc / FS) % CA_CODE_LENGTH


def chip_error(a, b):
    """Signed difference of two code phases, wrapped to +/- half a code period."""
    return (a - b + CA_CODE_LENGTH / 2) % CA_CODE_LENGTH - CA_CODE_LENGTH / 2


class FakeClock:
    """`time` stand-in advancing CLOCK_STEP per call: no real waiting, and the
    dump schedule (and hence the whole sweep) becomes deterministic."""
    def __init__(self):
        self.now = 0.0

    def time(self):
        self.now += CLOCK_STEP
        return self.now


class FakeChannelCSR:
    """One tracking channel behind the LiteXCSR read/write API.

    The code NCO steps at the `code_freq` word the host wrote (which carries the
    acquisition slide), a dump lands every DUMP_PERIOD on the sample that
    completes a code period, and prompt power follows the correlation triangle
    of a satellite at TRUE_DOPPLER / TRUE_EPOCH.
    """
    def __init__(self, clock, signal=True):
        self.clock      = clock
        self.signal     = signal
        self.regs       = {}
        self.t_restart  = None
        self.n_restart  = 0
        self.step       = 0.0    # chips per sample
        self.dump_base  = 0      # dump_count is free-running across restarts
        self._cached_k  = None
        self._cached    = None

    # ---- CSR API -------------------------------------------------------------
    def write(self, name, value):
        prev = self.regs.get(name, 0)
        self.regs[name] = value
        if name == PREFIX + "control" and (value & 1) and not (prev & 1):
            self._restart()

    def read(self, name):
        assert name.startswith(PREFIX) or name.startswith("gnss_"), name
        field = name[len(PREFIX):]
        if field == "dump_count":
            return self.dump_base + self._dumps_since_restart()
        return self._dump()[field]

    def read_signed(self, name, bits=32):
        v = self.read(name) & ((1 << bits) - 1)
        return v - (1 << bits) if v & (1 << (bits - 1)) else v

    # ---- channel model -------------------------------------------------------
    def _restart(self):
        self.dump_base += self._dumps_since_restart()
        self.t_restart  = self.clock.now
        self.n_restart  = SAMPLE_ORIGIN + int(self.clock.now * FS)
        self.step       = self.regs.get(PREFIX + "code_freq", 0) / float(1 << FRAC_BITS)
        self._cached_k  = None

    def _dumps_since_restart(self):
        if self.t_restart is None:
            return 0
        return int((self.clock.now - self.t_restart) / DUMP_PERIOD)

    def _carrier_doppler(self):
        d = self.regs.get(PREFIX + "carrier_freq", 0) / float(1 << 32) * FS
        return d - FS if d > FS / 2 else d

    def _dump(self):
        """Fields of the k-th dump since the last restart (k >= 1)."""
        k = self._dumps_since_restart()
        if self._cached_k == k:
            return self._cached
        # The epoch fires on the sample whose advance completes k code periods,
        # i.e. the last sample whose phase is still below k*1023 chips.
        m      = math.ceil(k * CA_CODE_LENGTH / self.step) - 1
        m_prev = math.ceil((k - 1) * CA_CODE_LENGTH / self.step) - 1 if k > 1 else -1
        phase  = m * self.step - (k - 1) * CA_CODE_LENGTH   # in [1022, 1023)
        sample_index = self.n_restart + m
        # Correlation triangle x Doppler response, on a deterministic noise floor.
        delta = chip_error(phase, true_code_phase(sample_index))
        dopp  = max(0.0, 1.0 - abs(self._carrier_doppler() - TRUE_DOPPLER) / 300.0)
        amp   = 1_000_000.0 * dopp * max(0.0, 1.0 - abs(delta)) if self.signal else 0.0
        noise = 10_000 + (k * 7919) % 4001
        self._cached_k = k
        self._cached = dict(
            dump_count         = self.dump_base + k,
            ip = int(amp + noise), qp = 0,
            ie = 0, qe = 0, il = 0, ql = 0,
            integrated_samples = m - m_prev,
            sample_index       = sample_index,
            dump_code_phase    = int((phase % 1.0) * (1 << FRAC_BITS)),
        )
        return self._cached


def run_acquire(signal=True, **kwargs):
    clock = FakeClock()
    csr   = FakeChannelCSR(clock, signal=signal)
    chan  = GNSSChannel(csr, FS, index=0)
    bank  = GNSSBank(csr)
    with mock.patch.object(gnss_tracking, "time", clock):
        return acquire(chan, bank, prn=1, fs=FS, **dict(SWEEP, **kwargs))


class TestAcquireHandover(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_acquire()

    def test_finds_the_doppler_bin(self):
        self.assertTrue(self.res.detected)
        self.assertEqual(self.res.doppler_hz, TRUE_DOPPLER)

    def test_returns_the_code_phase_at_the_peak(self):
        # The peak dump is within half the per-dump slide of true alignment.
        self.assertIsNotNone(self.res.code_phase)
        self.assertGreater(self.res.sample_index, SAMPLE_ORIGIN)
        err = chip_error(self.res.code_phase, true_code_phase(self.res.sample_index))
        self.assertLess(abs(err), 1.0, f"code phase off by {err:.3f} chips")

    def test_code_phase_propagates_to_another_sample_index(self):
        # The handover arithmetic: same phase, evaluated 100 ms later.
        n = self.res.sample_index + 400_000
        err = chip_error(self.res.code_phase_at(n, FS), true_code_phase(n))
        self.assertLess(abs(err), 1.0, f"propagated phase off by {err:.3f} chips")

    def test_tuple_order_is_unchanged(self):
        metric, doppler, power = self.res[:3]
        self.assertEqual((metric, doppler, power),
                         (self.res.metric, self.res.doppler_hz, self.res.power))

    def test_noise_only_run_is_not_detected(self):
        res = run_acquire(signal=False)
        self.assertFalse(res.detected)
        self.assertLess(res.metric, 8.0)


if __name__ == "__main__":
    unittest.main()
