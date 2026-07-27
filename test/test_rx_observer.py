#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""RX observer tests: one 64-bit AD9361 word is one GNSS sample in 2R2T, but
*two* consecutive samples in 1R1T ('a'/'b' slots are consecutive samples of the
single RX stream, not two antennas). Taking only bits [0:32] halves the sample
rate in 1R1T, so the bank never locks -- these tests pin both modes."""

import math
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.rx_observer import RXSampleObserver
from gnss_m2sdr.gateware.bank import GNSSTracking
from test.test_channel_lock import (
    synth_signal, FS, F_IF, CHIP_RATE, FRAC, PHASE_BITS, AMP, CARRIER_AMP,
)


def pack_word(ia, qa, ib, qb):
    """One 64-bit RX word: {ia, qa, ib, qb}, each 16-bit two's complement."""
    return ((ia & 0xffff) << 0) | ((qa & 0xffff) << 16) | \
           ((ib & 0xffff) << 32) | ((qb & 0xffff) << 48)


def run_observer(words, mode_1r1t, word_gap=0):
    """Feed words (one per word_gap+1 cycles) and collect emitted samples."""
    dut = RXSampleObserver()
    got = []

    def sample():
        if (yield dut.sample_stb):
            got.append(((yield dut.sample_i), (yield dut.sample_q)))

    def bench():
        yield dut.mode_1r1t.eq(mode_1r1t)
        yield
        for w in words:
            yield dut.rx_data.eq(w)
            yield dut.rx_stb.eq(1)
            yield
            yield from sample()
            yield dut.rx_stb.eq(0)
            for _ in range(word_gap):
                yield
                yield from sample()
        # Drain: the second sample of the last word trails its word by a cycle.
        for _ in range(4):
            yield
            yield from sample()

    run_simulation(dut, bench())
    return got


class _ObservedBank(Module):
    """RX observer + tracking bank, wired as soc.py wires them on hardware."""
    def __init__(self, **kwargs):
        self.submodules.obs  = obs  = RXSampleObserver()
        self.submodules.bank = bank = GNSSTracking(**kwargs)
        self.comb += [
            bank.sample_i.eq(obs.sample_i),
            bank.sample_q.eq(obs.sample_q),
            bank.sample_stb.eq(obs.sample_stb),
        ]


class TestRXObserver(unittest.TestCase):
    def test_2r2t_one_sample_per_word(self):
        # 2R2T: 'a' is RX1, 'b' is RX2 -- one GNSS sample per word, RX2 ignored.
        words = [pack_word(10 + k, 20 + k, -1, -2) for k in range(5)]
        got = run_observer(words, mode_1r1t=0)
        self.assertEqual(got, [(10 + k, 20 + k) for k in range(5)])

    def test_1r1t_two_samples_per_word(self):
        # 1R1T: 'a' then 'b' are two consecutive samples of the same stream.
        words = [pack_word(10 + 2 * k, 20 + 2 * k, 11 + 2 * k, 21 + 2 * k)
                 for k in range(5)]
        # word_gap=1: the tightest word cadence hardware can produce (the word
        # rate is fs/2 <= 30.72 MHz against a 125 MHz sys_clk).
        got = run_observer(words, mode_1r1t=1, word_gap=1)
        expected = []
        for k in range(5):
            expected.append((10 + 2 * k, 20 + 2 * k))
            expected.append((11 + 2 * k, 21 + 2 * k))
        self.assertEqual(got, expected)

    def test_1r1t_sparse_strobe(self):
        # Hardware strobes sparsely (word rate = fs/2 << sys_clk); the second
        # sample must still be emitted exactly once per word.
        words = [pack_word(1 + 2 * k, -(1 + 2 * k), 2 + 2 * k, -(2 + 2 * k))
                 for k in range(6)]
        got = run_observer(words, mode_1r1t=1, word_gap=7)
        expected = []
        for k in range(6):
            expected.append((1 + 2 * k, -(1 + 2 * k)))
            expected.append((2 + 2 * k, -(2 + 2 * k)))
        self.assertEqual(got, expected)

    def test_1r1t_slots_are_sign_extended(self):
        # Both slots carry 16-bit two's complement; the 'b' path must not lose
        # the sign the AD9361 core already extended.
        words = [pack_word(-2048, 2047, -1, -32768)]
        got = run_observer(words, mode_1r1t=1)
        self.assertEqual(got, [(-2048, 2047), (-1, -32768)])

    def test_back_to_back_words_keep_sample_order(self):
        # A burst out of the RX buffer (only possible after a DMA0 stall, where
        # samples were already lost upstream) drops the pending 'b' rather than
        # emitting samples out of order.
        words = [pack_word(1, 2, 3, 4), pack_word(5, 6, 7, 8)]
        got = run_observer(words, mode_1r1t=1, word_gap=0)
        self.assertEqual(got, [(1, 2), (5, 6), (7, 8)])


class TestObservedBankLock(unittest.TestCase):
    """The consequence of the bug: in 1R1T the bank sees fs/2 while the host
    programs the NCOs for fs, so nothing locks (and no dump is even produced)."""

    def _run(self, mode_1r1t, samples_per_word, word_gap=3):
        prn = 5
        I, Q = synth_signal(prn, code_offset_chips=0.0)
        dut = _ObservedBank(n_channels=1, prns=[prn], code_frac_bits=FRAC)
        bank = dut.bank
        carrier_fw = round(F_IF / FS * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
        code_step  = round(CHIP_RATE / FS * (1 << FRAC))
        dump = {}

        if samples_per_word == 2:
            words = [pack_word(I[2 * k], Q[2 * k], I[2 * k + 1], Q[2 * k + 1])
                     for k in range(len(I) // 2)]
        else:
            # 2R2T: RX2 (slots 'b') carries an unrelated signal; it is ignored.
            words = [pack_word(I[k], Q[k], AMP, -AMP) for k in range(len(I))]

        def poll():
            if (yield bank.ch0.channel.dump_stb) and not dump:
                dump.update(
                    ip=(yield bank.ch0.channel.ip),
                    qp=(yield bank.ch0.channel.qp),
                    n=(yield bank.ch0.channel.integrated_samples))

        def bench():
            yield dut.obs.mode_1r1t.eq(mode_1r1t)
            yield bank.ch0._carrier_freq.storage.eq(carrier_fw)
            yield bank.ch0._carrier_phase.storage.eq(0)
            yield bank.ch0._code_freq.storage.eq(code_step)
            yield bank.ch0._spacing.storage.eq(1 << (FRAC - 1))
            yield bank._control.storage.eq(1)          # enable bank
            yield bank.source.ready.eq(1)
            yield
            yield bank.ch0._control.storage.eq(0b11)   # restart + carrier_set
            yield
            yield bank.ch0._control.storage.eq(0)
            yield
            for w in words:
                yield dut.obs.rx_data.eq(w)
                yield dut.obs.rx_stb.eq(1)
                yield
                yield from poll()
                yield dut.obs.rx_stb.eq(0)
                for _ in range(word_gap):
                    yield
                    yield from poll()
                if dump:
                    break

        run_simulation(dut, bench())
        return dump

    def test_1r1t_bank_locks(self):
        d = self._run(mode_1r1t=1, samples_per_word=2)
        self.assertTrue(d, "no correlator dump: the bank saw fs/2, not fs")
        p = math.hypot(d["ip"], d["qp"])
        self.assertGreater(p, 0.9 * CARRIER_AMP * AMP * d["n"])

    def test_2r2t_bank_still_locks(self):
        d = self._run(mode_1r1t=0, samples_per_word=1)
        self.assertTrue(d, "no correlator dump")
        p = math.hypot(d["ip"], d["qp"])
        self.assertGreater(p, 0.9 * CARRIER_AMP * AMP * d["n"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
