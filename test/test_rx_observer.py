#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""RX observer tests: one 64-bit AD9361 word is one GNSS sample in 2R2T, but
*two* consecutive samples in 1R1T ('a'/'b' slots are consecutive samples of the
single RX stream, not two antennas). Taking only bits [0:32] halves the sample
rate in 1R1T, so the bank never locks -- these tests pin both modes."""

import math
import random
import unittest

from migen import *

from test.tap_helpers import set_el_offsets_csr
from migen.sim import run_simulation

from gnss_m2sdr.gateware.rx_observer import (
    RXSampleObserver, SampleStreamRegister,
)
from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.record_format import RECORD_WORDS, unpack_record
from test.test_bank_csr import (CTL_RESTART, CTL_CARRIER_SET, LOAD_DAT,
                                LOAD_RESET, LOAD_WE, csr_write, pulse_control)
from test.test_signal_config import FRAC, pseudo_code
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
            yield from set_el_offsets_csr(bank.ch0, 1 << (FRAC - 1), FRAC)
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


class TestSampleStreamRegister(unittest.TestCase):
    """The observer -> bank pipeline stage is a pure delay.

    It exists to cut the combinational path from LitePCIe's DMA0 readiness to
    the correlator DSP (see SampleStreamRegister), which only works if it
    changes nothing else: every output must be its input exactly one cycle
    later, with the strobe still on the samples it belongs to.
    """

    def test_bundle_is_delayed_by_exactly_one_cycle(self):
        dut  = SampleStreamRegister(num_ants=2)
        # (stb, i0, q0, i1, q1, ants_valid) driven in, sparse strobes included.
        drive = [(1, 10, -20, 30, -40, 2),
                 (0,  0,   0,  0,   0, 2),
                 (1, -1,   2, -3,   4, 1),
                 (1,  5,   6,  7,   8, 2),
                 (0,  0,   0,  0,   0, 2),
                 (0,  0,   0,  0,   0, 2)]
        got = []

        def bench():
            for stb, i0, q0, i1, q1, av in drive:
                yield dut.sample_stb.eq(stb)
                yield dut.sample_i_ants[0].eq(i0)
                yield dut.sample_q_ants[0].eq(q0)
                yield dut.sample_i_ants[1].eq(i1)
                yield dut.sample_q_ants[1].eq(q1)
                yield dut.ants_valid.eq(av)
                yield
                got.append(((yield dut.out_stb),
                            (yield dut.out_i_ants[0]), (yield dut.out_q_ants[0]),
                            (yield dut.out_i_ants[1]), (yield dut.out_q_ants[1]),
                            (yield dut.out_ants_valid)))

        run_simulation(dut, bench())
        # `yield` inside a migen bench reads the post-edge value, so got[k] is
        # what the stage holds after driving item k -- i.e. item k-1.
        self.assertEqual(got[1:], drive[:-1])

    def test_antenna_zero_keeps_the_scalar_names(self):
        dut = SampleStreamRegister(num_ants=1)
        self.assertIs(dut.sample_i, dut.sample_i_ants[0])
        self.assertIs(dut.out_i,    dut.out_i_ants[0])


class TestObserverRegisterBankChain(unittest.TestCase):
    """The `soc.py` composition: observer -> SampleStreamRegister -> bank.

    `SampleStreamRegister` is tested above as a one-cycle delay, and the bank is
    tested everywhere else, but the *wiring between them* had no test -- and it
    is the only part of the sample path that PR #36 changed. When a flashed v3
    build came back with railed, code-independent accumulators, that rewiring
    was the leading suspect precisely because nothing covered it.

    The property that matters is that the stage is invisible: the bank must
    produce identical records with it and without it. The observer's samples are
    pulse-qualified -- `sample_i` is only driven while `sample_stb` is high and
    reads zero otherwise -- so delaying the bundle is only correct if the strobe
    moves with the samples it belongs to. Both AD9361 channel modes are covered
    (1R1T emits two strobes per 64-bit word, from a register, which is where a
    skew would show), and so is DMA0 back-pressure, since `rx_stb` is
    `valid & ready` and the real stream is full of gaps.
    """

    LENGTH, SPC = 64, 4

    class Chain(Module):
        def __init__(self, piped, num_ants=1):
            self.submodules.rx = rx = RXSampleObserver(data_width=64,
                                                       num_ants=num_ants)
            self.submodules.gnss = gnss = GNSSTracking(
                n_channels=1, prns=[1], code_frac_bits=FRAC,
                max_code_length=TestObserverRegisterBankChain.LENGTH,
                num_ants=num_ants, num_taps=5, max_subchips=12)
            src_i, src_q = rx.sample_i_ants, rx.sample_q_ants
            src_stb, src_valid = rx.sample_stb, rx.ants_valid
            if piped:
                self.submodules.pipe = pipe = SampleStreamRegister(num_ants=num_ants)
                self.comb += [
                    *[pipe.sample_i_ants[n].eq(rx.sample_i_ants[n])
                      for n in range(num_ants)],
                    *[pipe.sample_q_ants[n].eq(rx.sample_q_ants[n])
                      for n in range(num_ants)],
                    pipe.sample_stb.eq(rx.sample_stb),
                    pipe.ants_valid.eq(rx.ants_valid),
                ]
                src_i, src_q = pipe.out_i_ants, pipe.out_q_ants
                src_stb, src_valid = pipe.out_stb, pipe.out_ants_valid
            self.comb += [
                *[gnss.sample_i_ants[n].eq(src_i[n]) for n in range(num_ants)],
                *[gnss.sample_q_ants[n].eq(src_q[n]) for n in range(num_ants)],
                gnss.sample_stb.eq(src_stb),
                gnss.ants_valid.eq(src_valid),
            ]

    def _records(self, piped, mode_1r1t, gap, nword=700, dc=80, std=80):
        dut  = self.Chain(piped)
        rnd  = random.Random(4242)
        bits = pseudo_code(self.LENGTH)
        step = (1 << FRAC) // self.SPC
        recs = []

        def bench():
            g = dut.gnss
            yield g.source.ready.eq(1)
            yield dut.rx.mode_1r1t.eq(mode_1r1t)
            yield g.ch0._code_freq.storage.eq(step)
            yield g.ch0._carrier_freq.storage.eq(0)
            yield g._control.storage.eq(1)
            yield from csr_write(g.ch0._subcarrier_load, 1 | (0 << 8) | (1 << 13))
            yield from csr_write(g.ch0._code_length, self.LENGTH)
            yield from csr_write(g.ch0._code_load, LOAD_RESET)
            for b in bits:
                yield from csr_write(g.ch0._code_load,
                                     LOAD_WE | (LOAD_DAT if b else 0))
            yield from csr_write(g.ch0._replica, 1 | (1 << 4))
            yield from pulse_control(g.ch0, CTL_RESTART | CTL_CARRIER_SET)
            out = []
            for _ in range(nword):
                w = 0
                for slot in range(4):
                    w |= (int(rnd.gauss(dc, std)) & 0xffff) << (16 * slot)
                yield dut.rx.rx_data.eq(w)
                yield dut.rx.rx_stb.eq(1)
                yield
                if (yield g.source.valid):
                    out.append((yield g.source.data))
                for _ in range(gap):          # DMA0 not ready
                    yield dut.rx.rx_stb.eq(0)
                    yield
                    if (yield g.source.valid):
                        out.append((yield g.source.data))
            yield dut.rx.rx_stb.eq(0)
            for _ in range(4 * RECORD_WORDS):
                yield
                if (yield g.source.valid):
                    out.append((yield g.source.data))
            for n in range(len(out) // RECORD_WORDS):
                recs.append(unpack_record(out[n * RECORD_WORDS:(n + 1) * RECORD_WORDS]))

        run_simulation(dut, bench())
        return [r for r in recs if not r.get("epoch_strobe")]

    def _assert_same(self, mode_1r1t, gap):
        direct = self._records(False, mode_1r1t, gap)
        piped  = self._records(True,  mode_1r1t, gap)
        self.assertTrue(direct, "no records at all -- the stimulus is wrong")
        self.assertEqual(len(direct), len(piped))
        for a, b in zip(direct, piped):
            for field in ("i_prompt", "q_prompt", "i_early", "q_early",
                          "i_late", "q_late", "integrated_samples",
                          "code_phase_chip"):
                self.assertEqual(a[field], b[field],
                                 f"{field} differs with the pipeline stage "
                                 f"(mode_1r1t={mode_1r1t}, gap={gap})")

    def test_the_stage_is_invisible_in_2r2t(self):
        self._assert_same(mode_1r1t=0, gap=1)

    def test_the_stage_is_invisible_with_no_dma_gaps(self):
        # rx_stb high every cycle: the strobe never falls, so a stage that
        # leaked the strobe's edge rather than delaying it would still pass
        # here -- this is the easy case, kept as the control for the next two.
        self._assert_same(mode_1r1t=0, gap=0)

    def test_the_stage_is_invisible_in_1r1t(self):
        # 1R1T emits a second strobe from `pending`, a cycle after the word.
        self._assert_same(mode_1r1t=1, gap=1)

    def test_the_stage_is_invisible_under_long_back_pressure(self):
        self._assert_same(mode_1r1t=0, gap=3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
