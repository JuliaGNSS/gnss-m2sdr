#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""TDM correlator: Migen simulation against the bit-exact model, and the
model's own correlation behaviour on a synthetic satellite."""

import unittest

from migen import run_simulation

import test.migen_compat  # noqa: F401  (simulator workaround)

from gnss_m2sdr.tdm import params as P
from gnss_m2sdr.tdm import model as M
from test.tdm_harness import TDMHarness, Scenario, gps_like_signal, random_code, parse_stream


FS = 4e6
CODE_LEN = 63
CODE_RATE = 1.023e6


def build_scenario(code, **kw):
    sc = Scenario(n_channels=4, code_words=256, strobe_period=100, buffer_words=16, **kw)
    sc.code(4, M.pack_code(code))
    doppler = 1250.0
    sig = gps_like_signal(900, fs=FS, code_chips=code, code_rate=CODE_RATE, doppler=doppler,
                          code_phase=10.0, amplitude=300, noise=60, seed=3, antenna=1)
    code_freq = M.code_word(CODE_RATE, FS)
    carrier = M.carrier_word(doppler, FS)
    # Channel 0: aligned with the satellite on three taps, starting at sample 5.
    # The NCO runs on the earliest tap: lead = 2 samples ahead of the prompt.
    lead_chips = 2 * CODE_RATE / FS
    start = 5
    phase = (10.0 + CODE_RATE * start / FS + lead_chips) % CODE_LEN
    sc.arm(0, carrier_freq=carrier, code_freq=code_freq, carrier_phase=0, code_chip=int(phase),
           code_frac=int((phase % 1) * 2**32), code_length=CODE_LEN, code_base=4,
           shifts=(-2, 0, 2), antenna=1, start=start)
    # Channel 2: noise antenna, wrong Doppler, five taps.
    sc.arm(2, carrier_freq=M.carrier_word(-3000.0, FS), code_freq=code_freq, carrier_phase=12345,
           code_chip=30, code_frac=0, code_length=CODE_LEN, code_base=4,
           shifts=(-4, -2, 0, 2, 4), antenna=0, start=0)
    # Channel 3: start already in the past once it lands (missed).
    sc.samples(sig[:120])
    sc.arm(3, carrier_freq=carrier, code_freq=code_freq, carrier_phase=0, code_chip=0, code_frac=0,
           code_length=CODE_LEN, code_base=4, shifts=(-2, 0, 2), antenna=1, start=50)
    # Channel 1: five taps, armed mid-run with a future start.
    phase1 = (10.0 + CODE_RATE * 300 / FS + 4 * CODE_RATE / FS) % CODE_LEN
    sc.arm(1, carrier_freq=carrier, code_freq=code_freq, carrier_phase=0, code_chip=int(phase1),
           code_frac=int((phase1 % 1) * 2**32), code_length=CODE_LEN, code_base=4,
           shifts=(-4, -2, 0, 2, 4), antenna=1, start=300)
    sc.samples(sig[120:500])
    # A new NCO word on channel 0, and channel 2 released.
    sc.word(0, M.carrier_word(doppler + 3.0, FS), code_freq + 17)
    sc.release(2)
    sc.samples(sig[500:900])
    sc.idle(3000)                          # let the flush timer pad the last buffer out
    return sc


def run_harness(sc, ready_pattern=None):
    dut = TDMHarness(n_channels=sc.n_channels, code_words=sc.code_words)
    words = []
    status = {}

    def stim():
        for k, (wv, wa, wd, sv, sw, si) in enumerate(sc.cycles):
            yield dut.wr_valid.eq(wv)
            yield dut.wr_addr.eq(wa)
            yield dut.wr_data.eq(wd)
            yield dut.sample_valid.eq(sv)
            yield dut.sample_data.eq(sw)
            yield dut.sample_index.eq(si)
            if ready_pattern is not None:
                yield dut.out_ready.eq(ready_pattern(k))
            yield
            if (yield dut.out_valid):
                words.append((yield dut.out_data))
        status["missed"] = (yield dut.dut._missed.status)
        status["enabled"] = (yield dut.dut._enabled.status)
        status["count"] = (yield dut.dut._sample_count.status)
        status["drops"] = (yield dut.dut._record_drops.status)
        status["overflows"] = (yield dut.dut._sample_overflows.status)

    run_simulation(dut, stim())
    return words, status


class TestTDMEngineAgainstModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.code = random_code(CODE_LEN, seed=7)
        cls.sc = build_scenario(cls.code)
        cls.words, cls.status = run_harness(cls.sc)

    def test_records_match_the_model(self):
        got = parse_stream(self.words)
        want = self.sc.records
        self.assertGreater(len(want), 10)
        self.assertEqual(len(got), len(want))
        for g, w in zip(got, want):
            self.assertEqual(g, w)

    def test_stream_is_whole_buffers(self):
        self.assertEqual(len(self.words) % 16, 0)

    def test_flags_and_counters(self):
        self.assertEqual(self.status["missed"], 0b1000)
        self.assertEqual(self.status["enabled"], 0b0011)
        self.assertEqual(self.status["count"], 900)
        self.assertEqual(self.status["drops"], 0)
        self.assertEqual(self.status["overflows"], 0)


class TestBackpressure(unittest.TestCase):
    def test_records_survive_a_stalling_dma(self):
        code = random_code(CODE_LEN, seed=7)
        sc = build_scenario(code)
        words, status = run_harness(sc, ready_pattern=lambda k: (k // 7) % 3 != 0)
        self.assertEqual(parse_stream(words), sc.records)
        self.assertEqual(status["drops"], 0)


class TestModelCorrelates(unittest.TestCase):
    """The model (hence the gateware) correlates: an aligned replica peaks on the
    prompt with balanced early/late taps, a misaligned one does not."""
    def test_prompt_peak(self):
        code = random_code(CODE_LEN, seed=11)
        eng = M.Engine(1, 64)
        eng.write_code(0, M.pack_code(code))
        sig = gps_like_signal(2000, fs=FS, code_chips=code, code_rate=CODE_RATE, doppler=-800.0,
                              code_phase=0.0, amplitude=200, noise=20, seed=5, antenna=0)
        lead, offsets, warmup = P.tap_offsets((-2, 0, 2))
        phase = lead * CODE_RATE / FS
        eng.arm(0, M.ChannelConfig(carrier_freq=M._signed(M.carrier_word(-800.0, FS), 32),
                                   code_freq=M.code_word(CODE_RATE, FS), code_length=CODE_LEN,
                                   lead=lead, offsets=tuple(offsets) + (0, 0), num_taps=3,
                                   warmup=warmup),
                M.Pending(start=0, code_chip=int(phase), code_frac=int((phase % 1) * 2**32)))
        recs = [r for n, s in enumerate(sig) for r in eng.step(n, s)]
        full = [r for r in recs if r.integrated > 200][1:]
        self.assertGreater(len(full), 3)
        for r in full:
            late, prompt, early = (abs(complex(*t)) for t in r.taps[:3])
            self.assertGreater(prompt, 1.5 * late)
            self.assertGreater(prompt, 1.5 * early)
            self.assertLess(abs(early - late) / prompt, 0.25)
            # One code period at 4 MS/s.
            self.assertAlmostEqual(r.integrated, CODE_LEN * FS / CODE_RATE, delta=1)


if __name__ == "__main__":
    unittest.main()
