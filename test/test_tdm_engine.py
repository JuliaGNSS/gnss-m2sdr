#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""TDM correlator: Migen simulation against the bit-exact model, and the
model's own correlation behaviour on a synthetic satellite."""

import copy
import unittest

from migen import run_simulation

import test.migen_compat  # noqa: F401  (simulator workaround)

from gnss_m2sdr.tdm import params as P
from gnss_m2sdr.tdm import model as M
from test.tdm_harness import TDMHarness, Scenario, REG, gps_like_signal, random_code, parse_stream


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


def run_harness(sc, ready_pattern=None, probe=None, writer=None):
    """Run ``sc``; return the DMA words and the status registers at the end.
    ``probe(k, dut)``, a generator, is called each cycle after the edge;
    ``writer(k, words)`` may return a register write, (name, value), for a
    cycle the stimulus does not write on."""
    dut = TDMHarness(n_channels=sc.n_channels, code_words=sc.code_words)
    words = []
    status = {}

    def stim():
        for k, (wv, wa, wd, sv, sw, si) in enumerate(sc.cycles):
            extra = None if (wv or writer is None) else writer(k, words)
            if extra is not None:
                wv, wa, wd = 1, REG[extra[0]], extra[1]
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
            if probe is not None:
                yield from probe(k, dut)
        status["missed"] = (yield dut.dut._missed.status)
        status["enabled"] = (yield dut.dut._enabled.status)
        status["count"] = (yield dut.dut._sample_count.status)
        status["drops"] = (yield dut.dut._record_drops.status)
        status["overflows"] = (yield dut.dut._sample_overflows.status)
        status["records"] = (yield dut.dut._records.status)

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


class TestBackToBackRounds(unittest.TestCase):
    """Samples faster than a round, so the rounds follow each other with no idle
    cycle: a slot is issued again after exactly ``num_slots + 1`` cycles, the
    least the state RAM's write-back allows (params.WRITEBACK_STAGE)."""
    def test_records_match_the_model(self):
        self.assertEqual(P.num_slots(4), P.WRITEBACK_STAGE)
        code = random_code(CODE_LEN, seed=7)
        # A round is 8 cycles; a sample every 7 builds a backlog of at most 50
        # samples per block (the FIFO holds 64), drained before each command.
        sc = build_scenario(code, spacing=7, drain=600)
        back_to_back = []

        def probe(k, dut):
            back_to_back.append((yield dut.dut.sample_fifo.re))
        words, status = run_harness(sc, probe=probe)
        self.assertEqual(status["overflows"], 0)
        self.assertEqual(parse_stream(words), sc.records)
        # Most rounds start the cycle after the previous one ended.
        pops = [k for k, re in enumerate(back_to_back) if re]
        tight = sum(1 for a, b in zip(pops, pops[1:]) if b - a == P.num_slots(4) + 1)
        self.assertGreater(tight, 500)


class TestRecordEnable(unittest.TestCase):
    """Clearing record_enable, even mid-record, leaves the DMA stream in whole
    records: the record in flight is finished and its buffer padded out."""
    def test_stream_stays_aligned(self):
        code = random_code(CODE_LEN, seed=7)
        sc = build_scenario(code)
        offs = []

        def writer(k, words):
            # Off three words into a record, back on 230 cycles later.
            if offs and k == offs[-1] + 230:
                return ("control", 0b011)
            if (len(words) % P.RECORD_WORDS == 3 and k < len(sc.cycles) - 3500
                    and (not offs or k > offs[-1] + 900)):
                offs.append(k)
                return ("control", 0b001)
        stall = lambda k: (k // 3) % 4 != 0
        words, status = run_harness(sc, ready_pattern=stall, writer=writer)
        self.assertGreater(len(offs), 5)
        self.assertEqual(len(words) % 16, 0)
        got = parse_stream(words)
        for w in (words[k:k + P.RECORD_WORDS] for k in range(0, len(words), P.RECORD_WORDS)):
            self.assertTrue(all(x == 0 for x in w) or w[0] & 0xFFFF == P.RECORD_MAGIC)
        # What arrives is the model's records with the disabled stretches cut out.
        it = iter(sc.records)
        self.assertTrue(all(any(g == w for w in it) for g in got))
        self.assertLess(len(got), len(sc.records))
        self.assertGreater(len(got), len(sc.records) // 2)
        self.assertEqual(status["records"], len(got))


class TestReset(unittest.TestCase):
    def test_reset_clears_the_counters_not_the_sample_index(self):
        code = random_code(CODE_LEN, seed=7)
        sc = build_scenario(code)
        sc.write("control", 0b111)
        sc.idle(4)
        sc.write("control", 0b011)
        sc.idle(4)
        words, status = run_harness(sc)
        self.assertGreater(len(parse_stream(words)), 0)
        self.assertEqual(status["records"], 0)
        self.assertEqual(status["enabled"], 0)
        self.assertEqual(status["missed"], 0)
        self.assertEqual(status["count"], 900)    # the index the engine has reached


def strobe_scenario(code):
    """A channel tracking through lost samples and strobe-period changes,
    starting at a sample index that is not a multiple of the period."""
    sc = Scenario(n_channels=4, code_words=256, strobe_period=0, buffer_words=16)
    sc.code(0, M.pack_code(code))
    sig = gps_like_signal(1300, fs=FS, code_chips=code, code_rate=CODE_RATE, doppler=500.0,
                          code_phase=3.0, amplitude=300, noise=60, seed=4, antenna=0)
    sc.skip_samples(1237)
    sc.samples(sig[:150])                  # no strobes: the period is 0
    sc.strobe_period(50)
    sc.arm(0, carrier_freq=M.carrier_word(500.0, FS), code_freq=M.code_word(CODE_RATE, FS),
           carrier_phase=0, code_chip=0, code_frac=0, code_length=CODE_LEN, code_base=0,
           shifts=(-2, 0, 2), antenna=0, start=sc.next_index + 20)
    sc.samples(sig[150:400])
    sc.skip_samples(7)                     # a short gap
    sc.samples(sig[400:600])
    sc.skip_samples(260)                   # a gap over several periods
    sc.samples(sig[600:800])
    sc.strobe_period(37)                   # a new period mid-run
    sc.samples(sig[800:1000])
    sc.skip_samples(5000)                  # a gap of over a hundred periods
    sc.samples(sig[1000:1300])
    sc.idle(3000)
    return sc


class TestStrobesAndLostSamples(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sc = strobe_scenario(random_code(CODE_LEN, seed=9))
        cls.words, cls.status = run_harness(cls.sc)

    def test_records_match_the_model(self):
        got = parse_stream(self.words)
        self.assertEqual(len(got), len(self.sc.records))
        for g, w in zip(got, self.sc.records):
            self.assertEqual(g, w)

    def test_strobes_sit_on_the_period(self):
        strobes = [r.sample_index for r in self.sc.records if r.kind == P.KIND_STROBE]
        on_50 = [n for n in strobes if n < 1237 + 800 + 267]
        on_37 = [n for n in strobes if n > 1237 + 800 + 267 + 40]
        self.assertGreater(len(on_50), 10)
        self.assertGreater(len(on_37), 10)
        # Off the period only right after a gap: one strobe per gap that skipped a multiple.
        self.assertLessEqual(sum(n % 50 != 0 for n in on_50), 2)
        self.assertLessEqual(sum(n % 37 != 0 for n in on_37), 1)
        # The first strobe once the period is set: no counting while it was 0.
        self.assertLess(strobes[0] - (1237 + 150), 50 + P.STROBE_SYNC_SAMPLES)

    def test_records_over_a_gap_are_flagged(self):
        dumps = [r for r in self.sc.records if r.kind == P.KIND_DUMP]
        # The first sample after each gap; each falls into exactly one record.
        gaps = (1237 + 7 + 400, 1237 + 267 + 600, 1237 + 5267 + 1000)
        ends = [0] + [r.sample_index for r in dumps]
        want = [r for r, start in zip(dumps, ends) if any(start <= g < r.sample_index for g in gaps)]
        self.assertEqual(len(want), 3)
        self.assertEqual([r for r in dumps if r.flags & P.FLAG_SAMPLE_OVERFLOW], want)


class TestSampleOverflow(unittest.TestCase):
    """Samples faster than the engine: the FIFO drops some, and the records over
    the loss carry FLAG_SAMPLE_OVERFLOW. The model is replayed on the samples
    the engine actually took."""
    def test_overflow_is_flagged(self):
        code = random_code(CODE_LEN, seed=5)
        sc = Scenario(n_channels=4, code_words=256, strobe_period=40, buffer_words=16)
        sc.code(0, M.pack_code(code))
        sig = gps_like_signal(1000, fs=FS, code_chips=code, code_rate=CODE_RATE, doppler=0.0,
                              code_phase=0.0, amplitude=300, noise=60, seed=6, antenna=1)
        sc.arm(1, carrier_freq=0, code_freq=M.code_word(CODE_RATE, FS), carrier_phase=0,
               code_chip=0, code_frac=0, code_length=CODE_LEN, code_base=0, shifts=(-1, 0, 1),
               antenna=1, start=10)
        model = copy.deepcopy(sc.model)
        sc.samples(sig[:200])
        sc.samples(sig[200:500], spacing=1)    # 8 per round: most of these are lost
        sc.samples(sig[500:])
        sc.idle(3000)
        taken = []

        def probe(k, dut):
            if (yield dut.dut.sample_fifo.re):
                taken.append((yield dut.dut.sample_fifo.dout) & ((1 << P.SAMPLE_INDEX_BITS) - 1))
        words, status = run_harness(sc, probe=probe)
        self.assertGreater(status["overflows"], 100)
        self.assertEqual(len(taken) + status["overflows"], len(sig))
        want = [r for n in taken for r in model.step(n, sig[n])]
        got = parse_stream(words)
        self.assertEqual(got, want)
        flagged = [r for r in got if r.kind == P.KIND_DUMP and r.flags & P.FLAG_SAMPLE_OVERFLOW]
        self.assertGreater(len(flagged), 0)
        self.assertTrue(all(r.sample_index > 200 for r in flagged))


class TestStrobeModel(unittest.TestCase):
    """The strobe rule in the model alone, on a long run."""
    def test_lattice(self):
        eng = M.Engine(1, 32, strobe_period=1000)
        quiet = ((0, 0), (0, 0))
        idx = [n for n in range(123456, 123456 + 20000)]
        idx = idx[:9000] + [n + 3333 for n in idx[9000:]]
        strobes = [r.sample_index for n in idx for r in eng.step(n, quiet)]
        self.assertEqual(strobes[0], 124000)
        self.assertEqual([n for n in strobes if n % 1000], [132456 + 3333 + 1])
        self.assertEqual(len(strobes), len(set(strobes)))


class TestHostHelpers(unittest.TestCase):
    def test_tap_offsets_reject_what_the_fields_cannot_hold(self):
        self.assertEqual(P.tap_offsets((-2, 0, 2)), (2, [4, 2, 0], 4))
        self.assertEqual(P.tap_offsets((31, 0)), (31, [0, 31], 31))
        for shifts in ((40, 38), (32, 31), (-32, 0), (0,) * 6, ()):
            with self.assertRaises(ValueError, msg=shifts):
                P.tap_offsets(shifts)

    def test_code_words_must_be_a_power_of_two(self):
        for n in (3000, 0, 1 << 17):
            with self.assertRaises(ValueError):
                P.capabilities_word(24, n)
        with self.assertRaises(ValueError):
            from gnss_m2sdr.tdm.engine import TDMCorrelator
            TDMCorrelator(n_channels=4, code_words=3000)


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
