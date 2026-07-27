#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Multi-antenna (N<=2) correlation: per-antenna accumulators, shared replicas.

GNSSReceiver.jl#107 beamforms *post-correlation* on the CPU from the per-antenna
prompt covariance, so the device has to stream one E/P/L set per antenna -- any
combining in gateware destroys the spatial information the beamformer needs.
The AD9361 is 2T2R, so N<=2 coherent RX on one board (shared LO => phase
coherent). These tests pin: the record layout (a reserved second-antenna block,
a num_ants field, framing preserved), the channel (shared carrier/code replicas,
independent accumulators), the bank (both antennas in one record), and the RX
observer (2R2T 'b' slot is RX2; 1R1T has no second antenna).
"""

import math
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.rx_observer import RXSampleObserver
from gnss_m2sdr.gateware.record import CorrelatorRecorder
from gnss_m2sdr.record_format import (
    ANT_PROMPT_WORD, DMA_BUFFER_SIZE, MAGIC_OFFSET, MAGIC_WORD, N_ANTS_MAX,
    NANTS_WORD, RECORD_BYTES, RECORD_MAGIC, RECORD_WORDS, STROBE_CHANNEL,
    has_magic_at, is_epoch_strobe, pack_record, unpack_record,
)
from test.test_channel_lock import (
    synth_signal, FS, F_IF, CHIP_RATE, FRAC, PHASE_BITS, AMP, CARRIER_AMP,
)
from test.test_rx_observer import pack_word
from test.test_epoch_strobe import sample_bench
from test.test_accum_saturation import (
    ACC_MAX, CODE_STEP_MAX, N_HEADROOM, N_SATURATE, PRN as SAT_PRN, STEP,
    chip0, AMP as SAT_AMP,
)
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH


def mag(i, q):
    return math.hypot(i, q)


def halve(seq):
    return [int(round(v / 2)) for v in seq]


def run_channel_2ant(prn, ants, spacing_chips=0.5, stb_gap=0):
    """Drive a 2-antenna TrackingChannel; return the per-antenna dump.

    `ants` is [(I0, Q0), (I1, Q1)] -- simultaneous samples of the same signal
    seen by the two antennas (only the spatial phase / amplitude differs).
    """
    dut = TrackingChannel(prn=prn, code_frac_bits=FRAC,
                          carrier_phase_bits=PHASE_BITS, num_ants=2)
    carrier_fw = round(F_IF / FS * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
    code_step  = round(CHIP_RATE / FS * (1 << FRAC))
    spacing    = round(spacing_chips * (1 << FRAC))
    dump = {}

    def poll():
        if (yield dut.dump_stb) and not dump:
            dump["n"] = (yield dut.integrated_samples)
            dump["ants"] = []
            for a in dut.acc:
                dump["ants"].append(dict(
                    ie=(yield a["ie"]), qe=(yield a["qe"]),
                    ip=(yield a["ip"]), qp=(yield a["qp"]),
                    il=(yield a["il"]), ql=(yield a["ql"])))

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
        for k in range(len(ants[0][0])):
            for n, (I, Q) in enumerate(ants):
                yield dut.sample_i_ants[n].eq(I[k])
                yield dut.sample_q_ants[n].eq(Q[k])
            yield dut.sample_stb.eq(1)
            yield
            yield from poll()
            for _ in range(stb_gap):
                yield dut.sample_stb.eq(0)
                yield
                yield from poll()
            if dump:
                break

    run_simulation(dut, bench())
    return dump


def run_bank_2ant(ants, n_channels=1, prns=(5,), stb_gap=0):
    """Drive a 2-antenna GNSSTracking bank; return the raw record words."""
    dut = GNSSTracking(n_channels=n_channels, prns=list(prns),
                       code_frac_bits=FRAC, num_ants=2)
    carrier_fw = round(F_IF / FS * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
    code_step  = round(CHIP_RATE / FS * (1 << FRAC))
    words = []

    def bench():
        for chan in dut.channels:
            yield chan._carrier_freq.storage.eq(carrier_fw)
            yield chan._carrier_phase.storage.eq(0)
            yield chan._code_freq.storage.eq(code_step)
            yield chan._spacing.storage.eq(1 << (FRAC - 1))
        yield dut._control.storage.eq(1)          # enable bank
        yield dut.source.ready.eq(1)
        yield
        for chan in dut.channels:
            yield chan._control.storage.eq(0b11)  # restart + carrier_set
        yield
        for chan in dut.channels:
            yield chan._control.storage.eq(0)
        yield

        def step():
            if (yield dut.source.valid) and (yield dut.source.ready):
                words.append(((yield dut.source.data),
                              (yield dut.source.first),
                              (yield dut.source.last)))

        n = len(ants[0][0])
        for k in range(n + 8 * RECORD_WORDS):
            if k < n:
                for a, (I, Q) in enumerate(ants):
                    yield dut.sample_i_ants[a].eq(I[k])
                    yield dut.sample_q_ants[a].eq(Q[k])
                yield dut.sample_stb.eq(1)
            else:
                yield dut.sample_stb.eq(0)
            yield
            yield from step()
            for _ in range(stb_gap if k < n else 0):
                yield dut.sample_stb.eq(0)
                yield
                yield from step()

    run_simulation(dut, bench())
    return words


def split_records(words):
    """(data, first, last) beats -> list of RECORD_WORDS-long word lists."""
    recs, cur = [], []
    for data, first, last in words:
        if first:
            cur = []
        cur.append(data)
        if last and len(cur) == RECORD_WORDS:
            recs.append(cur)
    return recs


class TestRecordLayout(unittest.TestCase):
    """The wire format reserves a block per antenna and says how many are valid."""

    def test_two_antenna_blocks_and_framing(self):
        self.assertEqual(N_ANTS_MAX, 2)                 # AD9361 is 2T2R
        self.assertEqual(len(ANT_PROMPT_WORD), N_ANTS_MAX)
        self.assertEqual(RECORD_BYTES, RECORD_WORDS * 8)
        # #4's invariant must survive the layout growth: whole records per
        # DMA buffer, magic at a fixed byte offset.
        self.assertEqual(DMA_BUFFER_SIZE % RECORD_BYTES, 0)
        self.assertEqual(MAGIC_OFFSET, 44)
        # Every antenna's three I/Q words fit inside the record.
        for base in ANT_PROMPT_WORD:
            self.assertLess(base + 2, RECORD_WORDS)

    def test_pack_unpack_both_antennas(self):
        ant1 = dict(i_early=-11, q_early=22, i_prompt=-333333, q_prompt=444444,
                    i_late=-55, q_late=66)
        words = pack_record(sample_index=7, integrated_samples=4092, channel=1,
                            prn=24, seq=3, flags=0,
                            i_early=111, q_early=-222,
                            i_prompt=333333, q_prompt=-444444,
                            i_late=555, q_late=-666, code_phase=0x00ABCDEF,
                            ants=[ant1])
        self.assertEqual(len(words), RECORD_WORDS)
        rec = unpack_record(words)
        self.assertEqual(rec["num_ants"], 2)
        self.assertEqual(len(rec["ants"]), 2)
        # Antenna 0 keeps the flat field names the single-antenna host uses.
        self.assertEqual(rec["i_prompt"], 333333)
        self.assertEqual(rec["ants"][0]["i_prompt"], 333333)
        self.assertEqual(rec["ants"][0]["q_late"], -666)
        for k, v in ant1.items():
            self.assertEqual(rec["ants"][1][k], v, k)

    def test_pack_single_antenna_zeroes_second_block(self):
        words = pack_record(sample_index=1, integrated_samples=2, channel=0,
                            prn=5, seq=0, flags=0, i_early=1, q_early=2,
                            i_prompt=3, q_prompt=4, i_late=5, q_late=6,
                            code_phase=9)
        rec = unpack_record(words)
        self.assertEqual(rec["num_ants"], 1)
        self.assertEqual(len(rec["ants"]), 1)
        base = ANT_PROMPT_WORD[1]
        self.assertEqual(words[base:base + 3], [0, 0, 0])

    def test_zero_payload_record_unpacks_safely(self):
        """A record with no accumulator payload must still parse.

        #7's epoch-strobe marker is exactly that: only sample_index / seq /
        flags / channel mean anything and the rest of the record is wired to
        constants, so its antenna blocks and its num_ants word read zero. The
        clamp in unpack_record has to keep that in range instead of indexing a
        block that is not there.
        """
        words = [0] * RECORD_WORDS
        words[MAGIC_WORD] = RECORD_MAGIC << 32
        words[0] = 12345                       # sample_index
        words[1] = 0xFF << 24                  # channel = 0xFF, no payload
        rec = unpack_record(words)
        self.assertEqual(rec["sample_index"], 12345)
        self.assertEqual(rec["channel"], 0xFF)
        self.assertEqual(rec["num_ants"], 1)   # clamped up from the zero word
        self.assertEqual(len(rec["ants"]), 1)
        self.assertEqual(set(rec["ants"][0].values()), {0})

    def test_record_bytes_still_frame(self):
        import struct
        words = pack_record(sample_index=1, integrated_samples=2, channel=0,
                            prn=5, seq=0, flags=0, i_early=1, q_early=2,
                            i_prompt=3, q_prompt=4, i_late=5, q_late=6,
                            code_phase=9, ants=[dict(
                                i_early=1, q_early=1, i_prompt=1, q_prompt=1,
                                i_late=1, q_late=1)])
        raw = struct.pack("<%dQ" % RECORD_WORDS, *words)
        self.assertTrue(has_magic_at(raw, 0))
        self.assertTrue(has_magic_at(raw * 2, RECORD_BYTES))


class TestChannelPerAntenna(unittest.TestCase):
    """num_ants x 6 accumulators behind one carrier NCO + one code replica."""

    def _run(self, stb_gap=0):
        prn = 5
        # Antenna 0: the reference. Antenna 1: same signal, +90 deg spatial
        # phase and half the amplitude -- what a second element of a coherent
        # array sees. Pre-combining in gateware would lose exactly this.
        I0, Q0 = synth_signal(prn, code_offset_chips=0.0, carrier_phase0=0.0)
        I1, Q1 = synth_signal(prn, code_offset_chips=0.0,
                              carrier_phase0=math.pi / 2)
        d = run_channel_2ant(prn, [(I0, Q0), (halve(I1), halve(Q1))],
                             stb_gap=stb_gap)
        self.assertTrue(d, "no correlator dump was produced")
        return d

    def test_per_antenna_accumulators(self):
        d = self._run()
        a0, a1 = d["ants"]
        aligned = CARRIER_AMP * AMP * d["n"]

        # Antenna 0 (phi=0): prompt energy in I.
        self.assertGreater(a0["ip"], 0.9 * aligned)
        self.assertLess(abs(a0["qp"]), 0.05 * a0["ip"])
        # Antenna 1 (phi=+90 deg): same code lock, energy rotated into Q, and
        # half the magnitude. Both are only visible if the accumulators are
        # per-antenna rather than summed.
        self.assertGreater(a1["qp"], 0.9 * 0.5 * aligned)
        self.assertLess(abs(a1["ip"]), 0.05 * a1["qp"])
        self.assertAlmostEqual(mag(a1["ip"], a1["qp"]) / mag(a0["ip"], a0["qp"]),
                               0.5, delta=0.05)
        # E/L still balanced on both antennas (shared replicas, same spacing).
        for a in (a0, a1):
            p = mag(a["ip"], a["qp"])
            e = mag(a["ie"], a["qe"]); l = mag(a["il"], a["ql"])
            self.assertLess(abs(e - l) / p, 0.05)

    def test_per_antenna_accumulators_sparse_strobe(self):
        # Hardware strobes once every ~31 sys cycles; both antennas are
        # presented on the same strobe and must dump on the same epoch.
        d = self._run(stb_gap=7)
        a0, a1 = d["ants"]
        aligned = CARRIER_AMP * AMP * d["n"]
        self.assertGreater(a0["ip"], 0.9 * aligned)
        self.assertGreater(a1["qp"], 0.9 * 0.5 * aligned)

    def test_replicas_are_shared(self):
        # Identical inputs on both antennas must give bit-identical
        # accumulators: one carrier NCO and one code replica feed both, so
        # there is no per-antenna phase state to drift.
        prn = 5
        I, Q = synth_signal(prn, code_offset_chips=0.0)
        d = run_channel_2ant(prn, [(I, Q), (I, Q)])
        self.assertTrue(d, "no correlator dump was produced")
        self.assertEqual(d["ants"][0], d["ants"][1])


class TestPerAntennaSaturation(unittest.TestCase):
    """The #12 clamp must guard every antenna, not just antenna 0.

    An unguarded antenna 1 would wrap, and a wrapped accumulator is
    indistinguishable from a plausible correlator value on the host -- the exact
    failure #12 exists to prevent, just moved one antenna over. Drive trick as in
    test_accum_saturation: freeze the code NCO so the prompt chip is constant,
    feed full-scale sign-matched samples, then run the NCO out to the epoch.
    """

    def _drive(self, n_drive, ant):
        dut = TrackingChannel(prn=SAT_PRN, code_frac_bits=FRAC,
                              carrier_phase_bits=PHASE_BITS, num_ants=2)
        out = {}

        def bench():
            yield dut.carrier_fw.eq(0)              # cos = +127, sin = 0
            yield dut.carrier_phase_in.eq(0)
            yield dut.spacing.eq(1 << (FRAC - 1))
            yield dut.code_step.eq(0)               # freeze the code phase
            yield dut.carrier_set.eq(1)
            yield dut.restart.eq(1)
            yield
            yield dut.carrier_set.eq(0)
            yield dut.restart.eq(0)
            # Only `ant` is driven; the other antenna stays at zero.
            yield dut.sample_i_ants[ant].eq(SAT_AMP * chip0(SAT_PRN))
            yield dut.sample_stb.eq(1)
            for _ in range(n_drive):
                yield
            out["saturated"] = (yield dut.saturated)
            yield dut.sample_i_ants[ant].eq(0)
            yield dut.code_step.eq(CODE_STEP_MAX)
            for _ in range(CA_CODE_LENGTH + 64):
                yield
                if (yield dut.dump_stb):
                    out["dump_saturated"] = (yield dut.dump_saturated)
                    out["ip"] = []
                    for a in dut.acc:
                        out["ip"].append((yield a["ip"]))
                    break
            yield dut.sample_stb.eq(0)
            yield

        run_simulation(dut, bench())
        self.assertIn("ip", out, "no dump produced")
        return out

    def test_antenna_1_overflow_clamps_and_is_flagged(self):
        out = self._drive(N_SATURATE, ant=1)
        self.assertGreater(N_SATURATE * STEP, ACC_MAX, "test drive does not overflow")
        self.assertEqual(out["ip"][1], ACC_MAX, "antenna 1 wrapped instead of clamping")
        self.assertEqual(out["ip"][0], 0)          # undriven antenna stays put
        # Saturation is a per-channel verdict: any antenna clamping spoils the
        # dump the host would beamform.
        self.assertEqual(out["saturated"], 1)
        self.assertEqual(out["dump_saturated"], 1)

    def test_antenna_1_within_range_does_not_flag(self):
        out = self._drive(N_HEADROOM, ant=1)
        self.assertEqual(out["ip"][1], N_HEADROOM * STEP)
        self.assertEqual(out["saturated"], 0)
        self.assertEqual(out["dump_saturated"], 0)


class TestBankMultiAntenna(unittest.TestCase):
    def test_record_carries_both_antennas(self):
        prn = 5
        I0, Q0 = synth_signal(prn, code_offset_chips=0.0, carrier_phase0=0.0)
        I1, Q1 = synth_signal(prn, code_offset_chips=0.0,
                              carrier_phase0=math.pi / 2)
        words = run_bank_2ant([(I0, Q0), (halve(I1), halve(Q1))], prns=(prn,))
        recs = [unpack_record(w) for w in split_records(words)]
        self.assertTrue(recs, "no records produced")
        r = recs[0]
        self.assertEqual(r["num_ants"], 2)
        self.assertEqual(r["prn"], prn)
        aligned = CARRIER_AMP * AMP * r["integrated_samples"]
        a0, a1 = r["ants"]
        self.assertGreater(a0["i_prompt"], 0.9 * aligned)
        self.assertGreater(a1["q_prompt"], 0.9 * 0.5 * aligned)
        self.assertLess(abs(a1["i_prompt"]), 0.05 * a1["q_prompt"])
        # Both blocks come from one integration against one replica set: same
        # timestamp, same code_phase, same E/L balance (#107 keeps NCOUpdate
        # one-per-channel, so the record has one of each, not one per antenna).
        for a in (a0, a1):
            p = mag(a["i_prompt"], a["q_prompt"])
            e = mag(a["i_early"], a["q_early"])
            l = mag(a["i_late"], a["q_late"])
            self.assertLess(abs(e - l) / p, 0.05)
        self.assertEqual(r["integrated_samples"], r["sample_index"] + 1,
                         "the integration should cover the whole run so far")

    def test_single_antenna_bank_reports_one_and_zeroes_the_block(self):
        prn = 5
        I, Q = synth_signal(prn, code_offset_chips=0.0)
        dut = GNSSTracking(n_channels=1, prns=[prn], code_frac_bits=FRAC)
        self.assertEqual(len(dut.channels[0].channel.acc), 1)
        words = []

        def bench():
            chan = dut.channels[0]
            carrier_fw = round(F_IF / FS * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
            yield chan._carrier_freq.storage.eq(carrier_fw)
            yield chan._code_freq.storage.eq(round(CHIP_RATE / FS * (1 << FRAC)))
            yield chan._spacing.storage.eq(1 << (FRAC - 1))
            yield dut._control.storage.eq(1)
            yield dut.source.ready.eq(1)
            yield
            yield chan._control.storage.eq(0b11)
            yield
            yield chan._control.storage.eq(0)
            yield
            for k in range(len(I) + 4 * RECORD_WORDS):
                if k < len(I):
                    yield dut.sample_i.eq(I[k])
                    yield dut.sample_q.eq(Q[k])
                    yield dut.sample_stb.eq(1)
                else:
                    yield dut.sample_stb.eq(0)
                yield
                if (yield dut.source.valid) and (yield dut.source.ready):
                    words.append(((yield dut.source.data),
                                  (yield dut.source.first),
                                  (yield dut.source.last)))

        run_simulation(dut, bench())
        recs = split_records(words)
        self.assertTrue(recs, "no records produced")
        base = ANT_PROMPT_WORD[1]
        self.assertEqual(recs[0][base:base + 3], [0, 0, 0],
                         "unused antenna block is not zeroed")
        self.assertEqual(unpack_record(recs[0])["num_ants"], 1)


class TestEpochStrobeWithTwoAntennas(unittest.TestCase):
    """#7's epoch strobe and the per-antenna blocks are orthogonal.

    The strobe is an extra round-robin *slot*; the antennas are a record-*layout*
    change. A marker carries no correlator payload, so on a 2-antenna build both
    antenna blocks and the num_ants word must read zero in it -- and the host's
    is_epoch_strobe() must still fire at the 16-word stride.
    """

    def test_strobe_record_has_no_antenna_payload(self):
        dut = CorrelatorRecorder(n_channels=1, num_ants=2)
        period = 8
        words = []

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.epoch_period.eq(period)
            yield from sample_bench(dut, words, n_samples=3 * period, origin=4096)

        run_simulation(dut, bench())
        raw = [words[i:i + RECORD_WORDS]
               for i in range(0, len(words) - RECORD_WORDS + 1, RECORD_WORDS)]
        self.assertTrue(raw, "no strobe records emitted")
        for w in raw:
            rec = unpack_record(w)
            self.assertTrue(is_epoch_strobe(rec), "marker lost at the 16-word stride")
            self.assertEqual(rec["channel"], STROBE_CHANNEL)
            # Both antenna blocks and the num_ants word are zero on the wire.
            for base in ANT_PROMPT_WORD:
                self.assertEqual(w[base:base + 3], [0, 0, 0],
                                 "strobe record carries an antenna payload")
            self.assertEqual(w[NANTS_WORD], 0)
            # The host still gets something safe to unpack.
            self.assertEqual(rec["num_ants"], 1)
            self.assertEqual(set(rec["ants"][0].values()), {0})

    def test_dumps_from_both_antennas_survive_alongside_strobes(self):
        # A marker must not preempt or corrupt a two-antenna dump.
        dut = CorrelatorRecorder(n_channels=1, num_ants=2)
        words = []
        ant1 = dict(ie=-11, qe=22, ip=-33, qp=44, il=-55, ql=66)

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.epoch_period.eq(6)
            p = dut.ports[0]
            yield p.sample_index.eq(4091)
            yield p.integrated_samples.eq(4092)
            yield p.code_phase.eq(0x123456)
            yield p.prn.eq(7)
            for k, v in (("ie", 11), ("qe", -12), ("ip", 13),
                         ("qp", -14), ("il", 15), ("ql", -16)):
                yield p.acc[0][k].eq(v)
            for k, v in ant1.items():
                yield p.acc[1][k].eq(v)

            def hook(k, step):
                if k == 9:
                    yield p.stb.eq(1)
                    yield from step()
                    yield p.stb.eq(0)

            yield from sample_bench(dut, words, n_samples=24, hook=hook)

        run_simulation(dut, bench())
        recs = [unpack_record(words[i:i + RECORD_WORDS])
                for i in range(0, len(words) - RECORD_WORDS + 1, RECORD_WORDS)]
        strobes = [r for r in recs if is_epoch_strobe(r)]
        dumps   = [r for r in recs if not is_epoch_strobe(r)]
        self.assertTrue(strobes, "no strobe records emitted")
        self.assertEqual(len(dumps), 1, "the dump was lost or duplicated")
        d = dumps[0]
        self.assertEqual(d["num_ants"], 2)
        self.assertEqual(d["ants"][0]["i_prompt"], 13)
        self.assertEqual(d["ants"][1]["i_prompt"], -33)
        self.assertEqual(d["ants"][1]["q_late"], 66)


class TestObserverSecondAntenna(unittest.TestCase):
    """In 2R2T the word's 'b' slot is RX2 -- antenna 1, not the next sample."""

    def _observe(self, words, mode_1r1t, word_gap=1):
        """Feed RX words; return (per-strobe [(ant0), (ant1)], ants_valid)."""
        dut = RXSampleObserver(num_ants=2)
        got, valid = [], []

        def sample():
            if (yield dut.sample_stb):
                got.append((((yield dut.sample_i_ants[0]),
                             (yield dut.sample_q_ants[0])),
                            ((yield dut.sample_i_ants[1]),
                             (yield dut.sample_q_ants[1]))))

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
            valid.append((yield dut.ants_valid))

        run_simulation(dut, bench())
        return got, valid[0]

    def test_2r2t_second_slot_is_antenna_1(self):
        words = [pack_word(10 + k, 20 + k, -30 - k, -40 - k) for k in range(4)]
        got, valid = self._observe(words, mode_1r1t=0)
        self.assertEqual(valid, 2)
        self.assertEqual(got, [((10 + k, 20 + k), (-30 - k, -40 - k))
                               for k in range(4)])

    def test_1r1t_has_no_second_antenna(self):
        # 1R1T: 'a'/'b' are consecutive samples of the *one* RX, so there is no
        # antenna 1. It mirrors antenna 0 and ants_valid drops to 1 -- a
        # beamformer fed two identical antennas has a singular covariance.
        words = [pack_word(1 + 2 * k, -(1 + 2 * k), 2 + 2 * k, -(2 + 2 * k))
                 for k in range(3)]
        got, valid = self._observe(words, mode_1r1t=1, word_gap=3)
        self.assertEqual(valid, 1)
        self.assertEqual([a for a, _ in got],
                         [(1, -1), (2, -2), (3, -3), (4, -4), (5, -5), (6, -6)])
        for a0, a1 in got:
            self.assertEqual(a0, a1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
