#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Five-tap correlation, mixed tap layouts in one bank, and BOC side peaks.

Three things this covers that a three-tap E/P/L bank could not:

  * a channel integrating **ten** accumulators (I/Q x VE/E/P/L/VL) against an
    amplitude-bearing replica, checked bit for bit against a software model of
    the same arithmetic -- so the multiply, the saturation logic and the record
    words are all held to the same standard the three-tap path always was;
  * **one bank running both layouts at once**: GPS L1 C/A on three taps next to
    Galileo E1B on five, with the L1 C/A channel's accumulators identical to
    what a three-tap-only bank produces. That is the "mixed operation without
    tap-shape or C/N0 scaling errors" acceptance criterion, and it is why
    `num_taps` is per record rather than per build;
  * what the extra taps are *for*: BOC(1,1)'s autocorrelation has side peaks
    half a chip either side of the main one, and a three-tap DLL locks onto them
    happily -- the discriminator has a stable zero crossing there. The Very
    Early / Very Late taps are what tell the two apart.
"""

import math
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.record_format import (
    ANT_VERY_WORD, RECORD_WORDS, TAPS_EPL, TAPS_VEPL, acc_signals, pack_record,
    tap_accumulators, unpack_record,
)
from gnss_m2sdr.subcarrier import replica_shape, signal_replica_shape
from test.test_bank_csr import (
    CTL_RESTART, CTL_CARRIER_SET, LOAD_DAT, LOAD_RESET, LOAD_WE,
    csr_write, pulse_control,
)
from test.test_signal_config import (
    FRAC, PHASE_BITS, LUT_BITS, AMP_BITS, CARRIER_AMP, _sincos_tables,
    pseudo_code, pm,
)

MAX_SUB = 12
SHIFTS  = (2, 1, 0, -1, -2)       # VE, E, P, L, VL in input samples


# --- software model of a five-tap channel ------------------------------------

def software_channel_n(words, shape, samples, code_step, tap_offsets, carrier_fw,
                       num_taps=TAPS_VEPL, frac_bits=FRAC, phase_bits=PHASE_BITS,
                       lut_bits=LUT_BITS, amp_bits=AMP_BITS):
    """Bit-exact reference for TrackingChannel with a sub-chip replica.

    `words` is the code RAM content: chip in bit 0, subcarrier-table select in
    bit 1 -- the same two bits the load port writes. Everything else mirrors the
    gateware sample by sample: carrier accumulator and LUT, code accumulator,
    each tap's own signed offset resolved onto chip index +/-1, the sub-chip
    index `floor(sub * subchips)`, and the dump on the wrapping sample, which is
    included in the integration it closes.
    """
    keys = acc_signals(num_taps)
    sin_t, cos_t = _sincos_tables(lut_bits, amp_bits)
    n_ants = len(samples)
    n      = len(words)
    last   = n - 1
    frac, idx, phase = 0, 0, 0
    acc   = [dict.fromkeys(keys, 0) for _ in range(n_ants)]
    nsamp, dumps = 0, []
    for k in range(len(samples[0])):
        addr = phase >> (phase_bits - lut_bits)
        cos, sin = cos_t[addr], sin_t[addr]
        idx_next = 0 if idx >= last else idx + 1
        idx_prev = last if idx == 0 else idx - 1
        rep = []
        for off in tap_offsets:
            ph = frac + off
            if ph >= (1 << frac_bits):
                chip_i, sub = idx_next, ph - (1 << frac_bits)
            elif ph < 0:
                chip_i, sub = idx_prev, ph + (1 << frac_bits)
            else:
                chip_i, sub = idx, ph
            w   = words[chip_i]
            lut = (shape.lut_b if ((w >> 1) & 1 and shape.lut_b is not None)
                   else shape.lut_a)
            rep.append(pm(w & 1) * lut[(sub * shape.subchips) >> frac_bits])
        for a in range(n_ants):
            i, q = samples[a][k]
            i_bb = i * cos + q * sin
            q_bb = q * cos - i * sin
            for t in range(num_taps):
                acc[a][keys[2 * t]]     += rep[t] * i_bb
                acc[a][keys[2 * t + 1]] += rep[t] * q_bb
        nsamp += 1

        acc_next = frac + code_step
        carry    = acc_next >> frac_bits
        if carry and idx >= last:
            dumps.append(dict(ants=[dict(a) for a in acc], n=nsamp,
                              sample_index=k, code_chip=idx, code_phase=frac))
            acc = [dict.fromkeys(keys, 0) for _ in range(n_ants)]
            nsamp = 0
        frac = acc_next & ((1 << frac_bits) - 1)
        if carry:
            idx = idx_next
        phase = (phase + carrier_fw) & ((1 << phase_bits) - 1)
    return dumps


def run_channel_n(words, shape, samples, code_step, tap_offsets, carrier_fw,
                  num_taps=TAPS_VEPL, n_ants=1):
    """Drive a TrackingChannel with a configured sub-chip replica."""
    keys = acc_signals(num_taps)
    dut = TrackingChannel(code_frac_bits=FRAC, carrier_phase_bits=PHASE_BITS,
                          max_code_length=len(words), num_ants=n_ants,
                          code_init=words, num_taps=num_taps,
                          max_subchips=MAX_SUB)
    dumps = []

    def bench():
        for sel, lut in ((0, shape.lut_a), (1, shape.lut_b)):
            if lut is None:
                continue
            for adr, val in enumerate(lut):
                yield dut.code.lut_adr.eq(adr)
                yield dut.code.lut_sel.eq(sel)
                yield dut.code.lut_dat.eq(val)
                yield dut.code.lut_we.eq(1)
                yield
        yield dut.code.lut_we.eq(0)
        yield dut.subchips.eq(shape.subchips)
        yield dut.code_step.eq(code_step)
        yield dut.code_length.eq(len(words))
        for t, off in enumerate(tap_offsets):
            yield dut.tap_offset[t].eq(off)
        yield dut.taps_cfg.eq(num_taps)
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
                for block in dut.acc:
                    got = {}
                    for key in keys:
                        got[key] = (yield block[key])
                    ants.append(got)
                dumps.append(dict(ants=ants,
                                  n=(yield dut.integrated_samples),
                                  num_taps=(yield dut.dump_num_taps)))
        yield dut.sample_stb.eq(0)

    run_simulation(dut, bench())
    return dumps


def s32(x):
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def bpsk_boc_signal(words, shape, n_samples, code_step, carrier_fw, amp=200,
                    code_offset_chips=0.0):
    """A satellite transmitting `shape`'s replica, at a code offset."""
    n = len(words)
    out = []
    for k in range(n_samples):
        cp   = (k * code_step / (1 << FRAC) + code_offset_chips) % n
        chip = int(cp)
        frac = cp - chip
        w    = words[chip]
        lut  = (shape.lut_b if ((w >> 1) & 1 and shape.lut_b is not None)
                else shape.lut_a)
        v    = pm(w & 1) * lut[int(frac * shape.subchips)] / shape.code_amplitude
        theta = 2 * math.pi * carrier_fw * k / (1 << PHASE_BITS)
        out.append((int(round(amp * v * math.cos(theta))),
                    int(round(amp * v * math.sin(theta)))))
    return out


class TestFiveTapAccumulators(unittest.TestCase):
    """Ten accumulators, bit for bit, against the software model."""

    LENGTH, SPC = 66, 6          # 66 = 33 * 2 so the TMBOC pattern tiles

    def _words(self, shape):
        chips = pseudo_code(self.LENGTH)
        select = shape.select or [0] * self.LENGTH
        return [c | (s << 1) for c, s in zip(chips, select)]

    def _check(self, shape, n_ants=1, carrier_fw=0):
        step   = (1 << FRAC) // self.SPC
        words  = self._words(shape)
        offs   = [s * step for s in SHIFTS]
        n      = 2 * self.SPC * self.LENGTH + 8
        sig    = bpsk_boc_signal(words, shape, n, step, carrier_fw)
        sig2   = [(i // 2, q // 2) for i, q in sig]
        samples = [sig, sig2][:n_ants]
        want = software_channel_n(words, shape, samples, step, offs, carrier_fw,
                                  num_taps=TAPS_VEPL)
        got  = run_channel_n(words, shape, samples, step, offs, carrier_fw,
                             num_taps=TAPS_VEPL, n_ants=n_ants)
        self.assertGreaterEqual(len(got), 2)
        for i, (g, w) in enumerate(zip(got, want)):
            self.assertEqual(g["n"], w["n"], f"dump {i} sample count")
            self.assertEqual(g["num_taps"], TAPS_VEPL)
            for a in range(n_ants):
                self.assertEqual(g["ants"][a], w["ants"][a],
                                 f"dump {i} antenna {a}")
        return got

    def test_cboc_five_taps_one_antenna(self):
        shape = replica_shape("CBOC", name="GalileoE1B")
        got   = self._check(shape)
        # The prompt, on an aligned replica, is the largest tap.
        d = got[0]["ants"][0]
        self.assertGreater(d["ip"], max(d["ie"], d["il"], d["ive"], d["ivl"]))

    def test_tmboc_five_taps_two_antennas(self):
        shape = replica_shape("TMBOC", code_length=self.LENGTH)
        self._check(shape, n_ants=2)

    def test_five_taps_with_a_carrier(self):
        self._check(replica_shape("BOCsin", m=1), carrier_fw=1 << 24)

    def test_the_epl_taps_do_not_change_when_ve_vl_are_added(self):
        # A five-tap build must produce the same E/P/L a three-tap one does;
        # otherwise "mixed operation" would quietly rescale GPS L1 C/A.
        shape = replica_shape("LOC")
        step  = (1 << FRAC) // self.SPC
        words = self._words(shape)
        n     = 2 * self.SPC * self.LENGTH + 8
        sig   = bpsk_boc_signal(words, shape, n, step, 0)
        five  = run_channel_n(words, shape, [sig], step,
                              [s * step for s in SHIFTS], 0, num_taps=TAPS_VEPL)
        three = run_channel_n(words, shape, [sig], step,
                              [s * step for s in SHIFTS[1:4]], 0,
                              num_taps=TAPS_EPL)
        for a, b in zip(five, three):
            self.assertEqual({k: a["ants"][0][k] for k in acc_signals(TAPS_EPL)},
                             b["ants"][0])


class TestFiveTapRecords(unittest.TestCase):
    """Words 12..15, gated by the record's own num_taps."""

    LENGTH, SPC = 40, 4

    def _bank(self, num_taps, taps_cfg, num_ants=1):
        shape = replica_shape("BOCsin", m=1)
        words = pseudo_code(self.LENGTH)
        step  = (1 << FRAC) // self.SPC
        dut = GNSSTracking(n_channels=1, prns=[1], code_frac_bits=FRAC,
                           max_code_length=self.LENGTH, num_ants=num_ants,
                           num_taps=num_taps, max_subchips=MAX_SUB)
        sig = bpsk_boc_signal([w for w in words], shape, 3 * self.SPC * self.LENGTH,
                              step, 0)
        recs = []

        def bench():
            yield dut.source.ready.eq(1)
            yield dut.ch0._code_freq.storage.eq(step)
            yield dut.ch0._carrier_freq.storage.eq(0)
            yield dut._control.storage.eq(1)
            for name, shift in zip(("ve", "e", "l", "vl"), (2, 1, -1, -2)):
                reg = getattr(dut.ch0, "_tap_offset_" + name, None)
                if reg is not None:
                    yield reg.storage.eq((shift * step) & ((1 << (FRAC + 1)) - 1))
            for adr, val in enumerate(shape.lut_a):
                yield from csr_write(dut.ch0._subcarrier_load,
                                     (val & 0xFF) | (adr << 8) | (1 << 13))
            yield from csr_write(dut.ch0._code_length, self.LENGTH)
            yield from csr_write(dut.ch0._code_load, LOAD_RESET)
            for bit in words:
                yield from csr_write(dut.ch0._code_load,
                                     LOAD_WE | (LOAD_DAT if bit else 0))
            yield from csr_write(dut.ch0._replica,
                                 shape.subchips | ((1 << 4) if taps_cfg == TAPS_VEPL else 0))
            yield from pulse_control(dut.ch0, CTL_RESTART | CTL_CARRIER_SET)
            words_out = []
            for k in range(len(sig)):
                yield dut.sample_i_ants[0].eq(sig[k][0])
                yield dut.sample_q_ants[0].eq(sig[k][1])
                yield dut.sample_stb.eq(1)
                yield
                if (yield dut.source.valid):
                    words_out.append((yield dut.source.data))
            yield dut.sample_stb.eq(0)
            for _ in range(4 * RECORD_WORDS):
                yield
                if (yield dut.source.valid):
                    words_out.append((yield dut.source.data))
            for i in range(len(words_out) // RECORD_WORDS):
                raw = words_out[i * RECORD_WORDS:(i + 1) * RECORD_WORDS]
                rec = unpack_record(raw)
                rec["_words"] = raw
                recs.append(rec)

        run_simulation(dut, bench())
        return recs

    def test_a_five_tap_record_fills_the_tail_words(self):
        recs = [r for r in self._bank(TAPS_VEPL, TAPS_VEPL) if r["channel"] == 0]
        self.assertTrue(recs)
        r = recs[0]
        self.assertEqual(r["num_taps"], TAPS_VEPL)
        self.assertNotEqual(r["i_very_early"], 0)
        self.assertNotEqual(r["i_very_late"], 0)
        self.assertNotEqual(r["_words"][ANT_VERY_WORD[0]], 0)
        # Latest first, the order Tracking's correlators want.
        taps = tap_accumulators(r)
        self.assertEqual(len(taps), 5)
        self.assertEqual(taps[2], (r["i_prompt"], r["q_prompt"]))
        self.assertEqual(taps[0], (r["i_very_late"], r["q_very_late"]))
        self.assertEqual(taps[4], (r["i_very_early"], r["q_very_early"]))
        # The prompt is the middle of a symmetric layout: |VE| ~ |VL|.
        self.assertAlmostEqual(r["i_very_early"], r["i_very_late"],
                               delta=abs(r["i_prompt"]) * 0.05)

    def test_a_three_tap_channel_on_a_five_tap_build_reads_zero_there(self):
        # The trailing words must not leak the accumulators the host never
        # agreed to: `num_taps` is the contract, and 0 is what a three-tap
        # record carries.
        recs = [r for r in self._bank(TAPS_VEPL, TAPS_EPL) if r["channel"] == 0]
        self.assertTrue(recs)
        r = recs[0]
        self.assertEqual(r["num_taps"], TAPS_EPL)
        self.assertNotEqual(r["i_prompt"], 0)
        self.assertIsNone(r["i_very_early"])
        self.assertEqual(len(tap_accumulators(r)), 3)
        # Checked on the wire, not through the parser: the parser reports None
        # for a tap the record does not have whatever the words contain, so only
        # the raw words can show a five-tap build leaking its extra accumulators
        # into a three-tap channel's record.
        for n in range(len(ANT_VERY_WORD)):
            self.assertEqual(r["_words"][ANT_VERY_WORD[n] + 0], 0)
            self.assertEqual(r["_words"][ANT_VERY_WORD[n] + 1], 0)

    def test_a_three_tap_build_never_reports_five(self):
        recs = [r for r in self._bank(TAPS_EPL, TAPS_EPL) if r["channel"] == 0]
        self.assertTrue(recs)
        self.assertEqual(recs[0]["num_taps"], TAPS_EPL)
        for n in range(len(ANT_VERY_WORD)):
            self.assertEqual(recs[0]["_words"][ANT_VERY_WORD[n]], 0)


class TestMixedTapLayoutsInOneBank(unittest.TestCase):
    """GPS L1 C/A on three taps next to a BOC channel on five."""

    LENGTH, SPC = 40, 4

    def _run(self):
        step  = (1 << FRAC) // self.SPC
        loc   = replica_shape("LOC")
        cboc  = replica_shape("CBOC", name="GalileoE1B")
        code0 = pseudo_code(self.LENGTH, seed=0x1234)
        code1 = pseudo_code(self.LENGTH, seed=0x5678)
        dut = GNSSTracking(n_channels=2, prns=[1, 2], code_frac_bits=FRAC,
                           max_code_length=self.LENGTH, num_taps=TAPS_VEPL,
                           max_subchips=MAX_SUB)
        n   = 3 * self.SPC * self.LENGTH
        # Both satellites are in the air at once, on one antenna.
        s0  = bpsk_boc_signal(code0, loc,  n, step, 0, amp=200)
        s1  = bpsk_boc_signal(code1, cboc, n, step, 0, amp=200)
        recs = []

        def configure(chan, code, shape, taps, shifts):
            yield chan._code_freq.storage.eq(step)
            yield chan._carrier_freq.storage.eq(0)
            for name, shift in zip(("ve", "e", "l", "vl"), shifts):
                yield getattr(chan, "_tap_offset_" + name).storage.eq(
                    (shift * step) & ((1 << (FRAC + 1)) - 1))
            for adr, val in enumerate(shape.lut_a):
                yield from csr_write(chan._subcarrier_load,
                                     (val & 0xFF) | (adr << 8) | (1 << 13))
            yield from csr_write(chan._code_length, self.LENGTH)
            yield from csr_write(chan._code_load, LOAD_RESET)
            for bit in code:
                yield from csr_write(chan._code_load,
                                     LOAD_WE | (LOAD_DAT if bit else 0))
            yield from csr_write(chan._replica,
                                 shape.subchips | ((1 << 4) if taps == TAPS_VEPL else 0))

        def bench():
            yield dut.source.ready.eq(1)
            yield dut._control.storage.eq(1)
            yield from configure(dut.ch0, code0, loc,  TAPS_EPL,  (0, 1, -1, 0))
            yield from configure(dut.ch1, code1, cboc, TAPS_VEPL, (2, 1, -1, -2))
            yield from pulse_control(dut.ch0, CTL_RESTART | CTL_CARRIER_SET)
            yield from pulse_control(dut.ch1, CTL_RESTART | CTL_CARRIER_SET)
            out = []
            for k in range(n):
                yield dut.sample_i_ants[0].eq(s0[k][0] + s1[k][0])
                yield dut.sample_q_ants[0].eq(s0[k][1] + s1[k][1])
                yield dut.sample_stb.eq(1)
                yield
                if (yield dut.source.valid):
                    out.append((yield dut.source.data))
            yield dut.sample_stb.eq(0)
            for _ in range(8 * RECORD_WORDS):
                yield
                if (yield dut.source.valid):
                    out.append((yield dut.source.data))
            for i in range(len(out) // RECORD_WORDS):
                recs.append(unpack_record(out[i * RECORD_WORDS:(i + 1) * RECORD_WORDS]))

        run_simulation(dut, bench())
        return recs

    def test_both_layouts_stream_side_by_side(self):
        recs = self._run()
        ca   = [r for r in recs if r["channel"] == 0]
        boc  = [r for r in recs if r["channel"] == 1]
        self.assertTrue(ca and boc)
        for r in ca:
            self.assertEqual(r["num_taps"], TAPS_EPL)
            self.assertIsNone(r["i_very_early"])
        for r in boc:
            self.assertEqual(r["num_taps"], TAPS_VEPL)
        # Each channel locks on its own satellite despite the other being
        # present: the prompt power of the aligned replica dominates.
        for r in (ca[0], boc[0]):
            p = abs(r["i_prompt"])
            self.assertGreater(p, 4 * abs(r["i_early"] - r["i_late"]) + 1)

    def test_the_l1ca_channel_is_unchanged_by_its_boc_neighbour(self):
        # The regression baseline: with a BOC channel alongside, GPS L1 C/A must
        # produce exactly the accumulators the software model says -- no tap
        # reshaping and no amplitude scaling leaking across channels.
        step  = (1 << FRAC) // self.SPC
        loc   = replica_shape("LOC")
        cboc  = replica_shape("CBOC", name="GalileoE1B")
        code0 = pseudo_code(self.LENGTH, seed=0x1234)
        code1 = pseudo_code(self.LENGTH, seed=0x5678)
        n     = 3 * self.SPC * self.LENGTH
        s0    = bpsk_boc_signal(code0, loc,  n, step, 0, amp=200)
        s1    = bpsk_boc_signal(code1, cboc, n, step, 0, amp=200)
        mixed = [(a[0] + b[0], a[1] + b[1]) for a, b in zip(s0, s1)]
        want  = software_channel_n(code0, loc, [mixed], step,
                                   [step, 0, -step], 0, num_taps=TAPS_EPL)
        recs  = [r for r in self._run() if r["channel"] == 0]
        self.assertTrue(recs)
        for r, w in zip(recs, want):
            for key, sig in (("i_prompt", "ip"), ("q_prompt", "qp"),
                             ("i_early", "ie"), ("i_late", "il")):
                self.assertEqual(r[key], w["ants"][0][sig], key)


class TestBOCSidePeakDiscrimination(unittest.TestCase):
    """Why VE/VL exist: a three-tap DLL locks onto a BOC side peak.

    The replica stream comes out of the gateware; the correlation against a
    swept code offset is done here, which keeps this an analysis of the hardware
    replica rather than a second simulation per offset.
    """

    LENGTH, SPC = 128, 12       # 12 samples/chip: 1/12-chip offset resolution

    def _correlations(self, shape, shifts):
        """Normalised tap correlations against code offset, offsets in 1/12 chip."""
        step  = (1 << FRAC) // self.SPC
        words = pseudo_code(self.LENGTH)
        dut = CodeReplica(frac_bits=FRAC, max_code_length=self.LENGTH,
                          code_init=words, num_taps=len(shifts),
                          max_subchips=MAX_SUB)
        taps = [[] for _ in shifts]
        n = self.SPC * self.LENGTH

        def bench():
            for adr, val in enumerate(shape.lut_a):
                yield dut.lut_adr.eq(adr)
                yield dut.lut_dat.eq(val)
                yield dut.lut_we.eq(1)
                yield
            yield dut.lut_we.eq(0)
            yield dut.subchips.eq(shape.subchips)
            yield dut.code_length.eq(self.LENGTH)
            yield dut.code_step.eq(step)
            for t, s in enumerate(shifts):
                yield dut.tap_offset[t].eq(s * step)
            yield dut.restart.eq(1)
            yield
            yield dut.restart.eq(0)
            yield dut.stb.eq(1)
            yield
            for _ in range(n):
                for t in range(len(shifts)):
                    taps[t].append((yield dut.replica[t]))
                yield

        run_simulation(dut, bench())
        # The incoming signal is the prompt replica itself, shifted by `d`
        # samples; correlating the captured taps against it gives R(tau).
        ref = taps[shifts.index(0)]
        out = {}
        for d in range(-2 * self.SPC, 2 * self.SPC + 1):
            sig = [ref[(k + d) % n] for k in range(n)]
            out[d] = [sum(t[k] * sig[k] for k in range(n)) / n for t in taps]
        return out

    def test_boc11_has_side_peaks_that_bpsk_does_not(self):
        boc = self._correlations(replica_shape("BOCsin", m=1), (0,))
        loc = self._correlations(replica_shape("LOC"), (0,))
        half = self.SPC // 2
        peak = boc[0][0]
        self.assertGreater(peak, 0)
        # BOC(1,1): R(+/-0.5 chip) = -0.5 R(0), a genuine secondary extremum.
        self.assertAlmostEqual(boc[half][0] / peak, -0.5, delta=0.08)
        self.assertAlmostEqual(boc[-half][0] / peak, -0.5, delta=0.08)
        # Plain BPSK is a single triangle: no such lobe.
        self.assertAlmostEqual(loc[half][0] / loc[0][0], 0.5, delta=0.08)

    def test_a_three_tap_dll_has_a_false_lock_point(self):
        # E - L crosses zero at +/-0.5 chip with the *same* slope sign as at 0,
        # so a loop that settles on the main peak settles on a side peak just as
        # happily and never leaves. This is the failure the outer taps catch.
        # Tracking's DLL discriminator is the non-coherent |E| - |L| (its
        # normalisation is (2-d)/2 * (E-L)/(E+L) on the envelopes), so it zeroes
        # at every maximum of |R| -- and BOC(1,1)'s |R| peaks at 0 *and* at
        # +/-0.5 chip, where R = -0.5.
        c = self._correlations(replica_shape("BOCsin", m=1), (1, 0, -1))
        disc = {d: abs(v[0]) - abs(v[2]) for d, v in c.items()}
        half  = self.SPC // 2
        slopes = []
        for centre in (0, half, -half):
            lo, hi = disc[centre - 2], disc[centre + 2]
            self.assertLess(lo * hi, 0, f"no zero crossing at {centre}")
            slopes.append(hi - lo)
        self.assertTrue(all(s * slopes[0] > 0 for s in slopes),
                        f"side-peak crossings are not lock points: {slopes}")

    def test_the_very_taps_separate_the_main_peak_from_a_side_peak(self):
        # Bump jumping: with the prompt on the true peak, |P| beats both outer
        # taps. On a side peak one outer tap sits on the true peak instead and
        # overtakes it -- information a three-tap bank simply does not have.
        shifts = (7, 1, 0, -1, -7)          # VE/VL ~ 0.58 chip, E/L ~ 0.083
        c = self._correlations(replica_shape("BOCsin", m=1), shifts)
        half = self.SPC // 2
        on_peak = [abs(v) for v in c[0]]
        self.assertGreater(on_peak[2], on_peak[0])
        self.assertGreater(on_peak[2], on_peak[4])
        for side in (half, -half):
            v = [abs(x) for x in c[side]]
            self.assertGreater(max(v[0], v[4]), v[2],
                               f"side peak at {side} not detected")


class TestFiveTapWireFormat(unittest.TestCase):
    """pack/unpack of the five-tap layout, and what it refuses to guess."""

    def _rec(self, **kw):
        return pack_record(7, 11, 0, 5, 1, 0, 10, 20, 30, 40, 50, 60, 0x1234, **kw)

    def test_a_three_tap_record_is_byte_for_byte_what_it_was(self):
        # The regression baseline: adding the layout must not move a byte of the
        # record every GPS L1 C/A host already parses.
        with_five_args = self._rec(i_very_early=1, q_very_early=2,
                                   i_very_late=3, q_very_late=4)
        plain = self._rec()
        self.assertEqual(with_five_args, plain)
        self.assertEqual(plain[ANT_VERY_WORD[0]], 0)
        self.assertEqual(plain[ANT_VERY_WORD[1]], 0)

    def test_the_tail_words_carry_the_fifth_and_first_tap(self):
        w = self._rec(num_taps=TAPS_VEPL, i_very_early=-1, q_very_early=2,
                      i_very_late=3, q_very_late=-4,
                      ants=[dict(i_early=1, q_early=1, i_prompt=1, q_prompt=1,
                                 i_late=1, q_late=1, i_very_early=5,
                                 q_very_early=6, i_very_late=7, q_very_late=8)])
        r = unpack_record(w)
        self.assertEqual(r["num_taps"], TAPS_VEPL)
        self.assertEqual((r["i_very_early"], r["q_very_early"]), (-1, 2))
        self.assertEqual((r["i_very_late"], r["q_very_late"]), (3, -4))
        self.assertEqual(r["ants"][1]["i_very_early"], 5)
        self.assertEqual(r["ants"][1]["q_very_late"], 8)

    def test_latest_first_for_both_layouts(self):
        three = unpack_record(self._rec())
        self.assertEqual(tap_accumulators(three),
                         [(50, 60), (30, 40), (10, 20)])
        five = unpack_record(self._rec(num_taps=TAPS_VEPL, i_very_early=1,
                                       q_very_early=2, i_very_late=3,
                                       q_very_late=4))
        self.assertEqual(tap_accumulators(five),
                         [(3, 4), (50, 60), (30, 40), (10, 20), (1, 2)])

    def test_an_unknown_tap_count_is_refused_not_padded(self):
        # Padding a record out to a layout it does not have hands the loop
        # filters accumulators that never saw a replica.
        r = unpack_record(self._rec(num_taps=4))
        with self.assertRaises(ValueError):
            tap_accumulators(r)

    def test_the_record_still_divides_the_dma_buffer(self):
        self.assertEqual(len(self._rec(num_taps=TAPS_VEPL)), RECORD_WORDS)
        self.assertEqual(max(ANT_VERY_WORD) + 2, RECORD_WORDS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
