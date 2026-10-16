#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""The TDM correlator engine, its host interface and its record stream.

``TDMEngine`` is the datapath: one round per input sample, one cycle per slot,
an eight-stage pipeline from the slot's state read to its write-back. The
stage boundaries are the ``# Sk`` comments below and match the order of the
operations in ``model.py``.

``TDMCorrelator`` wraps it with what the SoC needs: the sample FIFO fed from
the RX stream, the CSRs the host drives it through, the record FIFO, and the
serialiser that cuts the records into DMA buffers and pads a buffer once its
oldest record has waited ``flush_cycles``.

Arithmetic note. Every constant below is written as an unsigned bit pattern
and every signed operand is a signed ``Signal``: LiteX's Verilog backend has
emitted negative literals as unsigned before (docs/gateware_builds.md 5.6e),
so nothing here depends on how a negative constant is rendered.
"""

from functools import reduce
from operator import or_

from migen import *
from migen.genlib.fifo import SyncFIFOBuffered

from litex.gen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField

from gnss_m2sdr.tdm import params as P


def _record_layout():
    return [
        ("kind",         2),
        ("channel",      8),
        ("num_taps",     3),
        ("antenna",      1),
        ("flags",        6),
        ("assign_id",    P.ASSIGN_BITS),
        ("sample_index", P.SAMPLE_INDEX_BITS),
        ("integrated",   P.INTEG_BITS),
        ("code_chip",    P.CHIP_BITS),
        ("code_frac20",  20),
    ] + [(f"tap{k}_{part}", P.ACC_BITS) for k in range(P.MAX_TAPS) for part in ("i", "q")]


class _Fields:
    """A flat Signal split into named fields (record payloads, memory words).

    Reset-less: the pipeline registers built from these carry data that is only
    looked at under a valid bit, and leaving them out of the reset tree keeps
    the reset's fanout off the datapath's timing (it was the engine's worst
    path, at 0 logic levels)."""
    def __init__(self, layout, name):
        self.layout = layout
        self.width = sum(w for _, w in layout)
        self.raw = Signal(self.width, name=name, reset_less=True)
        offset = 0
        for fname, w in layout:
            setattr(self, fname, self.raw[offset:offset + w])
            offset += w


class TDMEngine(LiteXModule):
    """Round-robin correlator datapath.

    Inputs: ``issue`` with ``slot``, ``sample_index`` and the four 12-bit samples
    (I/Q of two antennas), one slot per cycle. Host-side configuration arrives on
    the memory write ports exposed below (``cfg_*_wr``, ``code_wr``) and on the
    flag requests; records leave on ``rec_valid`` / ``rec``.
    """
    def __init__(self, n_channels, code_words):
        assert 1 <= n_channels <= 32
        self.n_channels = n_channels
        self.n_slots    = P.num_slots(n_channels)

        # Issue port.
        self.issue        = Signal()
        self.slot         = Signal(8)
        self.sample_index = Signal(P.SAMPLE_INDEX_BITS)
        self.samples      = [Signal((P.SAMPLE_BITS, True)) for _ in range(4)]   # i0, q0, i1, q1
        self.strobe_due   = Signal()

        # Records out (one cycle pulse, at most one per cycle).
        self.rec_valid = Signal()
        self.rec = _Fields(_record_layout(), "rec")

        # Flag interface: the host side owns the flags; the engine asks.
        self.flag_enabled = Array(Signal(name=f"en{i}") for i in range(n_channels))
        self.flag_pending = Array(Signal(name=f"pend{i}") for i in range(n_channels))
        self.flag_gen     = Array(Signal(2, name=f"gen{i}") for i in range(n_channels))
        self.req_start    = Signal()
        self.req_missed   = Signal()
        self.req_slot     = Signal(8)
        self.req_gen      = Signal(2)

        # Host-written per-channel memories.
        freq_layout = [("carrier_freq", P.FREQ_BITS), ("code_freq", P.FREQ_BITS)]
        static_layout = [
            ("code_length", P.CHIP_BITS),
            ("code_base",   16),
            ("lead",        P.HISTORY_BITS),
        ] + [(f"d{k}", P.HISTORY_BITS) for k in range(P.MAX_TAPS)] + [
            ("num_taps",    3),
            ("warmup",      P.HISTORY_BITS),
            ("antenna",     1),
            ("assign_id",   P.ASSIGN_BITS),
        ]
        pending_layout = [
            ("start",         P.SAMPLE_INDEX_BITS),
            ("carrier_phase", P.PHASE_BITS),
            ("code_chip",     P.CHIP_BITS),
            ("code_frac",     P.FRAC_BITS),
        ]
        self.freq_layout, self.static_layout, self.pending_layout = freq_layout, static_layout, pending_layout
        freq_w    = sum(w for _, w in freq_layout)
        static_w  = sum(w for _, w in static_layout)
        pending_w = sum(w for _, w in pending_layout)

        mem_freq    = Memory(freq_w,    n_channels, name="tdm_freq")
        mem_static  = Memory(static_w,  n_channels, name="tdm_static")
        mem_pending = Memory(pending_w, n_channels, name="tdm_pending")
        mem_code    = Memory(P.CODE_WORD_BITS, code_words, name="tdm_code")
        self.cfg_freq_wr    = mem_freq.get_port(write_capable=True)
        self.cfg_static_wr  = mem_static.get_port(write_capable=True)
        self.cfg_pending_wr = mem_pending.get_port(write_capable=True)
        self.code_wr        = mem_code.get_port(write_capable=True)
        freq_rd    = mem_freq.get_port(async_read=True)
        static_rd  = mem_static.get_port(async_read=True)
        pending_rd = mem_pending.get_port(async_read=True)
        code_rd    = mem_code.get_port()                       # synchronous: block RAM
        self.specials += [mem_freq, mem_static, mem_pending, mem_code,
                          self.cfg_freq_wr, self.cfg_static_wr, self.cfg_pending_wr, self.code_wr,
                          freq_rd, static_rd, pending_rd, code_rd]

        # Channel state RAM.
        state_layout = [
            ("carrier_phase", P.PHASE_BITS),
            ("code_frac",     P.FRAC_BITS),
            ("code_chip",     P.CHIP_BITS),
            ("hist_rep",      P.HISTORY),
            ("hist_wrap",     P.HISTORY),
            ("integ",         P.INTEG_BITS),
            ("warm",          P.HISTORY_BITS),
            ("sat",           1),
        ] + [(f"acc{k}_{part}", P.ACC_BITS) for k in range(P.MAX_TAPS) for part in ("i", "q")]
        state_w = sum(w for _, w in state_layout)
        mem_state = Memory(state_w, n_channels, name="tdm_state")
        state_rd = mem_state.get_port()                        # synchronous
        state_wr = mem_state.get_port(write_capable=True)
        self.specials += mem_state, state_rd, state_wr

        # # #

        cos_t, sin_t = P.carrier_table()
        is_channel = lambda slot: slot < n_channels

        # S0: issue. ------------------------------------------------------------------------------
        self.comb += [
            state_rd.adr.eq(self.slot),
            freq_rd.adr.eq(self.slot),
            static_rd.adr.eq(self.slot),
            pending_rd.adr.eq(self.slot),
        ]
        s1_valid, s1_slot, s1_n, s1_strobe = Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s1_samples = [Signal((P.SAMPLE_BITS, True)) for _ in range(4)]
        s1_freq    = _Fields(freq_layout, "s1_freq")
        s1_static  = _Fields(static_layout, "s1_static")
        s1_pending = _Fields(pending_layout, "s1_pending")
        s1_en, s1_pend, s1_gen = Signal(), Signal(), Signal(2)
        slot_ok = Signal()
        self.comb += slot_ok.eq(is_channel(self.slot))
        self.sync += [
            s1_valid.eq(self.issue),
            s1_slot.eq(self.slot),
            s1_n.eq(self.sample_index),
            s1_strobe.eq(self.strobe_due & (self.slot == self.n_slots - 1)),
            *[a.eq(b) for a, b in zip(s1_samples, self.samples)],
            s1_freq.raw.eq(freq_rd.dat_r),
            s1_static.raw.eq(static_rd.dat_r),
            s1_pending.raw.eq(pending_rd.dat_r),
            s1_en.eq(slot_ok & self.flag_enabled[self.slot]),
            s1_pend.eq(slot_ok & self.flag_pending[self.slot]),
            s1_gen.eq(self.flag_gen[self.slot]),
        ]
        s1_state = _Fields(state_layout, "s1_state")
        self.comb += s1_state.raw.eq(state_rd.dat_r)

        # S1: start-sample comparison, antenna selection. -----------------------------------------
        s2_valid, s2_slot, s2_n, s2_strobe = Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s2_freq    = _Fields(freq_layout, "s2_freq")
        s2_static  = _Fields(static_layout, "s2_static")
        s2_pending = _Fields(pending_layout, "s2_pending")
        s2_state   = _Fields(state_layout, "s2_state")
        s2_en, s2_pend, s2_gen = Signal(), Signal(), Signal(2)
        s2_diff = Signal(P.SAMPLE_INDEX_BITS + 1)
        s2_i, s2_q = Signal((P.SAMPLE_BITS, True)), Signal((P.SAMPLE_BITS, True))
        diff = Signal(P.SAMPLE_INDEX_BITS + 1)
        self.comb += diff.eq(Cat(s1_n, C(0, 1)) - Cat(s1_pending.start, C(0, 1)))
        self.sync += [
            s2_valid.eq(s1_valid), s2_slot.eq(s1_slot), s2_n.eq(s1_n), s2_strobe.eq(s1_strobe & s1_valid),
            s2_freq.raw.eq(s1_freq.raw), s2_static.raw.eq(s1_static.raw),
            s2_pending.raw.eq(s1_pending.raw), s2_state.raw.eq(s1_state.raw),
            s2_en.eq(s1_en), s2_pend.eq(s1_pend), s2_gen.eq(s1_gen),
            s2_diff.eq(diff),
            If(s1_static.antenna,
                s2_i.eq(s1_samples[2]), s2_q.eq(s1_samples[3]),
            ).Else(
                s2_i.eq(s1_samples[0]), s2_q.eq(s1_samples[1]),
            ),
        ]

        # S2: start / missed decision, effective state, NCO sums. ---------------------------------
        start  = Signal()
        missed = Signal()
        diff_zero = Signal()
        self.comb += [
            diff_zero.eq(s2_diff == 0),
            start.eq(s2_valid & s2_pend & diff_zero),
            # n - start > 0: no borrow out of the top bit and not zero.
            missed.eq(s2_valid & s2_pend & ~s2_diff[P.SAMPLE_INDEX_BITS] & ~diff_zero),
            self.req_start.eq(start),
            self.req_missed.eq(missed),
            self.req_slot.eq(s2_slot),
            self.req_gen.eq(s2_gen),
        ]
        e = _Fields(state_layout, "s2_eff")
        self.comb += [
            e.raw.eq(s2_state.raw),
            If(start,
                e.raw.eq(0),
                e.carrier_phase.eq(s2_pending.carrier_phase),
                e.code_chip.eq(s2_pending.code_chip),
                e.code_frac.eq(s2_pending.code_frac),
                e.warm.eq(s2_static.warmup),
            ),
        ]
        active = Signal()
        self.comb += active.eq(s2_valid & (s2_en | start))

        s3_valid, s3_active, s3_slot, s3_n, s3_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s3_static = _Fields(static_layout, "s3_static")
        s3_state  = _Fields(state_layout, "s3_state")     # effective state, NCOs not yet advanced
        s3_lut    = Signal(P.CARRIER_LUT_BITS)
        s3_cp_next   = Signal(P.PHASE_BITS)
        s3_frac_next = Signal(P.FRAC_BITS)
        s3_carry     = Signal()
        s3_code_adr  = Signal(16)
        s3_i, s3_q = Signal((P.SAMPLE_BITS, True)), Signal((P.SAMPLE_BITS, True))
        frac_sum = Signal(P.FRAC_BITS + 1)
        self.comb += frac_sum.eq(e.code_frac + s2_freq.code_freq)
        self.sync += [
            s3_valid.eq(s2_valid), s3_active.eq(active), s3_slot.eq(s2_slot), s3_n.eq(s2_n), s3_strobe.eq(s2_strobe),
            s3_static.raw.eq(s2_static.raw),
            s3_state.raw.eq(e.raw),
            s3_lut.eq(e.carrier_phase[P.PHASE_BITS - P.CARRIER_LUT_BITS:]),
            # The frequency word is signed, but modulo 2^32 the unsigned sum is the same bits.
            s3_cp_next.eq(e.carrier_phase + s2_freq.carrier_freq),
            s3_frac_next.eq(frac_sum[:P.FRAC_BITS]),
            s3_carry.eq(frac_sum[P.FRAC_BITS]),
            s3_code_adr.eq(s2_static.code_base + e.code_chip[5:]),
            s3_i.eq(s2_i), s3_q.eq(s2_q),
        ]

        # S3: code-memory address out, carrier table, chip increment. -----------------------------
        self.comb += code_rd.adr.eq(s3_code_adr)
        lut_cos = Signal(8)
        lut_sin = Signal(8)
        self.comb += Case(s3_lut, {
            k: [lut_cos.eq(cos_t[k] & 0xFF), lut_sin.eq(sin_t[k] & 0xFF)] for k in range(len(cos_t))
        })
        chip_inc = Signal(P.CHIP_BITS + 1)
        self.comb += chip_inc.eq(s3_state.code_chip + s3_carry)
        s4_valid, s4_active, s4_slot, s4_n, s4_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s4_static = _Fields(static_layout, "s4_static")
        s4_state  = _Fields(state_layout, "s4_state")
        s4_cos, s4_sin = Signal((8, True)), Signal((8, True))
        s4_i, s4_q = Signal((P.SAMPLE_BITS, True)), Signal((P.SAMPLE_BITS, True))
        s4_cp_next, s4_frac_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS)
        s4_chip_next, s4_wrap = Signal(P.CHIP_BITS), Signal()
        s4_bitsel = Signal(5)
        wrap = Signal()
        self.comb += wrap.eq(chip_inc == s3_static.code_length)
        self.sync += [
            s4_valid.eq(s3_valid), s4_active.eq(s3_active), s4_slot.eq(s3_slot), s4_n.eq(s3_n), s4_strobe.eq(s3_strobe),
            s4_static.raw.eq(s3_static.raw), s4_state.raw.eq(s3_state.raw),
            s4_cos.eq(lut_cos), s4_sin.eq(lut_sin),
            s4_i.eq(s3_i), s4_q.eq(s3_q),
            s4_cp_next.eq(s3_cp_next), s4_frac_next.eq(s3_frac_next),
            s4_wrap.eq(wrap),
            If(wrap, s4_chip_next.eq(0)).Else(s4_chip_next.eq(chip_inc[:P.CHIP_BITS])),
            s4_bitsel.eq(s3_state.code_chip[:5]),
        ]

        # S4: code bit select, wipe-off products. -------------------------------------------------
        code_shifted = Signal(P.CODE_WORD_BITS)
        code_bit = Signal()
        self.comb += [
            code_shifted.eq(code_rd.dat_r >> s4_bitsel),
            code_bit.eq(code_shifted[0]),
        ]
        prod_w = P.SAMPLE_BITS + 8
        s5_valid, s5_active, s5_slot, s5_n, s5_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s5_static = _Fields(static_layout, "s5_static")
        s5_state  = _Fields(state_layout, "s5_state")
        s5_cp_next, s5_frac_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS)
        s5_chip_next, s5_wrap, s5_bit = Signal(P.CHIP_BITS), Signal(), Signal()
        s5_ic, s5_qs, s5_qc, s5_is = [Signal((prod_w, True)) for _ in range(4)]
        self.sync += [
            s5_valid.eq(s4_valid), s5_active.eq(s4_active), s5_slot.eq(s4_slot), s5_n.eq(s4_n), s5_strobe.eq(s4_strobe),
            s5_static.raw.eq(s4_static.raw), s5_state.raw.eq(s4_state.raw),
            s5_cp_next.eq(s4_cp_next), s5_frac_next.eq(s4_frac_next),
            s5_chip_next.eq(s4_chip_next), s5_wrap.eq(s4_wrap), s5_bit.eq(code_bit),
            s5_ic.eq(s4_i * s4_cos),
            s5_qs.eq(s4_q * s4_sin),
            s5_qc.eq(s4_q * s4_cos),
            s5_is.eq(s4_i * s4_sin),
        ]

        # S5: baseband sums, history shift, tap selection. ----------------------------------------
        bb_w = prod_w + 1
        hist_rep_new  = Signal(P.HISTORY)
        hist_wrap_new = Signal(P.HISTORY)
        self.comb += [
            hist_rep_new.eq(Cat(s5_bit, s5_state.hist_rep[:P.HISTORY - 1])),
            hist_wrap_new.eq(Cat(s5_wrap, s5_state.hist_wrap[:P.HISTORY - 1])),
        ]
        s6_valid, s6_active, s6_slot, s6_n, s6_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s6_static = _Fields(static_layout, "s6_static")
        s6_state  = _Fields(state_layout, "s6_state")
        s6_cp_next, s6_frac_next, s6_chip_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS), Signal(P.CHIP_BITS)
        s6_hist_rep, s6_hist_wrap = Signal(P.HISTORY), Signal(P.HISTORY)
        s6_bb_i, s6_bb_q = Signal((bb_w, True)), Signal((bb_w, True))
        s6_neg = [Signal(name=f"s6_neg{k}") for k in range(P.MAX_TAPS)]
        s6_prompt_wrap = Signal()
        d = [getattr(s5_static, f"d{k}") for k in range(P.MAX_TAPS)]
        tap_shifted = [Signal(P.HISTORY, name=f"s5_tap_shifted{k}") for k in range(P.MAX_TAPS)]
        wrap_shifted = Signal(P.HISTORY)
        self.comb += [
            *[tap_shifted[k].eq(hist_rep_new >> d[k]) for k in range(P.MAX_TAPS)],
            wrap_shifted.eq(hist_wrap_new >> s5_static.lead),
        ]
        self.sync += [
            s6_valid.eq(s5_valid), s6_active.eq(s5_active), s6_slot.eq(s5_slot), s6_n.eq(s5_n), s6_strobe.eq(s5_strobe),
            s6_static.raw.eq(s5_static.raw), s6_state.raw.eq(s5_state.raw),
            s6_cp_next.eq(s5_cp_next), s6_frac_next.eq(s5_frac_next), s6_chip_next.eq(s5_chip_next),
            s6_hist_rep.eq(hist_rep_new), s6_hist_wrap.eq(hist_wrap_new),
            s6_bb_i.eq(s5_ic + s5_qs),
            s6_bb_q.eq(s5_qc - s5_is),
            *[s6_neg[k].eq(tap_shifted[k][0]) for k in range(P.MAX_TAPS)],
            s6_prompt_wrap.eq(wrap_shifted[0]),
        ]

        # S6: accumulate. -------------------------------------------------------------------------
        sum_w = P.ACC_BITS + 1
        warming = Signal()
        self.comb += warming.eq(s6_state.warm != 0)
        s7_valid, s7_active, s7_slot, s7_n, s7_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s7_static = _Fields(static_layout, "s7_static")
        s7_cp_next, s7_frac_next, s7_chip_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS), Signal(P.CHIP_BITS)
        s7_hist_rep, s7_hist_wrap = Signal(P.HISTORY), Signal(P.HISTORY)
        s7_integ = Signal(P.INTEG_BITS)
        s7_warm  = Signal(P.HISTORY_BITS)
        s7_sat_in = Signal()
        s7_dump  = Signal()
        s7_sums  = {}
        acc_stmts = []
        for k in range(P.MAX_TAPS):
            for part, bb in (("i", s6_bb_i), ("q", s6_bb_q)):
                acc = getattr(s6_state, f"acc{k}_{part}")
                acc_s = Signal((P.ACC_BITS, True), name=f"s6_acc{k}_{part}")
                self.comb += acc_s.eq(acc)
                total = Signal((sum_w, True), name=f"s7_sum{k}_{part}", reset_less=True)
                use = Signal(name=f"s6_use{k}_{part}")
                self.comb += use.eq(~warming & (s6_static.num_taps > k))
                acc_stmts.append(
                    If(use,
                        If(s6_neg[k],
                            total.eq(acc_s - bb),
                        ).Else(
                            total.eq(acc_s + bb),
                        )
                    ).Else(
                        total.eq(acc_s),
                    )
                )
                s7_sums[(k, part)] = total
        self.sync += acc_stmts
        self.sync += [
            s7_valid.eq(s6_valid), s7_active.eq(s6_active), s7_slot.eq(s6_slot), s7_n.eq(s6_n), s7_strobe.eq(s6_strobe),
            s7_static.raw.eq(s6_static.raw),
            s7_cp_next.eq(s6_cp_next), s7_frac_next.eq(s6_frac_next), s7_chip_next.eq(s6_chip_next),
            s7_hist_rep.eq(s6_hist_rep), s7_hist_wrap.eq(s6_hist_wrap),
            s7_sat_in.eq(s6_state.sat),
            If(warming,
                s7_warm.eq(s6_state.warm - 1),
                s7_integ.eq(s6_state.integ),
                s7_dump.eq(0),
            ).Else(
                s7_warm.eq(0),
                s7_integ.eq(s6_state.integ + 1),
                s7_dump.eq(s6_prompt_wrap),
            ),
        ]

        # S7: saturate, write back, emit. ---------------------------------------------------------
        top, sign = P.ACC_BITS, P.ACC_BITS - 1
        max_pos = C((1 << (P.ACC_BITS - 1)) - 1, P.ACC_BITS)
        max_neg = C(1 << (P.ACC_BITS - 1), P.ACC_BITS)
        sat_vals = {}
        any_clip = []
        for (k, part), total in s7_sums.items():
            v = Signal(P.ACC_BITS, name=f"s7_acc{k}_{part}")
            pos = Signal(name=f"s7_pos{k}_{part}")
            neg = Signal(name=f"s7_neg{k}_{part}")
            self.comb += [
                pos.eq(~total[top] & total[sign]),
                neg.eq(total[top] & ~total[sign]),
                If(pos, v.eq(max_pos)).Elif(neg, v.eq(max_neg)).Else(v.eq(total[:P.ACC_BITS])),
            ]
            sat_vals[(k, part)] = v
            any_clip += [pos, neg]
        sat_now = Signal()
        self.comb += sat_now.eq(s7_sat_in | reduce(or_, any_clip))

        w = _Fields(state_layout, "s7_wb")
        self.comb += [
            w.carrier_phase.eq(s7_cp_next),
            w.code_frac.eq(s7_frac_next),
            w.code_chip.eq(s7_chip_next),
            w.hist_rep.eq(s7_hist_rep),
            w.hist_wrap.eq(s7_hist_wrap),
            w.warm.eq(s7_warm),
            If(s7_dump,
                w.integ.eq(0),
                w.sat.eq(0),
                *[getattr(w, f"acc{k}_{part}").eq(0) for (k, part) in sat_vals],
            ).Else(
                w.integ.eq(s7_integ),
                w.sat.eq(sat_now),
                *[getattr(w, f"acc{k}_{part}").eq(v) for (k, part), v in sat_vals.items()],
            ),
            state_wr.adr.eq(s7_slot),
            state_wr.dat_w.eq(w.raw),
            state_wr.we.eq(s7_valid & s7_active),
        ]

        r = self.rec
        dump_out = Signal()
        self.comb += dump_out.eq(s7_valid & s7_active & s7_dump)
        self.comb += [
            self.rec_valid.eq(dump_out | s7_strobe),
            If(dump_out,
                r.kind.eq(P.KIND_DUMP),
                r.channel.eq(s7_slot),
                r.num_taps.eq(s7_static.num_taps),
                r.antenna.eq(s7_static.antenna),
                r.flags.eq(Mux(sat_now, P.FLAG_SATURATED, 0)),
                r.assign_id.eq(s7_static.assign_id),
                r.sample_index.eq(s7_n + 1),
                r.integrated.eq(s7_integ),
                r.code_chip.eq(s7_chip_next),
                r.code_frac20.eq(s7_frac_next[12:]),
                *[getattr(r, f"tap{k}_{part}").eq(v) for (k, part), v in sat_vals.items()],
            ).Else(
                r.kind.eq(P.KIND_STROBE),
                r.sample_index.eq(s7_n + 1),
            ),
        ]


class TDMCorrelator(LiteXModule):
    """The engine with its sample FIFO, host interface and record stream.

    ``sample_valid`` / ``sample_data`` (the 64-bit RX word, accepted this cycle)
    feed it; ``source`` is the 64-bit record stream for the DMA writer.
    """
    def __init__(self, n_channels=24, code_words=4096, sample_fifo_depth=2048, record_fifo_depth=64):
        self.sample_valid = Signal()
        self.sample_data  = Signal(64)
        self.sample_index = Signal(P.SAMPLE_INDEX_BITS)   # index of the word accepted this cycle
        self.source = stream.Endpoint([("data", 64)])
        self.tag_enable = Signal()

        self.engine = engine = TDMEngine(n_channels, code_words)
        n_slots = engine.n_slots

        # CSRs ------------------------------------------------------------------------------------
        self._id      = CSRStatus(32, reset=P.GNSS_ID, description="TDM correlator id/revision.")
        self._caps    = CSRStatus(32, reset=P.capabilities_word(n_channels, code_words),
                                  description="[7:0] channels, [15:8] slots, [19:16] taps, [25:20] history, [30:26] log2 code words.")
        self._control = CSRStorage(fields=[
            CSRField("enable",        size=1, offset=0, description="Feed RX samples to the engine."),
            CSRField("record_enable", size=1, offset=1, description="Stream records to the DMA (else drop them)."),
            CSRField("reset",         size=1, offset=2, description="Hold the channels, FIFOs and counters in reset."),
            CSRField("tag",           size=1, offset=3, description="Tag each host RX word with its sample index (see RXTap)."),
        ])
        self._strobe_period = CSRStorage(32, description="Epoch-strobe period in samples (0 = none).")
        self._flush_cycles  = CSRStorage(32, reset=12500, description="Pad a DMA buffer once its first record is this many sys cycles old.")
        self._buffer_words  = CSRStorage(24, reset=64, description="64-bit words per DMA buffer (multiple of 8).")
        self._sample_count    = CSRStatus(32, description="Samples processed, low 32 bits.")
        self._sample_count_hi = CSRStatus(16, description="Samples processed, bits [47:32] (not latched).")
        self._sample_overflows = CSRStatus(32, description="Samples lost to a full sample FIFO.")
        self._record_drops     = CSRStatus(32, description="Records lost to a full record FIFO.")
        self._records          = CSRStatus(32, description="Records written to the DMA stream.")
        self._enabled  = CSRStatus(32, description="Bitmap: channel running.")
        self._pending  = CSRStatus(32, description="Bitmap: channel armed, waiting for its start sample.")
        self._missed   = CSRStatus(32, description="Bitmap: channel's start sample passed before the arm landed.")
        # Staging registers for the commands.
        self._s_carrier_freq  = CSRStorage(32)
        self._s_code_freq     = CSRStorage(32)
        self._s_carrier_phase = CSRStorage(32)
        self._s_code_frac     = CSRStorage(32)
        self._s_code_chip     = CSRStorage(P.CHIP_BITS)
        self._s_code_length   = CSRStorage(P.CHIP_BITS)
        self._s_code_base     = CSRStorage(16)
        self._s_taps          = CSRStorage(32, description="[24:0] five 5-bit history offsets, [27:25] num_taps.")
        self._s_misc          = CSRStorage(32, description="[4:0] lead, [9:5] warmup, [10] antenna, [31:16] assignment id.")
        self._s_start_lo      = CSRStorage(32)
        self._s_start_hi      = CSRStorage(16)
        self._cmd       = CSRStorage(32, description="Write [1:0] op (1 word, 2 arm, 3 release), [15:8] channel.")
        self._code_addr = CSRStorage(16, description="Code-memory word address for the next code_data write.")
        self._code_data = CSRStorage(32, description="Write one code word at code_addr, then increment it.")

        # # #

        rst = self._control.fields.reset
        self.comb += self.tag_enable.eq(self._control.fields.tag)

        # Sample FIFO -----------------------------------------------------------------------------
        fifo = ResetInserter()(SyncFIFOBuffered(P.SAMPLE_INDEX_BITS + 48, sample_fifo_depth))
        self.sample_fifo = fifo
        d = self.sample_data
        overflows = Signal(32)
        self.comb += [
            fifo.reset.eq(rst),
            fifo.we.eq(self.sample_valid & self._control.fields.enable),
            fifo.din.eq(Cat(self.sample_index,
                            d[0:12], d[16:28], d[32:44], d[48:60])),
        ]
        self.sync += If(rst, overflows.eq(0)).Elif(fifo.we & ~fifo.writable, overflows.eq(overflows + 1))
        self.comb += self._sample_overflows.status.eq(overflows)

        # Rounds ----------------------------------------------------------------------------------
        slot     = Signal(8)
        in_round = Signal()
        cur_n    = Signal(P.SAMPLE_INDEX_BITS)
        cur_s    = [Signal((P.SAMPLE_BITS, True)) for _ in range(4)]
        processed = Signal(P.SAMPLE_INDEX_BITS)
        strobe_cnt = Signal(32)
        strobe_due = Signal()
        period = self._strobe_period.storage
        self.comb += fifo.re.eq(~in_round & fifo.readable & ~rst)
        self.sync += [
            If(rst,
                in_round.eq(0), slot.eq(0), strobe_cnt.eq(0), strobe_due.eq(0),
            ).Elif(fifo.re,
                in_round.eq(1),
                slot.eq(0),
                cur_n.eq(fifo.dout[:P.SAMPLE_INDEX_BITS]),
                *[cur_s[k].eq(fifo.dout[P.SAMPLE_INDEX_BITS + 12 * k:P.SAMPLE_INDEX_BITS + 12 * (k + 1)]) for k in range(4)],
                processed.eq(fifo.dout[:P.SAMPLE_INDEX_BITS] + 1),
                If((period != 0) & (strobe_cnt == period - 1),
                    strobe_due.eq(1), strobe_cnt.eq(0),
                ).Else(
                    strobe_due.eq(0), strobe_cnt.eq(strobe_cnt + 1),
                ),
            ).Elif(in_round,
                If(slot == n_slots - 1,
                    in_round.eq(0),
                ).Else(
                    slot.eq(slot + 1),
                ),
            ),
        ]
        self.comb += [
            engine.issue.eq(in_round),
            engine.slot.eq(slot),
            engine.sample_index.eq(cur_n),
            *[engine.samples[k].eq(cur_s[k]) for k in range(4)],
            engine.strobe_due.eq(strobe_due),
            self._sample_count.status.eq(processed[:32]),
            self._sample_count_hi.status.eq(processed[32:48]),
        ]

        # Commands and flags ----------------------------------------------------------------------
        cmd_we = self._cmd.re
        op = self._cmd.storage[0:2]
        ch = self._cmd.storage[8:16]
        cmd_word    = Signal()
        cmd_arm     = Signal()
        cmd_release = Signal()
        self.comb += [
            cmd_word.eq(cmd_we & (op == P.CMD_WORD) & (ch < n_channels)),
            cmd_arm.eq(cmd_we & (op == P.CMD_ARM) & (ch < n_channels)),
            cmd_release.eq(cmd_we & (op == P.CMD_RELEASE) & (ch < n_channels)),
        ]
        taps = self._s_taps.storage
        misc = self._s_misc.storage
        static_fields = {
            "code_length": self._s_code_length.storage,
            "code_base":   self._s_code_base.storage,
            "lead":        misc[0:5],
            **{f"d{k}": taps[5 * k:5 * (k + 1)] for k in range(P.MAX_TAPS)},
            "num_taps":    taps[25:28],
            "warmup":      misc[5:10],
            "antenna":     misc[10],
            "assign_id":   misc[16:32],
        }
        pending_fields = {
            "start":         Cat(self._s_start_lo.storage, self._s_start_hi.storage),
            "carrier_phase": self._s_carrier_phase.storage,
            "code_chip":     self._s_code_chip.storage,
            "code_frac":     self._s_code_frac.storage,
        }
        self.comb += [
            engine.cfg_freq_wr.adr.eq(ch),
            engine.cfg_freq_wr.dat_w.eq(Cat(self._s_carrier_freq.storage, self._s_code_freq.storage)),
            engine.cfg_freq_wr.we.eq(cmd_word | cmd_arm),
            engine.cfg_static_wr.adr.eq(ch),
            engine.cfg_static_wr.dat_w.eq(Cat(*[static_fields[n] for n, _ in engine.static_layout])),
            engine.cfg_static_wr.we.eq(cmd_arm),
            engine.cfg_pending_wr.adr.eq(ch),
            engine.cfg_pending_wr.dat_w.eq(Cat(*[pending_fields[n] for n, _ in engine.pending_layout])),
            engine.cfg_pending_wr.we.eq(cmd_arm),
        ]
        missed_flags = [Signal(name=f"missed{i}") for i in range(n_channels)]
        for i in range(n_channels):
            en, pend, gen = engine.flag_enabled[i], engine.flag_pending[i], engine.flag_gen[i]
            eng_hit = Signal(name=f"eng_hit{i}")
            host_hit = Signal(name=f"host_hit{i}")
            self.comb += [
                eng_hit.eq((engine.req_slot == i) & (engine.req_gen == gen)),
                host_hit.eq(ch == i),
            ]
            # Engine requests first; a host command to the same channel in the
            # same cycle comes later in the block and wins.
            self.sync += [
                If(rst,
                    en.eq(0), pend.eq(0), missed_flags[i].eq(0), gen.eq(0),
                ).Else(
                    If(engine.req_start & eng_hit, en.eq(1), pend.eq(0)),
                    If(engine.req_missed & eng_hit, missed_flags[i].eq(1), pend.eq(0)),
                    If(cmd_arm & host_hit,
                        en.eq(0), pend.eq(1), missed_flags[i].eq(0), gen.eq(gen + 1),
                    ),
                    If(cmd_release & host_hit,
                        en.eq(0), pend.eq(0), gen.eq(gen + 1),
                    ),
                ),
            ]
        self.comb += [
            self._enabled.status.eq(Cat(*[engine.flag_enabled[i] for i in range(n_channels)])),
            self._pending.status.eq(Cat(*[engine.flag_pending[i] for i in range(n_channels)])),
            self._missed.status.eq(Cat(*missed_flags)),
        ]

        # Code memory writes ----------------------------------------------------------------------
        code_ptr = Signal(16)
        self.sync += [
            If(self._code_addr.re, code_ptr.eq(self._code_addr.storage)
            ).Elif(self._code_data.re, code_ptr.eq(code_ptr + 1)),
        ]
        self.comb += [
            engine.code_wr.adr.eq(code_ptr),
            engine.code_wr.dat_w.eq(self._code_data.storage),
            engine.code_wr.we.eq(self._code_data.re),
        ]

        # Record FIFO -----------------------------------------------------------------------------
        rec = engine.rec
        rfifo = ResetInserter()(SyncFIFOBuffered(rec.width, record_fifo_depth))
        self.record_fifo = rfifo
        drops = Signal(32)
        self.comb += [
            rfifo.reset.eq(rst),
            rfifo.we.eq(engine.rec_valid),
            rfifo.din.eq(rec.raw),
        ]
        self.sync += If(rst, drops.eq(0)).Elif(rfifo.we & ~rfifo.writable, drops.eq(drops + 1))
        self.comb += self._record_drops.status.eq(drops)

        # Serialiser: records -> 8 x 64-bit words, padded to the DMA buffer. ---------------------
        out = _Fields(_record_layout(), "ser")
        words = Array(Signal(64, name=f"ser_w{k}") for k in range(P.RECORD_WORDS))
        self.comb += out.raw.eq(rfifo.dout)
        w0 = Signal(64)
        self.comb += w0.eq(Cat(
            C(P.RECORD_MAGIC, 16), C(P.RECORD_VERSION, 4), out.kind, C(0, 2), out.channel,
            out.num_taps, out.antenna, out.flags, C(0, 6), out.assign_id))
        w1 = Signal(64)
        w2 = Signal(64)
        self.comb += [
            w1.eq(Cat(out.sample_index, C(0, 16))),
            w2.eq(Cat(out.integrated, out.code_chip, out.code_frac20)),
        ]
        tap_words = [Cat(getattr(out, f"tap{k}_i"), getattr(out, f"tap{k}_q")) for k in range(P.MAX_TAPS)]

        word_idx  = Signal(3)
        buf_pos   = Signal(24)
        age       = Signal(32)
        records   = Signal(32)
        sending   = Signal()
        padding   = Signal()
        rec_en    = self._control.fields.record_enable
        last_in_buffer = Signal()
        self.comb += last_in_buffer.eq(buf_pos == self._buffer_words.storage - 1)
        src = self.source
        self.comb += [
            src.data.eq(Mux(padding, 0, words[word_idx])),
            src.valid.eq(rec_en & (sending | padding)),
            src.first.eq(0),
            src.last.eq(0),
        ]
        start_record = Signal()
        self.comb += start_record.eq(rec_en & ~sending & ~padding & rfifo.readable)
        start_pad    = Signal()
        self.comb += start_pad.eq(rec_en & ~sending & ~padding & ~rfifo.readable &
                                  (buf_pos != 0) & (age >= self._flush_cycles.storage))
        self.comb += rfifo.re.eq(start_record | (~rec_en & rfifo.readable))
        self.sync += [
            If(rst | ~rec_en,
                sending.eq(0), padding.eq(0), word_idx.eq(0), buf_pos.eq(0), age.eq(0),
            ).Else(
                If(start_record,
                    sending.eq(1), word_idx.eq(0),
                    words[0].eq(w0), words[1].eq(w1), words[2].eq(w2),
                    *[words[3 + k].eq(tap_words[k]) for k in range(P.MAX_TAPS)],
                ),
                If(start_pad, padding.eq(1)),
                If(src.valid & src.ready,
                    If(last_in_buffer, buf_pos.eq(0)).Else(buf_pos.eq(buf_pos + 1)),
                    If(sending,
                        word_idx.eq(word_idx + 1),
                        If(word_idx == P.RECORD_WORDS - 1, sending.eq(0), records.eq(records + 1)),
                    ),
                    If(padding & last_in_buffer, padding.eq(0)),
                ),
                If(buf_pos == 0, age.eq(0)).Else(age.eq(age + 1)),
            ),
        ]
        self.comb += self._records.status.eq(records)


class RXTap(LiteXModule):
    """Counts every RX word the RFIC delivers, hands each to the correlator, and
    forwards it, tagged with its sample index if ``tag_enable`` is set, to the
    host path.

    The tap never stalls the RFIC: the AD9361 PHY free-runs and drops words
    under backpressure, so a stalling host path (no reader on DMA0, a slow
    one) must not reach it, or the correlator would lose samples and its sample
    index would stop following time. Instead the host's copy goes through a
    FIFO, and a word the host path cannot take is dropped from the host's copy
    only (``host_drops``) — the host sees the hole in the tags.

    Tags: the AD9361's samples are 12-bit, sign-extended into 16-bit slots, so
    bits [15:12] of every slot carry nothing. Tagged, the i0, q0 and i1 nibbles
    hold the index's low 12 bits, and the q1 nibble holds nibble ``m`` of
    ``index >> 12``, where ``m = index[8:12]``: within every aligned block of
    4096 samples (over which ``index >> 12`` is constant) all sixteen nibbles of
    ``index >> 12`` pass by, so any 8192 consecutive host samples name their
    device indices exactly.
    """
    def __init__(self, host_fifo_depth=1024):
        self.sink   = stream.Endpoint([("data", 64)])
        self.source = stream.Endpoint([("data", 64)])
        self.tag_enable = Signal()
        self.valid = Signal()
        self.data  = Signal(64)
        self.index = Signal(P.SAMPLE_INDEX_BITS)
        self._host_drops = CSRStatus(32, description="RX words the host path could not take (dropped from DMA0 only).")

        # # #

        self.count = count = Signal(P.SAMPLE_INDEX_BITS)
        high = Signal(64)
        high_nibble = Signal(4)
        tagged = Signal(64)
        d = self.sink.data
        self.comb += [
            high.eq(count[12:]),
            high_nibble.eq(Array(high[4 * k:4 * (k + 1)] for k in range(16))[count[8:12]]),
            tagged.eq(Cat(
                d[0:12],  count[0:4],
                d[16:28], count[4:8],
                d[32:44], count[8:12],
                d[48:60], high_nibble,
            )),
        ]
        self.host_fifo = fifo = stream.SyncFIFO([("data", 64)], host_fifo_depth, buffered=True)
        drops = Signal(32)
        self.comb += [
            self.sink.ready.eq(1),
            fifo.sink.valid.eq(self.sink.valid),
            If(self.tag_enable, fifo.sink.data.eq(tagged)).Else(fifo.sink.data.eq(d)),
            fifo.source.connect(self.source),
            self.valid.eq(self.sink.valid),
            self.data.eq(d),
            self.index.eq(count),
            self._host_drops.status.eq(drops),
        ]
        self.sync += [
            If(self.valid, count.eq(count + 1)),
            If(self.sink.valid & ~fifo.sink.ready, drops.eq(drops + 1)),
        ]
