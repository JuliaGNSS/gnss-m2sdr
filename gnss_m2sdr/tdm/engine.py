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
emitted negative literals as unsigned before (docs/tdm_correlator.md,
"Two lowering faults"),
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

    Inputs: ``issue`` with ``slot``, ``sample_index``, the four 12-bit samples
    (I/Q of two antennas) and ``sample_gap`` (samples were lost before this
    one), one slot per cycle. Host-side configuration arrives on
    the memory write ports exposed below (``cfg_*_wr``, ``code_wr``) and on the
    flag requests; records leave on ``rec_valid`` / ``rec``.
    """
    def __init__(self, n_channels, code_words):
        assert 1 <= n_channels <= 32
        P.check_code_words(code_words)
        self.n_channels = n_channels
        self.n_slots    = P.num_slots(n_channels)

        # Issue port.
        self.issue        = Signal()
        self.slot         = Signal(8)
        self.sample_index = Signal(P.SAMPLE_INDEX_BITS)
        self.samples      = [Signal((P.SAMPLE_BITS, True)) for _ in range(4)]   # i0, q0, i1, q1
        self.sample_gap   = Signal()
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
            ("subchips",    P.SUBCHIP_BITS),
            ("sc_sign",     P.MAX_SUBCHIPS),
            ("sc_level",    P.MAX_SUBCHIPS),
            ("amp_hi",      P.AMP_BITS),
            ("amp_lo",      P.AMP_BITS),
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

        # Channel state RAMs: the NCOs and histories, read at issue, and the
        # accumulators, read only in time for S6 so that they do not ride the
        # pipeline from S1. Both are written back in S7.
        state_layout = [
            ("carrier_phase", P.PHASE_BITS),
            ("code_frac",     P.FRAC_BITS),
            ("code_chip",     P.CHIP_BITS),
            ("hist_rep",      P.HISTORY),
            ("hist_wrap",     P.HISTORY),
            ("hist_lvl",      P.HISTORY),
            ("warm",          P.HISTORY_BITS),
        ]
        acc_layout = [
            ("integ",         P.INTEG_BITS),
            ("sat",           1),
            ("lost",          1),
        ] + [(f"acc{k}_{part}", P.ACC_BITS) for k in range(P.MAX_TAPS) for part in ("i", "q")]
        mem_state = Memory(sum(w for _, w in state_layout), n_channels, name="tdm_state")
        mem_acc   = Memory(sum(w for _, w in acc_layout),   n_channels, name="tdm_acc")
        state_rd = mem_state.get_port()                        # synchronous
        state_wr = mem_state.get_port(write_capable=True)
        acc_rd   = mem_acc.get_port()                          # synchronous
        acc_wr   = mem_acc.get_port(write_capable=True)
        self.specials += mem_state, state_rd, state_wr, mem_acc, acc_rd, acc_wr

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
        s1_valid, s1_slot, s1_n, s1_strobe, s1_gap = Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal(), Signal()
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
            s1_gap.eq(self.sample_gap),
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
        s2_valid, s2_slot, s2_n, s2_strobe, s2_gap = Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal(), Signal()
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
            s2_gap.eq(s1_gap),
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
            If(start,                                  # the accumulators are cleared in S5
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
        s3_gap, s3_start = Signal(), Signal()
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
            # Samples lost before a channel's start sample do not concern it.
            s3_gap.eq(s2_gap & ~start),
            s3_start.eq(start),
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

        # S3: code-memory address out, carrier table scaled by the two replica
        # amplitudes, sub-chip index, chip increment. ----------------------------------------------
        self.comb += code_rd.adr.eq(s3_code_adr)
        lut_cos = Signal(8)
        lut_sin = Signal(8)
        self.comb += Case(s3_lut, {
            k: [lut_cos.eq(cos_t[k] & 0xFF), lut_sin.eq(sin_t[k] & 0xFF)] for k in range(len(cos_t))
        })
        lut_cos_s, lut_sin_s = Signal((8, True)), Signal((8, True))
        amp_hi_s, amp_lo_s = Signal((P.AMP_BITS + 1, True)), Signal((P.AMP_BITS + 1, True))
        self.comb += [
            lut_cos_s.eq(lut_cos), lut_sin_s.eq(lut_sin),
            # Zero-extended into signed signals: every product below is signed x signed.
            amp_hi_s.eq(s3_static.amp_hi), amp_lo_s.eq(s3_static.amp_lo),
        ]
        sc_w = 8 + P.AMP_BITS + 1
        # floor(frac * P): the full product, since a sub-chip edge at k/12 chip is not dyadic.
        sub_prod = Signal(P.FRAC_BITS + P.SUBCHIP_BITS)
        self.comb += sub_prod.eq(s3_state.code_frac * s3_static.subchips)
        chip_inc = Signal(P.CHIP_BITS + 1)
        self.comb += chip_inc.eq(s3_state.code_chip + s3_carry)
        s4_valid, s4_active, s4_slot, s4_n, s4_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s4_gap, s4_start = Signal(), Signal()
        s4_static = _Fields(static_layout, "s4_static")
        s4_state  = _Fields(state_layout, "s4_state")
        s4_cos_hi, s4_sin_hi = Signal((sc_w, True)), Signal((sc_w, True))
        s4_cos_lo, s4_sin_lo = Signal((sc_w, True)), Signal((sc_w, True))
        s4_sub = Signal(P.SUBCHIP_BITS)
        s4_i, s4_q = Signal((P.SAMPLE_BITS, True)), Signal((P.SAMPLE_BITS, True))
        s4_cp_next, s4_frac_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS)
        s4_chip_next, s4_wrap = Signal(P.CHIP_BITS), Signal()
        s4_bitsel = Signal(5)
        wrap = Signal()
        self.comb += wrap.eq(chip_inc == s3_static.code_length)
        self.sync += [
            s4_valid.eq(s3_valid), s4_active.eq(s3_active), s4_slot.eq(s3_slot), s4_n.eq(s3_n), s4_strobe.eq(s3_strobe),
            s4_gap.eq(s3_gap), s4_start.eq(s3_start),
            s4_static.raw.eq(s3_static.raw), s4_state.raw.eq(s3_state.raw),
            s4_cos_hi.eq(lut_cos_s * amp_hi_s), s4_sin_hi.eq(lut_sin_s * amp_hi_s),
            s4_cos_lo.eq(lut_cos_s * amp_lo_s), s4_sin_lo.eq(lut_sin_s * amp_lo_s),
            s4_sub.eq(sub_prod[P.FRAC_BITS:]),
            s4_i.eq(s3_i), s4_q.eq(s3_q),
            s4_cp_next.eq(s3_cp_next), s4_frac_next.eq(s3_frac_next),
            s4_wrap.eq(wrap),
            If(wrap, s4_chip_next.eq(0)).Else(s4_chip_next.eq(chip_inc[:P.CHIP_BITS])),
            s4_bitsel.eq(s3_state.code_chip[:5]),
        ]

        # S4: code bit and sub-chip select, wipe-off products, accumulator read. -----------------
        self.comb += acc_rd.adr.eq(s4_slot)
        code_shifted = Signal(P.CODE_WORD_BITS)
        sign_shifted = Signal(P.MAX_SUBCHIPS)
        level_shifted = Signal(P.MAX_SUBCHIPS)
        code_bit = Signal()
        self.comb += [
            code_shifted.eq(code_rd.dat_r >> s4_bitsel),
            sign_shifted.eq(s4_static.sc_sign >> s4_sub),
            level_shifted.eq(s4_static.sc_level >> s4_sub),
            code_bit.eq(code_shifted[0] ^ sign_shifted[0]),
        ]
        prod_w = P.SAMPLE_BITS + sc_w
        s5_valid, s5_active, s5_slot, s5_n, s5_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s5_gap, s5_start = Signal(), Signal()
        s5_static = _Fields(static_layout, "s5_static")
        s5_state  = _Fields(state_layout, "s5_state")
        s5_cp_next, s5_frac_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS)
        s5_chip_next, s5_wrap, s5_bit, s5_lvl = Signal(P.CHIP_BITS), Signal(), Signal(), Signal()
        s5_ic_hi, s5_qs_hi, s5_qc_hi, s5_is_hi = [Signal((prod_w, True)) for _ in range(4)]
        s5_ic_lo, s5_qs_lo, s5_qc_lo, s5_is_lo = [Signal((prod_w, True)) for _ in range(4)]
        self.sync += [
            s5_valid.eq(s4_valid), s5_active.eq(s4_active), s5_slot.eq(s4_slot), s5_n.eq(s4_n), s5_strobe.eq(s4_strobe),
            s5_gap.eq(s4_gap), s5_start.eq(s4_start),
            s5_static.raw.eq(s4_static.raw), s5_state.raw.eq(s4_state.raw),
            s5_cp_next.eq(s4_cp_next), s5_frac_next.eq(s4_frac_next),
            s5_chip_next.eq(s4_chip_next), s5_wrap.eq(s4_wrap), s5_bit.eq(code_bit),
            s5_lvl.eq(level_shifted[0]),
            s5_ic_hi.eq(s4_i * s4_cos_hi),
            s5_qs_hi.eq(s4_q * s4_sin_hi),
            s5_qc_hi.eq(s4_q * s4_cos_hi),
            s5_is_hi.eq(s4_i * s4_sin_hi),
            s5_ic_lo.eq(s4_i * s4_cos_lo),
            s5_qs_lo.eq(s4_q * s4_sin_lo),
            s5_qc_lo.eq(s4_q * s4_cos_lo),
            s5_is_lo.eq(s4_i * s4_sin_lo),
        ]

        # S5: baseband sums, history shift, tap selection. ----------------------------------------
        bb_w = prod_w + 1
        hist_rep_new  = Signal(P.HISTORY)
        hist_wrap_new = Signal(P.HISTORY)
        hist_lvl_new  = Signal(P.HISTORY)
        self.comb += [
            hist_rep_new.eq(Cat(s5_bit, s5_state.hist_rep[:P.HISTORY - 1])),
            hist_wrap_new.eq(Cat(s5_wrap, s5_state.hist_wrap[:P.HISTORY - 1])),
            hist_lvl_new.eq(Cat(s5_lvl, s5_state.hist_lvl[:P.HISTORY - 1])),
        ]
        s6_valid, s6_active, s6_slot, s6_n, s6_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s6_gap = Signal()
        s6_static = _Fields(static_layout, "s6_static")
        s6_state  = _Fields(state_layout, "s6_state")
        s6_acc    = _Fields(acc_layout, "s6_acc")      # a channel starting gets cleared ones
        s6_cp_next, s6_frac_next, s6_chip_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS), Signal(P.CHIP_BITS)
        s6_hist_rep, s6_hist_wrap, s6_hist_lvl = Signal(P.HISTORY), Signal(P.HISTORY), Signal(P.HISTORY)
        s6_bb_hi_i, s6_bb_hi_q = Signal((bb_w, True)), Signal((bb_w, True))
        s6_bb_lo_i, s6_bb_lo_q = Signal((bb_w, True)), Signal((bb_w, True))
        s6_neg = [Signal(name=f"s6_neg{k}") for k in range(P.MAX_TAPS)]
        s6_lvl = [Signal(name=f"s6_lvl{k}") for k in range(P.MAX_TAPS)]
        s6_prompt_wrap = Signal()
        d = [getattr(s5_static, f"d{k}") for k in range(P.MAX_TAPS)]
        tap_shifted = [Signal(P.HISTORY, name=f"s5_tap_shifted{k}") for k in range(P.MAX_TAPS)]
        lvl_shifted = [Signal(P.HISTORY, name=f"s5_lvl_shifted{k}") for k in range(P.MAX_TAPS)]
        wrap_shifted = Signal(P.HISTORY)
        self.comb += [
            *[tap_shifted[k].eq(hist_rep_new >> d[k]) for k in range(P.MAX_TAPS)],
            *[lvl_shifted[k].eq(hist_lvl_new >> d[k]) for k in range(P.MAX_TAPS)],
            wrap_shifted.eq(hist_wrap_new >> s5_static.lead),
        ]
        self.sync += [
            s6_valid.eq(s5_valid), s6_active.eq(s5_active), s6_slot.eq(s5_slot), s6_n.eq(s5_n), s6_strobe.eq(s5_strobe),
            s6_gap.eq(s5_gap),
            s6_static.raw.eq(s5_static.raw), s6_state.raw.eq(s5_state.raw),
            If(s5_start, s6_acc.raw.eq(0)).Else(s6_acc.raw.eq(acc_rd.dat_r)),
            s6_cp_next.eq(s5_cp_next), s6_frac_next.eq(s5_frac_next), s6_chip_next.eq(s5_chip_next),
            s6_hist_rep.eq(hist_rep_new), s6_hist_wrap.eq(hist_wrap_new), s6_hist_lvl.eq(hist_lvl_new),
            s6_bb_hi_i.eq(s5_ic_hi + s5_qs_hi),
            s6_bb_hi_q.eq(s5_qc_hi - s5_is_hi),
            s6_bb_lo_i.eq(s5_ic_lo + s5_qs_lo),
            s6_bb_lo_q.eq(s5_qc_lo - s5_is_lo),
            *[s6_neg[k].eq(tap_shifted[k][0]) for k in range(P.MAX_TAPS)],
            *[s6_lvl[k].eq(lvl_shifted[k][0]) for k in range(P.MAX_TAPS)],
            s6_prompt_wrap.eq(wrap_shifted[0]),
        ]

        # S6: accumulate. -------------------------------------------------------------------------
        sum_w = P.ACC_BITS + 1
        warming = Signal()
        self.comb += warming.eq(s6_state.warm != 0)
        s7_valid, s7_active, s7_slot, s7_n, s7_strobe = Signal(), Signal(), Signal(8), Signal(P.SAMPLE_INDEX_BITS), Signal()
        s7_gap = Signal()
        s7_static = _Fields(static_layout, "s7_static")
        s7_cp_next, s7_frac_next, s7_chip_next = Signal(P.PHASE_BITS), Signal(P.FRAC_BITS), Signal(P.CHIP_BITS)
        s7_hist_rep, s7_hist_wrap, s7_hist_lvl = Signal(P.HISTORY), Signal(P.HISTORY), Signal(P.HISTORY)
        s7_integ = Signal(P.INTEG_BITS)
        s7_warm  = Signal(P.HISTORY_BITS)
        s7_sat_in = Signal()
        s7_lost_in = Signal()
        s7_dump  = Signal()
        s7_sums  = {}
        acc_stmts = []
        for k in range(P.MAX_TAPS):
            for part, bb_hi, bb_lo in (("i", s6_bb_hi_i, s6_bb_lo_i), ("q", s6_bb_hi_q, s6_bb_lo_q)):
                # The tap's replica amplitude: the level its history entry carries.
                bb = Signal((bb_w, True), name=f"s6_bb{k}_{part}")
                self.comb += If(s6_lvl[k], bb.eq(bb_hi)).Else(bb.eq(bb_lo))
                acc = getattr(s6_acc, f"acc{k}_{part}")
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
            s7_gap.eq(s6_gap),
            s7_static.raw.eq(s6_static.raw),
            s7_cp_next.eq(s6_cp_next), s7_frac_next.eq(s6_frac_next), s7_chip_next.eq(s6_chip_next),
            s7_hist_rep.eq(s6_hist_rep), s7_hist_wrap.eq(s6_hist_wrap), s7_hist_lvl.eq(s6_hist_lvl),
            s7_sat_in.eq(s6_acc.sat),
            s7_lost_in.eq(s6_acc.lost),
            If(warming,
                s7_warm.eq(s6_state.warm - 1),
                s7_integ.eq(s6_acc.integ),
                s7_dump.eq(0),
            ).Else(
                s7_warm.eq(0),
                s7_integ.eq(s6_acc.integ + 1),
                s7_dump.eq(s6_prompt_wrap),
            ),
        ]

        # S7: saturate, write back, emit. ---------------------------------------------------------
        # The write-back below is in stage S7; num_slots() keeps a slot's next
        # issue from reading the state before it lands.
        assert P.WRITEBACK_STAGE == 7 and self.n_slots >= P.WRITEBACK_STAGE
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
        lost_now = Signal()
        self.comb += [
            sat_now.eq(s7_sat_in | reduce(or_, any_clip)),
            lost_now.eq(s7_lost_in | s7_gap),
        ]

        w = _Fields(state_layout, "s7_wb")
        wa = _Fields(acc_layout, "s7_wb_acc")
        self.comb += [
            w.carrier_phase.eq(s7_cp_next),
            w.code_frac.eq(s7_frac_next),
            w.code_chip.eq(s7_chip_next),
            w.hist_rep.eq(s7_hist_rep),
            w.hist_wrap.eq(s7_hist_wrap),
            w.hist_lvl.eq(s7_hist_lvl),
            w.warm.eq(s7_warm),
            If(s7_dump,
                wa.raw.eq(0),
            ).Else(
                wa.integ.eq(s7_integ),
                wa.sat.eq(sat_now),
                wa.lost.eq(lost_now),
                *[getattr(wa, f"acc{k}_{part}").eq(v) for (k, part), v in sat_vals.items()],
            ),
            state_wr.adr.eq(s7_slot),
            state_wr.dat_w.eq(w.raw),
            state_wr.we.eq(s7_valid & s7_active),
            acc_wr.adr.eq(s7_slot),
            acc_wr.dat_w.eq(wa.raw),
            acc_wr.we.eq(s7_valid & s7_active),
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
                r.flags.eq(Mux(sat_now, P.FLAG_SATURATED, 0) | Mux(lost_now, P.FLAG_SAMPLE_OVERFLOW, 0)),
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
            CSRField("record_enable", size=1, offset=1, description="Stream records to the DMA (else drop them). Cleared, the record in flight is finished and its DMA buffer padded out."),
            CSRField("reset",         size=1, offset=2, description="Hold the channels, FIFOs, serialiser, strobes and counters in reset (not sample_count, which follows the device sample index)."),
            CSRField("tag",           size=1, offset=3, description="Tag each host RX word with its sample index (see RXTap)."),
        ])
        self._strobe_period = CSRStorage(32, description="Epoch strobes on the multiples of this many samples of the sample index (0 = none).")
        self._flush_cycles  = CSRStorage(32, reset=12500, description="Pad a DMA buffer once its first record is this many sys cycles old.")
        self._buffer_words  = CSRStorage(24, reset=64, description="64-bit words per DMA buffer (multiple of 8).")
        self._sample_count    = CSRStatus(32, description="Sample index one past the last sample processed, low 32 bits.")
        self._sample_count_hi = CSRStatus(16, description="Sample index one past the last sample processed, bits [47:32] (not latched).")
        self._sample_overflows = CSRStatus(32, description="Samples lost to a full sample FIFO (the records over them carry FLAG_SAMPLE_OVERFLOW).")
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
        self._s_subcarrier    = CSRStorage(32, reset=P.SUBCARRIER_BPSK,
                                           description="Replica sub-chips: [11:0] sign (1 = negative), [23:12] level (1 = amp_hi), [27:24] sub-chips per chip.")
        self._s_amplitude     = CSRStorage(16, reset=P.AMPLITUDE_UNIT,
                                           description="Replica amplitudes: [6:0] amp_hi, [14:8] amp_lo.")
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
        processed = Signal(P.SAMPLE_INDEX_BITS)    # one past the last sample popped
        seen     = Signal()                         # a sample was popped since the reset
        cur_gap  = Signal()                         # samples were lost before cur_n
        popped   = Signal()                         # the first cycle of a round
        strobe_due = Signal()
        self.comb += fifo.re.eq(~in_round & fifo.readable & ~rst)
        self.sync += [
            popped.eq(fifo.re),
            If(rst,
                in_round.eq(0), slot.eq(0), seen.eq(0),
            ).Elif(fifo.re,
                in_round.eq(1),
                slot.eq(0),
                cur_n.eq(fifo.dout[:P.SAMPLE_INDEX_BITS]),
                *[cur_s[k].eq(fifo.dout[P.SAMPLE_INDEX_BITS + 12 * k:P.SAMPLE_INDEX_BITS + 12 * (k + 1)]) for k in range(4)],
                processed.eq(fifo.dout[:P.SAMPLE_INDEX_BITS] + 1),
                seen.eq(1),
                cur_gap.eq(seen & (fifo.dout[:P.SAMPLE_INDEX_BITS] != processed)),
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
            engine.sample_gap.eq(cur_gap),
            engine.strobe_due.eq(strobe_due),
            self._sample_count.status.eq(processed[:32]),
            self._sample_count_hi.status.eq(processed[32:48]),
        ]

        # Epoch strobes (params.py, "Epoch strobes"). ------------------------------------------------
        # In a round's first cycle ``processed`` is the new sample's index + 1:
        # it either anchors the period's phase or is compared against
        # ``strobe_next``; ``strobe_due`` is read in the round's last slot. The
        # anchor's next multiple of the period comes from a bit-serial
        # remainder, done before the anchor's STROBE_SYNC_SAMPLES-th successor.
        W = P.STROBE_SYNC_SAMPLES
        div_steps = P.SAMPLE_INDEX_BITS + 1
        assert W * (n_slots + 1) > div_steps + 2, "the strobe anchor's division must end before its first check"
        period      = self._strobe_period.storage
        strobe_next = Signal(P.SAMPLE_INDEX_BITS + 1)
        need_sync   = Signal(reset=1)              # anchor the phase at the next sample
        synced      = Signal()                     # strobe_next is valid
        hold        = Signal(max=W)                # samples still to pass before the first check
        stale_check = Signal()
        div_busy    = Signal()
        div_count   = Signal(max=div_steps + 1)
        div_x       = Signal(P.SAMPLE_INDEX_BITS + 1)   # the anchor's index + 1 + W
        div_bits    = Signal(P.SAMPLE_INDEX_BITS + 1)   # div_x, shifted out MSB first
        div_rem     = Signal(33)
        div_trial   = Signal(33)
        self.comb += div_trial.eq(Cat(div_bits[-1], div_rem[:32]))
        self.sync += [
            If(rst | self._strobe_period.re,
                need_sync.eq(1), synced.eq(0), hold.eq(0), strobe_due.eq(0), stale_check.eq(0),
                div_busy.eq(0),
            ).Else(
                stale_check.eq(0),
                If(popped & (period != 0),
                    strobe_due.eq(0),
                    If(need_sync,
                        need_sync.eq(0), synced.eq(0), hold.eq(W - 1),
                        div_busy.eq(1), div_count.eq(div_steps), div_rem.eq(0),
                        div_x.eq(processed + W), div_bits.eq(processed + W),
                    ).Elif(hold != 0,
                        hold.eq(hold - 1),
                    ).Elif(synced & (processed >= strobe_next),
                        strobe_due.eq(1),
                        strobe_next.eq(strobe_next + period),
                        stale_check.eq(1),
                    ),
                ).Elif(popped,
                    strobe_due.eq(0),
                ),
                # A gap longer than a period: the next multiple is still behind.
                If(stale_check & (strobe_next <= processed), need_sync.eq(1)),
                If(div_busy,
                    If(div_count != 0,
                        div_count.eq(div_count - 1),
                        div_bits.eq(div_bits << 1),
                        If(div_trial >= period,
                            div_rem.eq(div_trial - period),
                        ).Else(
                            div_rem.eq(div_trial),
                        ),
                    ).Else(
                        div_busy.eq(0), synced.eq(1),
                        If(div_rem == 0,
                            strobe_next.eq(div_x),
                        ).Else(
                            strobe_next.eq(div_x - div_rem + period),
                        ),
                    ),
                ),
            ),
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
            "subchips":    self._s_subcarrier.storage[2 * P.MAX_SUBCHIPS:2 * P.MAX_SUBCHIPS + P.SUBCHIP_BITS],
            "sc_sign":     self._s_subcarrier.storage[0:P.MAX_SUBCHIPS],
            "sc_level":    self._s_subcarrier.storage[P.MAX_SUBCHIPS:2 * P.MAX_SUBCHIPS],
            "amp_hi":      self._s_amplitude.storage[0:P.AMP_BITS],
            "amp_lo":      self._s_amplitude.storage[8:8 + P.AMP_BITS],
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
            src.valid.eq(sending | padding),
            src.first.eq(0),
            src.last.eq(0),
        ]
        # Clearing record_enable stops at a buffer boundary: the record in
        # flight is finished and its buffer padded out, so the DMA's buffers
        # stay whole records; records that come meanwhile are dropped.
        idle = Signal()
        self.comb += idle.eq(~sending & ~padding)
        start_record = Signal()
        self.comb += start_record.eq(rec_en & idle & rfifo.readable)
        start_pad    = Signal()
        self.comb += start_pad.eq(idle & (buf_pos != 0) &
                                  (~rec_en | (~rfifo.readable & (age >= self._flush_cycles.storage))))
        self.comb += rfifo.re.eq(start_record | (~rec_en & rfifo.readable))
        self.sync += [
            If(rst,
                sending.eq(0), padding.eq(0), word_idx.eq(0), buf_pos.eq(0), age.eq(0), records.eq(0),
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
        self._host_drops = CSRStatus(32, description="RX words the host path could not take (dropped from DMA0 only). Free-running, like the sample index: the correlator's reset does not touch the tap.")

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
