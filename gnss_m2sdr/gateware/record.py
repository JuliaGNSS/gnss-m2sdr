#
# This file is part of gnss-m2sdr.
#
# Correlator-dump recorder: channel dumps -> 64-bit record stream (for DMA1).
# SPDX-License-Identifier: BSD-2-Clause

"""Serialize per-channel correlator dumps into the DMA record stream.

Each channel's dump (produced once per code period) is latched into a holding
register with a per-channel sequence counter; an overflow flag is raised if a
new dump arrives before the previous one has been serialized (should not happen
at ~1 kHz dumps) -- a dump landing on the very cycle the previous record is
retired still fits, because the holding register frees on that cycle. A
round-robin FSM emits the 16-word record (see
record_format.py) for each pending channel into an output SyncFIFO, whose
source is connected to the DMA1 sink at SoC level.

The record is 128 bytes rather than the 80 the two-antenna payload needs so that
it divides litepcie's 8192-byte DMA buffer exactly (8192 % 48 = 32, and 80 and 96
leave 32 too, which would leave the stream permanently misaligned after the
driver drops a buffer), and every record carries RECORD_MAGIC in the upper half
of word 5 as the host's resync anchor -- litepcie's DMA writer ignores the
stream's first/last, so framing has to live in the payload.

The per-antenna E/P/L blocks are always reserved, whatever `num_ants` the bank
was built with, so the host does not have to know the build option to find a
record; the words of an absent antenna read zero and the record's num_ants field
says how many are valid.

A lost dump is reported on two separate paths, because they answer different
questions:

  * `FLAG_OVERFLOW` in the *next* captured record -- transient, tells the host
    exactly where in the record stream the gap is.
  * `overflow` / `dropped[]` -- status for a host that is polling rather than
    reading the stream. Both are sticky: `overflow` stays set and `dropped[]`
    keeps counting (saturating) until the host writes 1 to the matching bit of
    `overflow_clear`. They must not self-clear on the next capture: at ~1 kHz
    dumps that would leave the bit observable for under a millisecond, so any
    realistic poll rate would miss it. A drop on the same cycle as a clear
    still wins -- the host may never lose a drop to a racing clear.

The serializer round-robins over n_channels + 1 slots: the extra one (index
n_channels, STROBE_SLOT) is the epoch strobe, a periodic timebase marker
(channel STROBE_CHANNEL on the wire) emitted every `epoch_period` input samples
straight from the free-running sample counter. It exists because the host closes
an epoch on the first record past the boundary: with correlator dumps as the
only trigger, a receiver with nothing locked never closes one at all (see
record_format.py).

It is a slot like any other, and that is literal, not a slogan: pend/seqs/
retire/flag_next/overflow/dropped are all sized n_slots and the capture,
overflow and drop-counting logic is the single `slot_status()` block below,
shared with the channels. So a marker never preempts a dump, `pend[]` keeps its
one driver, and a dropped marker is reported exactly the way a dropped dump is
-- sticky bit n_channels of `overflow`, its own saturating `dropped` counter,
FLAG_OVERFLOW on the next marker.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litepcie.common import dma_layout

from gnss_m2sdr.record_format import (
    ACC_SIGNALS, ANT_PROMPT_WORD, MAGIC_SHIFT, MAGIC_WORD, NANTS_WORD,
    N_ANTS_MAX, RECORD_MAGIC, RECORD_WORDS, STROBE_CHANNEL,
    FLAG_EPOCH_STROBE, FLAG_OVERFLOW,
)


class ChannelDumpPort:
    """Signals a TrackingChannel drives into the recorder (one per channel).

    acc[n] is antenna n's six accumulators; antenna 0's are also exposed under
    the original scalar names (ie/qe/ip/qp/il/ql).
    """
    def __init__(self, accum_bits, code_frac_bits, num_ants=1):
        self.stb                = Signal()
        self.acc = [{k: Signal((accum_bits, True)) for k in ACC_SIGNALS}
                    for _ in range(num_ants)]
        for k, sig in self.acc[0].items():
            setattr(self, k, sig)
        self.integrated_samples = Signal(32)
        self.sample_index       = Signal(64)
        self.code_phase         = Signal(code_frac_bits)
        self.prn                = Signal(8)


class CorrelatorRecorder(LiteXModule):
    def __init__(self, n_channels, accum_bits=32, code_frac_bits=24, fifo_depth=64,
                 drop_count_bits=16, num_ants=1):
        assert code_frac_bits <= 32, "code_phase shares word 5 with the magic"
        assert 1 <= num_ants <= N_ANTS_MAX, f"1..{N_ANTS_MAX} antennas"
        self.ports  = [ChannelDumpPort(accum_bits, code_frac_bits, num_ants)
                       for _ in range(n_channels)]
        self.source = stream.Endpoint(dma_layout(64))

        # Slots = one per channel, plus the epoch strobe last. Every per-slot
        # array below is n_slots wide, so bit/index n_channels is the strobe's
        # everywhere -- there is no shorter per-channel array left to index out
        # of range.
        n_slots     = n_channels + 1
        STROBE_SLOT = n_channels

        # Host-visible status: sticky until explicitly cleared (see docstring).
        self.overflow       = Signal(n_slots)      # per-slot: a record was lost
        self.overflow_clear = Signal(n_slots)      # one-cycle write-1-to-clear
        self.dropped        = [Signal(drop_count_bits) for _ in range(n_slots)]

        # Epoch-strobe generator inputs: the bank's ungated sample strobe and
        # its free-running counter (the marker must keep ticking while the bank
        # is disabled -- that is the case the host cannot otherwise clock).
        self.sample_stb   = Signal()
        self.sample_count = Signal(64)
        self.epoch_period = Signal(32)             # input samples per marker; 0 = off

        # How many antenna blocks the records report as valid. Defaults to the
        # build-time count and may be lowered at runtime (in 1R1T the AD9361
        # delivers one antenna, whatever the gateware was built for), so it is
        # latched per dump like any other record field.
        self.num_ants = Signal(8, reset=num_ants)

        # # #

        self.fifo = fifo = stream.SyncFIFO(dma_layout(64), fifo_depth, buffered=True)
        self.comb += fifo.source.connect(self.source)

        def s32(x):  # low 32 bits of a signed accumulator
            return x[:32]

        # Per-slot holding registers + pending/seq/overflow.
        pend  = Array(Signal() for _ in range(n_slots))
        seqs  = Array(Signal(8) for _ in range(n_slots))
        hold  = []
        # Transient companion of `overflow`: consumed by the next captured
        # record as its FLAG_OVERFLOW, so exactly one record marks the gap.
        flag_next = [Signal() for _ in range(n_slots)]
        drop_max  = (1 << drop_count_bits) - 1

        # `pend` must have exactly one driver.  The serializer FSM below only
        # pulses `retiring` combinationally (Migen flattens submodule fragments
        # after the parent's, so a NextValue(pend[ch], 0) inside the FSM would
        # silently override a same-cycle capture here); the capture block owns
        # the flag.  `retire[i]` marks the cycle on which channel i's holding
        # register is freed, so a dump strobing on that cycle is captured -- it
        # is a fresh integration, not an overflow.
        ch       = Signal(max=max(2, n_slots))
        retiring = Signal()
        retire   = Signal(n_slots)
        self.comb += [retire[i].eq(retiring & (ch == i)) for i in range(n_slots)]

        def slot_status(i, stb, capture):
            """Capture/overflow bookkeeping for slot `i` -- identical for every slot.

            `capture` is the list of statements that latch a new record into
            slot i's holding register. Everything around it (the pend/retire
            rule, the sticky bit, the transient flag, the saturating counter and
            its race with a host clear) is shared, so the epoch strobe cannot
            drift from the channels as this logic evolves.
            """
            # Counter base: 0 when a clear lands on this cycle, so a colliding
            # drop restarts the count at 1 instead of resurrecting the old one.
            base = Signal(drop_count_bits)
            self.comb += base.eq(Mux(self.overflow_clear[i], 0, self.dropped[i]))
            return [
                # The host clear comes first so the drop below overrides it on a
                # colliding cycle (in a Migen sync block the last assignment to a
                # signal wins), i.e. a drop is never lost to a racing clear.
                If(self.overflow_clear[i],
                    self.overflow[i].eq(0),
                    self.dropped[i].eq(0),
                ),
                If(stb,
                    If(pend[i] & ~retire[i],
                        flag_next[i].eq(1),         # next record carries FLAG_OVERFLOW
                        self.overflow[i].eq(1),     # sticky until the host clears it
                        If(base != drop_max,        # saturating drop counter
                            self.dropped[i].eq(base + 1),
                        ),
                    ).Else(
                        pend[i].eq(1),
                        *capture,
                        seqs[i].eq(seqs[i] + 1),
                        flag_next[i].eq(0),  # transient flag consumed by this record
                    ),
                ).Elif(retire[i],
                    pend[i].eq(0),
                ),
            ]

        for i, p in enumerate(self.ports):
            h = dict(
                # cphase is held zero-extended to the 32-bit wire field so the
                # magic can share word 5 at a fixed offset.
                sidx=Signal(64), nsamp=Signal(32), cphase=Signal(32),
                prn=Signal(8), seq=Signal(8), flags=Signal(8), nants=Signal(8),
                # One 32-bit wire field per accumulator per antenna.
                acc=[{k: Signal(32) for k in ACC_SIGNALS} for _ in range(num_ants)],
            )
            hold.append(h)
            self.sync += slot_status(i, p.stb, [
                h["sidx"].eq(p.sample_index),
                h["nsamp"].eq(p.integrated_samples),
                h["cphase"].eq(p.code_phase),
                h["prn"].eq(p.prn),
                h["seq"].eq(seqs[i]),
                h["flags"].eq(Mux(flag_next[i], FLAG_OVERFLOW, 0)),
                h["nants"].eq(self.num_ants),
                *[h["acc"][n][k].eq(s32(p.acc[n][k]))
                  for n in range(num_ants) for k in ACC_SIGNALS],
            ])

        # Epoch strobe: one marker every `epoch_period` *input samples*, so the
        # divider counts sample strobes, not sys_clk cycles (fs << sys_clk).
        # `count` is re-armed while the period is 0, which makes enabling the
        # marker start a fresh epoch at a known sample instead of continuing a
        # stale phase; changing a non-zero period just carries the current
        # phase over (at most one late marker).
        count  = Signal(32)
        strobe = Signal()
        self.comb += strobe.eq(self.sample_stb & (self.epoch_period != 0) & (count == 0))
        self.sync += [
            If(self.epoch_period == 0,
                count.eq(0),
            ).Elif(self.sample_stb,
                If(count == 0,
                    count.eq(self.epoch_period - 1),
                ).Else(
                    count.eq(count - 1),
                ),
            ),
        ]

        # The strobe's holding register is the last slot. Only sample_index/seq/
        # flags mean anything, so the rest of the record is wired to constants --
        # a marker is not a correlator dump and the host must not read a payload
        # out of it. That includes every antenna block and the num_ants word: a
        # strobe record reads zero there, and unpack_record() clamps num_ants up
        # to 1 rather than indexing a block that carries nothing.
        sh = dict(
            sidx=Signal(64), nsamp=C(0, 32), cphase=C(0, 32), prn=C(0, 8),
            seq=Signal(8), flags=Signal(8), nants=C(0, 8),
            acc=[{k: C(0, 32) for k in ACC_SIGNALS} for _ in range(num_ants)],
        )
        hold.append(sh)
        self.sync += slot_status(STROBE_SLOT, strobe, [
            sh["sidx"].eq(self.sample_count),
            sh["seq"].eq(seqs[STROBE_SLOT]),
            sh["flags"].eq(Mux(flag_next[STROBE_SLOT],
                               FLAG_EPOCH_STROBE | FLAG_OVERFLOW, FLAG_EPOCH_STROBE)),
        ])

        # Round-robin serializer FSM (`ch` is declared with `retire` above).
        widx = Signal(max=RECORD_WORDS)
        word = Signal(64)

        # Selected channel's holding register -> current record word. Words the
        # layout does not assign stay zero: the reserved tail that keeps the
        # record at 128 B (so it divides the DMA buffer), and the block of any
        # antenna this build does not have.
        def build_words(h, chan_id):
            words = [C(0, 64)] * RECORD_WORDS
            words[0] = h["sidx"]
            words[1] = Cat(h["seq"], h["flags"], h["prn"], C(chan_id, 8), h["nsamp"])  # low..high
            words[MAGIC_WORD] = Cat(h["cphase"], C(RECORD_MAGIC, 64 - MAGIC_SHIFT))
            words[NANTS_WORD] = Cat(h["nants"], C(0, 56))
            for n, a in enumerate(h["acc"]):
                ant_word = ANT_PROMPT_WORD[n]
                words[ant_word + 0] = Cat(a["ip"], a["qp"])
                words[ant_word + 1] = Cat(a["ie"], a["qe"])
                words[ant_word + 2] = Cat(a["il"], a["ql"])
            return words
        cases = {}
        for i, h in enumerate(hold):
            words_i = build_words(h, STROBE_CHANNEL if i == STROBE_SLOT else i)
            cases[i] = word.eq(Array(words_i)[widx])
        self.comb += Case(ch, cases)

        self.fsm = fsm = FSM(reset_state="SCAN")
        fsm.act("SCAN",
            If(pend[ch],
                NextState("EMIT"),
            ).Else(
                NextValue(ch, Mux(ch == (n_slots - 1), 0, ch + 1)),
            ),
        )
        fsm.act("EMIT",
            fifo.sink.valid.eq(1),
            fifo.sink.data.eq(word),
            fifo.sink.first.eq(widx == 0),
            fifo.sink.last.eq(widx == (RECORD_WORDS - 1)),
            If(fifo.sink.ready,
                If(widx == (RECORD_WORDS - 1),
                    NextValue(widx, 0),
                    retiring.eq(1),  # comb: the capture block clears pend[ch]
                    NextValue(ch, Mux(ch == (n_slots - 1), 0, ch + 1)),
                    NextState("SCAN"),
                ).Else(
                    NextValue(widx, widx + 1),
                ),
            ),
        )
