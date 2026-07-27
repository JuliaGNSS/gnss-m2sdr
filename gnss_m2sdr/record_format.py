#
# This file is part of gnss-m2sdr.
#
# Correlator-dump wire format shared by gateware and host driver.
# SPDX-License-Identifier: BSD-2-Clause

"""Fixed-size correlator-dump record streamed over DMA1.

One record = 8 x 64-bit words = 64 bytes, little-endian on the wire:

  word 0 : sample_index                       [63:0]   free-running input-sample counter at dump
  word 1 : integrated_samples [63:32] | channel [31:24] | prn [23:16] | flags [15:8] | seq [7:0]
  word 2 : q_prompt [63:32] | i_prompt [31:0]  (signed)
  word 3 : q_early  [63:32] | i_early  [31:0]  (signed)
  word 4 : q_late   [63:32] | i_late   [31:0]  (signed)
  word 5 : magic [63:32] | code_phase [31:0]
  word 6 : reserved (zero)
  word 7 : reserved (zero)

`seq` is a per-channel record counter (wraps at 256) for host-side loss
detection; `flags` bit 0 = overflow (a dump was dropped before this one).

These map onto Tracking.jl's CorrelatorOutput. Mind the accumulator order:
Tracking.jl's `EarlyPromptLateCorrelator.accumulators` runs *latest first*, not
early first -- `get_prompt_index` is `div(3-1,2)+1` = 2, the late accumulator is
`prompt_index - 1` = 1 and the early one is `prompt_index + 1` = 3 (matching
`get_correlator_sample_shifts`, whose shifts are "ordered from latest to
earliest replica"). So the host glue must build

    CorrelatorOutput(EarlyPromptLateCorrelator(SVector(late, prompt, early),
                                               spacing),
                     integrated_samples, sample_index)

i.e. word 4, then word 2, then word 3 -- the reverse of the wire order. Passing
`SVector(early, prompt, late)` swaps E and L, which inverts the sign of the DLL
discriminator `(2-d)/2 * (E-L)/(E+L)` and drives the code phase away from lock;
the symptom is "tracking never converges" rather than an obvious error.

`code_phase` (low half of word 5) is **not** part of that contract -- it is additional
device-side metadata that Tracking.jl does not currently consume. As of
Tracking.jl v4.1.1 (with #207 merged) `CorrelatorOutput` has exactly the three
fields above and no `code_phase` keyword constructor, even though #207's
description advertises one. The field stays in the record because the host
needs it for acquisition handover and downstream vector tracking; it just has
to be carried out of band rather than passed to the constructor.

`sample_index` is the **0-based** index of the last sample included in the
integration, on the bank's single free-running counter (gnss_sample_count CSR):
shared by every channel, never reset by a channel restart, so records from
channels handed over at different times are directly comparable. Tracking.jl
wants the 1-based index relative to the current chunk origin, so the host maps

    sample_index_julia = sample_index - chunk_origin + 1

where chunk_origin is the counter value at the first sample of the chunk. The
`+1` is deliberate, not an off-by-one. The companion invariant holds on both
sides: first_sample = sample_index - integrated_samples + 1.

Epoch strobes
-------------
`channel == STROBE_CHANNEL` (0xFF) with `flags` bit 1 (`FLAG_EPOCH_STROBE`) set
marks a **timebase record**, not a correlator dump: the recorder emits one every
`epoch_period` input samples (`gnss_epoch_period` CSR, 0 = off), carrying only
`sample_index` on the same free-running counter as the dumps -- every other
payload field is zero. The host's epoch rule ("close epoch e once something with
`sample_index >= (e+1)*delta` arrives", GNSSReceiver.jl#107) then has a clock
that does not depend on a satellite being locked: without it a receiver with
nothing acquired, or one that has just lost lock on every channel, stalls the
loop indefinitely, and with only one channel dumping the boundary jitters with
that satellite's code phase. Set `epoch_period` to the host's delta so a strobe
lands exactly on each boundary.

The strobe is a recorder slot like a channel, so it inherits the whole
lost-record story unchanged: `FLAG_OVERFLOW` on a marker means a previous marker
was dropped (period shorter than a record takes to serialize), bit `n_channels`
of `gnss_overflow` is its sticky status, `gnss_droppedstrobe` counts the losses,
and the same `gnss_overflow_clear` bit clears both.

Host glue must skip these when building CorrelatorOutputs -- use
`is_epoch_strobe()` -- and use them only to advance the epoch clock.

Framing
-------
The record is 64 bytes -- not the 48 bytes the payload needs -- because
litepcie's kernel driver writes fixed `DMA_BUFFER_SIZE` (8192 B) buffers and
drops *whole* buffers when the ring overruns. 8192 % 48 = 32, so a single
dropped buffer would shift every subsequent record by 32 bytes with no way to
recover; 8192 / 64 = 128 exactly, so every DMA buffer starts on a record
boundary and a drop costs whole records only. 64 divides any power-of-two
buffer size, so a future per-DMA buffer length keeps working.

The upper half of word 5 carries `RECORD_MAGIC` ("GNSS" in wire order) as a
sync anchor: a host that attaches to an already-running stream, or that sees a
torn buffer, resynchronises with `find_record_offset()` / `parse_records()`
instead of trusting the stream to be contiguous. The stream endpoint's
`first`/`last` are no help here -- litepcie's DMA writer ignores them.
"""

import struct

# litepcie kernel driver, software/kernel/config.h.
DMA_BUFFER_SIZE = 8192

RECORD_WORDS = 8
RECORD_BYTES = RECORD_WORDS * 8
RECORDS_PER_DMA_BUFFER = DMA_BUFFER_SIZE // RECORD_BYTES

# "GNSS" as it reads in a little-endian hexdump; bump on a layout change.
RECORD_MAGIC  = 0x53534E47
MAGIC_WORD    = 5
MAGIC_SHIFT   = 32
MAGIC_OFFSET  = MAGIC_WORD * 8 + MAGIC_SHIFT // 8   # byte offset within a record

assert DMA_BUFFER_SIZE % RECORD_BYTES == 0, "record must divide the DMA buffer"

FLAG_OVERFLOW      = 1 << 0
FLAG_EPOCH_STROBE  = 1 << 1

# Reserved `channel` id for the periodic timebase marker. 0xFF cannot collide
# with a real channel: the round-robin serializer only reaches n_channels.
STROBE_CHANNEL = 0xFF


def pack_record(sample_index, integrated_samples, channel, prn, seq, flags,
                i_early, q_early, i_prompt, q_prompt, i_late, q_late, code_phase):
    """Build the 8 little-endian 64-bit words for one record (for tests)."""
    def u32(x): return x & 0xFFFFFFFF
    w0 = sample_index & ((1 << 64) - 1)
    w1 = ((integrated_samples & 0xFFFFFFFF) << 32) | ((channel & 0xFF) << 24) | \
         ((prn & 0xFF) << 16) | ((flags & 0xFF) << 8) | (seq & 0xFF)
    w2 = (u32(q_prompt) << 32) | u32(i_prompt)
    w3 = (u32(q_early)  << 32) | u32(i_early)
    w4 = (u32(q_late)   << 32) | u32(i_late)
    w5 = (RECORD_MAGIC << MAGIC_SHIFT) | u32(code_phase)
    return [w0, w1, w2, w3, w4, w5, 0, 0]


def _s32(x):
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def unpack_record(words):
    """Inverse of pack_record: dict of fields from 8 words (host-side)."""
    assert len(words) == RECORD_WORDS
    w0, w1, w2, w3, w4, w5 = words[:6]
    return dict(
        sample_index       = w0,
        integrated_samples = (w1 >> 32) & 0xFFFFFFFF,
        channel            = (w1 >> 24) & 0xFF,
        prn                = (w1 >> 16) & 0xFF,
        flags              = (w1 >> 8) & 0xFF,
        seq                = w1 & 0xFF,
        i_prompt = _s32(w2), q_prompt = _s32(w2 >> 32),
        i_early  = _s32(w3), q_early  = _s32(w3 >> 32),
        i_late   = _s32(w4), q_late   = _s32(w4 >> 32),
        code_phase = w5 & 0xFFFFFFFF,
        magic      = (w5 >> MAGIC_SHIFT) & 0xFFFFFFFF,
    )


def is_epoch_strobe(rec):
    """True for a timebase marker (no correlator payload), false for a dump."""
    return bool(rec["flags"] & FLAG_EPOCH_STROBE) and rec["channel"] == STROBE_CHANNEL


def has_magic_at(data, offset):
    """True if a record starting at byte `offset` of `data` carries the magic."""
    o = offset + MAGIC_OFFSET
    if offset < 0 or offset + RECORD_BYTES > len(data):
        return False
    return struct.unpack_from("<I", data, o)[0] == RECORD_MAGIC


def find_record_offset(data, confirm=2):
    """Byte offset of the first whole record in `data`, or None.

    Candidates come from scanning for the magic itself rather than from the
    RECORD_BYTES possible phases, so a stream that is torn mid-buffer (not
    just shifted) still resynchronises. `confirm` magics one record apart are
    required (where that many records are available) so payload bytes that
    happen to spell the magic cannot lock the host onto a wrong offset.
    """
    magic = struct.pack("<I", RECORD_MAGIC)
    pos   = data.find(magic)
    while pos != -1:
        off = pos - MAGIC_OFFSET
        if off >= 0 and off + RECORD_BYTES <= len(data):
            avail = (len(data) - off) // RECORD_BYTES
            if all(has_magic_at(data, off + i * RECORD_BYTES)
                   for i in range(min(confirm, avail))):
                return off
        pos = data.find(magic, pos + 1)
    return None


def parse_records(data, offset=None):
    """Unpack every record in a raw DMA byte stream, resynchronising on loss.

    `offset` defaults to 0 when `data` already starts on a record boundary (the
    normal case: DMA buffers are whole numbers of records) and to the first
    boundary found otherwise. Bytes that do not start a valid record -- a
    mid-record attach, a torn buffer -- are skipped rather than misparsed.
    """
    if offset is None:
        offset = 0 if has_magic_at(data, 0) else find_record_offset(data)
        if offset is None:
            return []
    recs = []
    while offset + RECORD_BYTES <= len(data):
        if has_magic_at(data, offset):
            words = struct.unpack_from("<%dQ" % RECORD_WORDS, data, offset)
            recs.append(unpack_record(list(words)))
            offset += RECORD_BYTES
        else:
            skip = find_record_offset(data[offset + 1:])
            if skip is None:
                break
            offset += 1 + skip
    return recs
