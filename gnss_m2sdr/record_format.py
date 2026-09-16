#
# This file is part of gnss-m2sdr.
#
# Correlator-dump wire format shared by gateware and host driver.
# SPDX-License-Identifier: BSD-2-Clause

"""Fixed-size correlator-dump record streamed over DMA1.

One record = 16 x 64-bit words = 128 bytes, little-endian on the wire:

  word  0 : sample_index                      [63:0]   free-running input-sample counter at dump
  word  1 : integrated_samples [63:32] | channel [31:24] | prn [23:16] | flags [15:8] | seq [7:0]
  word  2 : antenna 0  q_prompt [63:32] | i_prompt [31:0]  (signed)
  word  3 : antenna 0  q_early  [63:32] | i_early  [31:0]  (signed)
  word  4 : antenna 0  q_late   [63:32] | i_late   [31:0]  (signed)
  word  5 : magic [63:32] | code_phase [31:0]
  word  6 : antenna 1  q_prompt [63:32] | i_prompt [31:0]  (signed)
  word  7 : antenna 1  q_early  [63:32] | i_early  [31:0]  (signed)
  word  8 : antenna 1  q_late   [63:32] | i_late   [31:0]  (signed)
  word  9 : reserved [63:24] | num_taps [23:16] | version [15:8] | num_ants [7:0]
  word 10 : code_length [63:32] | code_phase_chip [31:0]
  word 11 : reserved [63:32] | code_step [31:0]
  words 12-15 : reserved (zero)

`seq` is a per-channel record counter (wraps at 256) for host-side loss
detection; `flags` bit 0 = overflow (a dump was dropped before this one).

Versioning
----------
`version` (word 9, bits [15:8]) is RECORD_FORMAT_VERSION: the wire contract the
gateware was built to. Version 1 is the GPS-L1-C/A-only layout, which reads
zero there because it left the byte reserved; version 2 adds the three signal
fields below and `num_taps`, all in words that version 1 left reserved, so a
version-1 host keeps parsing a version-2 record correctly and simply does not
see them. The magic is therefore *not* bumped -- it is the framing anchor
(`find_record_offset`), and a host that cannot even frame the stream cannot read
the version byte that would tell it why. Bump the magic only for a layout change
that moves or resizes an existing field; bump the version for anything else, and
have the host refuse a version it does not know rather than guess.

Signal configuration per record
-------------------------------
A channel's primary code length and code rate are runtime-programmable
(gateware/code_replica.py), so a record has to say which ones produced it:

  * `code_phase_chip` -- the replica's *integer* chip index on the last sample of
    the integration, alongside the fractional phase in `code_phase`. Together
    they are the complete code phase; see `code_phase_chips()`. A dump that ends
    on a code wrap reads `code_length - 1` here, but the host must not assume
    that: it is exactly the "1022" assumption that breaks for every non-1023
    code (and for the sub-period dumps of GNSSReceiver.jl#133).
  * `code_length` -- primary-code chips the channel was configured for.
  * `code_step` -- the code NCO's phase increment per input sample, in
    `code_frac_bits` fixed-point chips (`gnss_capabilities` reports the scale).
    `code_step / 2**code_frac_bits * fs` is the chip rate this record was
    integrated at, which is what the host needs to propagate `code_phase_chips`
    to another sample index after a scheduled rate change.

`num_taps` is how many leading correlator taps this record carries, as
GNSSReceiver's hardware contract requires: 3 (late, prompt, early) for this
gateware, with the field present so a five-tap build (gnss-m2sdr#30) can say 5
on the same wire without another version bump.

Antennas
--------
`num_ants` is how many antenna blocks this record actually carries (1 or
N_ANTS_MAX, and 0 in a record with no correlator payload at all -- see Epoch
strobes below); blocks at or above it are zero and must be ignored. `unpack_record`
clamps the field up to 1 so a caller can read `ants[0]` unconditionally rather
than index a block that is not there. The block is
always reserved so the wire format does not depend on a gateware build option:
the host has to know the record stride before it can find a record at all (the
magic scan below), so a size that varies with the antenna count would have to be
probed. At 1 kHz dumps the unused 48 bytes cost 48 kB/s per channel.

Everything outside the per-antenna blocks is shared: one carrier NCO, one code
NCO, one E/P/L replica set, so one `code_phase`, one `sample_index` and one
`integrated_samples` per channel -- all antennas of a coherent array track the
same signal and only the spatial phase differs (GNSSReceiver.jl#107 keeps
NCOUpdate one-per-channel for the same reason). The host builds Tracking.jl's
`SVector{N,Complex}` accumulators from the blocks, which is what makes
post-correlation beamforming (`EigenBeamformer`, adapting from the per-antenna
prompt covariance `prompt * prompt'`) possible: any combining in gateware would
destroy the spatial information it needs.

N_ANTS_MAX is 2 because the M2SDR's AD9361 is 2T2R -- two coherent RX on one
board, sharing the LO, hence phase-coherent. Larger arrays need
phase-synchronised multi-board setups and are out of scope; note that in 1R1T
mode there is only one antenna and `num_ants` reports 1 even on a 2-antenna
build (the two 64-bit slots of an RX word are then two *consecutive samples* of
the single RX, not two antennas -- see rx_observer.py).

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

`code_phase` (low half of word 5, with its integer chip index in word 10) is
**not** part of that contract -- it is additional
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
payload field is zero, including both antenna blocks, `num_ants`, `num_taps` and
the three signal fields. `version` is the one exception: it describes the wire,
not the payload, so a host that has only ever seen strobes (nothing locked, which
is exactly when strobes matter) can still read the format it is parsing. The host's epoch rule ("close epoch e once something with
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
The record is a power-of-two number of bytes -- more than the payload needs --
because litepcie's kernel driver writes fixed `DMA_BUFFER_SIZE` (8192 B) buffers
and drops *whole* buffers when the ring overruns. 8192 % 48 = 32, so a single
dropped buffer would shift every subsequent record by 32 bytes with no way to
recover; 8192 / 128 = 64 exactly, so every DMA buffer starts on a record
boundary and a drop costs whole records only. The two-antenna payload is 10
words, but neither 80 nor 96 bytes divides 8192 (both leave 32), so the record
is padded to 128; that also divides any power-of-two buffer size a future
per-DMA length would pick. Fewer, larger records per buffer additionally lowers
the buffer-completion latency of docs/dma1_record_path.md (64 records/buffer =
16 ms at 4 channels).

The upper half of word 5 carries `RECORD_MAGIC` ("GNSS" in wire order) as a
sync anchor: a host that attaches to an already-running stream, or that sees a
torn buffer, resynchronises with `find_record_offset()` / `parse_records()`
instead of trusting the stream to be contiguous. The stream endpoint's
`first`/`last` are no help here -- litepcie's DMA writer ignores them.
"""

import struct

# litepcie kernel driver, software/kernel/config.h.
DMA_BUFFER_SIZE = 8192

RECORD_WORDS = 16
RECORD_BYTES = RECORD_WORDS * 8
RECORDS_PER_DMA_BUFFER = DMA_BUFFER_SIZE // RECORD_BYTES

# "GNSS" as it reads in a little-endian hexdump. This anchors the *framing*, so
# it changes only when a field moves or changes size; a compatible extension
# bumps RECORD_FORMAT_VERSION instead (see the Versioning section above).
RECORD_MAGIC  = 0x53534E47
MAGIC_WORD    = 5
MAGIC_SHIFT   = 32
MAGIC_OFFSET  = MAGIC_WORD * 8 + MAGIC_SHIFT // 8   # byte offset within a record

# Wire-format revision reported in every record (word 9, bits [15:8]).
#   1 : GPS L1 C/A only; words 9[15:8] upward reserved (reads 0).
#   2 : + version / num_taps, code_phase_chip, code_length, code_step.
RECORD_FORMAT_VERSION = 2

# CSR-layout revision reported by the gnss_version CSR. Bumped together with
# any change to the register set the host driver addresses by name.
CSR_LAYOUT_VERSION = 2

# Correlator taps this gateware produces per record, latest first on the host
# (late, prompt, early). GNSSReceiver's hardware contract wants it per record so
# one stream can carry several layouts; a five-tap build (gnss-m2sdr#30) reports
# 5 in the same field.
NUM_TAPS = 3

# Replica modulations the gateware can synthesise, as the bitmask reported by
# the gnss_modulations CSR. They name GNSSReceiver's
# `HardwareCorrelatorCapabilities.modulations` symbols, so the adapter maps a
# set bit straight onto one: bit 0 -> :LOC. The BOC/CBOC/TMBOC bits are
# *allocated*, not implemented -- gnss-m2sdr#30 sets them when the replicas
# exist. A capability that is only allocated must read back 0, because an
# over-declared capability is a channel that arms and never locks.
MOD_LOC   = 1 << 0                       # plain +/-1 BPSK code (:LOC)
MOD_BOC11 = 1 << 1                       # reserved: :BOCcos / BOC(1,1)
MOD_CBOC  = 1 << 2                       # reserved: :CBOC
MOD_TMBOC = 1 << 3                       # reserved: :TMBOC
MODULATIONS = MOD_LOC

# Longest secondary (overlay) code the gateware wipes off itself. 1 means
# "primary code only", which is what GNSSReceiver's contract asks for today
# (`requested_secondary_code_mode` is always :primary_only); overlay removal is
# GNSSReceiver.jl#132.
MAX_SECONDARY_CODE_LENGTH = 1

# Furthest an E/P/L tap can sit from the prompt replica, in chips: the taps
# address chip index +/- 1, so a tap offset of a whole chip is the hard limit.
# GNSSReceiver calls this `max_tap_offset_chips`.
MAX_TAP_OFFSET_CHIPS = 1.0

# Antenna n's E/P/L block starts at ANT_PROMPT_WORD[n] (prompt, early, late).
# Antenna 0 keeps the words it had in the single-antenna layout, so antenna 1
# lands after the magic word rather than adjacent to antenna 0.
N_ANTS_MAX      = 2                  # AD9361 is 2T2R -> 2 coherent RX per board
ANT_PROMPT_WORD = (2, 6)
ANT_BLOCK_WORDS = 3

# Word 9: num_ants [7:0] | version [15:8] | num_taps [23:16].
NANTS_WORD      = 9
VERSION_SHIFT   = 8
NUM_TAPS_SHIFT  = 16
# Word 10: code_phase_chip [31:0] | code_length [63:32].
CODE_WORD       = 10
CODE_LENGTH_SHIFT = 32
# Word 11: code_step [31:0] | reserved.
CODE_STEP_WORD  = 11

assert len(ANT_PROMPT_WORD) == N_ANTS_MAX
assert DMA_BUFFER_SIZE % RECORD_BYTES == 0, "record must divide the DMA buffer"

FLAG_OVERFLOW      = 1 << 0
FLAG_EPOCH_STROBE  = 1 << 1

# Reserved `channel` id for the periodic timebase marker. 0xFF cannot collide
# with a real channel: the round-robin serializer only reaches n_channels.
STROBE_CHANNEL = 0xFF

ACC_KEYS = ("i_early", "q_early", "i_prompt", "q_prompt", "i_late", "q_late")
# The gateware's short names for the same six accumulators, in the same order
# (TrackingChannel.acc[n] / ChannelDumpPort.acc[n] are keyed by these).
ACC_SIGNALS = ("ie", "qe", "ip", "qp", "il", "ql")


def pack_record(sample_index, integrated_samples, channel, prn, seq, flags,
                i_early, q_early, i_prompt, q_prompt, i_late, q_late, code_phase,
                ants=(), num_ants=None, code_phase_chip=0, code_length=0,
                code_step=0, num_taps=NUM_TAPS, version=RECORD_FORMAT_VERSION):
    """Build the 16 little-endian 64-bit words for one record (for tests).

    The flat accumulator arguments are antenna 0; `ants` holds the additional
    antennas (dicts keyed by ACC_KEYS), so a single-antenna caller is unchanged.
    `num_ants` defaults to how many blocks were given; pass 0 for a record that
    carries no correlator payload at all, which is what an epoch strobe is.

    `code_phase_chip` / `code_length` / `code_step` are the version-2 signal
    fields; leaving them at 0 and passing `version=1` produces the version-1
    layout byte for byte, which is what the compatibility tests compare against.
    """
    def u32(x): return x & 0xFFFFFFFF
    blocks = [dict(i_early=i_early, q_early=q_early, i_prompt=i_prompt,
                   q_prompt=q_prompt, i_late=i_late, q_late=q_late)] + list(ants)
    assert len(blocks) <= N_ANTS_MAX, f"at most {N_ANTS_MAX} antennas"
    if num_ants is None:
        num_ants = len(blocks)
    assert 0 <= num_ants <= N_ANTS_MAX, f"0..{N_ANTS_MAX} antennas"

    words = [0] * RECORD_WORDS
    words[0] = sample_index & ((1 << 64) - 1)
    words[1] = ((integrated_samples & 0xFFFFFFFF) << 32) | ((channel & 0xFF) << 24) | \
               ((prn & 0xFF) << 16) | ((flags & 0xFF) << 8) | (seq & 0xFF)
    words[MAGIC_WORD] = (RECORD_MAGIC << MAGIC_SHIFT) | u32(code_phase)
    words[NANTS_WORD] = ((num_ants & 0xFF)
                         | ((version & 0xFF) << VERSION_SHIFT)
                         | ((num_taps & 0xFF) << NUM_TAPS_SHIFT))
    words[CODE_WORD]      = (u32(code_length) << CODE_LENGTH_SHIFT) | u32(code_phase_chip)
    words[CODE_STEP_WORD] = u32(code_step)
    for n, b in enumerate(blocks[:num_ants]):
        base = ANT_PROMPT_WORD[n]
        words[base + 0] = (u32(b["q_prompt"]) << 32) | u32(b["i_prompt"])
        words[base + 1] = (u32(b["q_early"])  << 32) | u32(b["i_early"])
        words[base + 2] = (u32(b["q_late"])   << 32) | u32(b["i_late"])
    return words


def _s32(x):
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def unpack_ant_block(words, n):
    """Antenna n's six accumulators, as a dict keyed by ACC_KEYS."""
    base = ANT_PROMPT_WORD[n]
    wp, we, wl = words[base:base + ANT_BLOCK_WORDS]
    return dict(
        i_prompt = _s32(wp), q_prompt = _s32(wp >> 32),
        i_early  = _s32(we), q_early  = _s32(we >> 32),
        i_late   = _s32(wl), q_late   = _s32(wl >> 32),
    )


def unpack_record(words):
    """Inverse of pack_record: dict of fields from 16 words (host-side).

    `ants` is the list of the `num_ants` valid per-antenna accumulator blocks
    (the host's `SVector{N,Complex}`); antenna 0's fields are also spliced in
    flat, so single-antenna callers need no change.
    """
    assert len(words) == RECORD_WORDS
    w0, w1 = words[0], words[1]
    w5     = words[MAGIC_WORD]
    # Clamped, so a record from a future/garbled build cannot make this index
    # past the reserved blocks; every record carries at least antenna 0.
    w9     = words[NANTS_WORD]
    w10    = words[CODE_WORD]
    num_ants = min(max(w9 & 0xFF, 1), N_ANTS_MAX)
    ants = [unpack_ant_block(words, n) for n in range(num_ants)]
    return dict(
        sample_index       = w0,
        integrated_samples = (w1 >> 32) & 0xFFFFFFFF,
        channel            = (w1 >> 24) & 0xFF,
        prn                = (w1 >> 16) & 0xFF,
        flags              = (w1 >> 8) & 0xFF,
        seq                = w1 & 0xFF,
        code_phase = w5 & 0xFFFFFFFF,
        magic      = (w5 >> MAGIC_SHIFT) & 0xFFFFFFFF,
        num_ants   = num_ants,
        # Version-2 fields. A version-1 record reads 0 in all of them, which is
        # why `version` has to be checked before `code_length` is believed.
        version         = (w9 >> VERSION_SHIFT) & 0xFF,
        num_taps        = (w9 >> NUM_TAPS_SHIFT) & 0xFF,
        code_phase_chip = w10 & 0xFFFFFFFF,
        code_length     = (w10 >> CODE_LENGTH_SHIFT) & 0xFFFFFFFF,
        code_step       = words[CODE_STEP_WORD] & 0xFFFFFFFF,
        ants       = ants,
        **ants[0],
    )


def code_phase_chips(rec, frac_bits):
    """Complete code phase of a dump, in chips, as a float.

    The replica's phase on the *last* sample of the integration: the integer
    chip index the record reports plus the fractional chip phase, with no
    assumption about where in the code the dump landed. `frac_bits` is the
    gateware's code_frac_bits (`gnss_capabilities`).

    A version-1 record carries no chip index, so this would silently read 0
    there; it raises instead, because "chip 0" is a plausible-looking answer.
    """
    if rec.get("version", 0) < 2:
        raise ValueError(
            "record format version %r carries no code_phase_chip; the chip index "
            "cannot be reconstructed (assuming code_length-1 is only valid for a "
            "dump that ends exactly on a code wrap)" % rec.get("version", 0))
    return rec["code_phase_chip"] + rec["code_phase"] / float(1 << frac_bits)


def code_chip_rate(rec, frac_bits, sampling_freq):
    """Chip rate (Hz) the dump was integrated at, from its `code_step`."""
    if rec.get("version", 0) < 2:
        raise ValueError("record format version %r carries no code_step"
                         % rec.get("version", 0))
    return rec["code_step"] / float(1 << frac_bits) * sampling_freq


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
