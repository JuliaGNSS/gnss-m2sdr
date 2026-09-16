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
  word 12 : antenna 0  q_very_early [63:32] | i_very_early [31:0]  (5-tap only)
  word 13 : antenna 0  q_very_late  [63:32] | i_very_late  [31:0]  (5-tap only)
  word 14 : antenna 1  q_very_early [63:32] | i_very_early [31:0]  (5-tap only)
  word 15 : antenna 1  q_very_late  [63:32] | i_very_late  [31:0]  (5-tap only)

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

Taps
----
`num_taps` is how many correlator taps this record carries, as GNSSReceiver's
hardware contract requires it per record: 3 (late, prompt, early) or 5 (very
late, late, prompt, early, very early). It is **per channel, not per build** --
one bank can run GPS L1 C/A on three taps and Galileo E1 on five at the same
time, and the field is what tells the two apart in one stream.

  * `num_taps == 3` -- words 2..4 (and 6..8) carry prompt/early/late as they
    always have, and words 12..15 read zero.
  * `num_taps == 5` -- the same words carry the same three taps, and words
    12..15 add very-early and very-late for antenna 0 and antenna 1.

The version is **not** bumped for this. Word 9's `num_taps` was allocated in
version 2 for exactly this purpose, and a version-2 host already has to check it:
the contract drops a record whose tap count is not the tracked correlator's
rather than reshaping it, so a three-tap host never reads words 12..15 and a
three-tap record still reads zero there. The magic, the stride and every
existing field stay put.

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

A five-tap record is the same rule with two more slots:

    SVector(very_late, late, prompt, early, very_early)

i.e. word 13, word 4, word 2, word 3, word 12 for antenna 0. `tap_accumulators()`
below returns exactly that order for a record of either width, so a host does not
have to rebuild the mapping (and cannot rebuild it early-first by accident).

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
#   1 : GPS L1 C/A bring-up register set.
#   2 : + code_length / code_load / capability + signal-capability registers.
#   3 : + per-tap offsets (replacing the single symmetric `spacing`), the
#       per-channel replica shape (`replica`) and the subcarrier table write
#       port. A driver written for v2 must refuse v3 rather than address the
#       old names: `spacing` is gone, and a channel left at its reset offsets
#       would correlate at 0.5 chips whatever the host meant to program.
CSR_LAYOUT_VERSION = 3

# Correlator tap layouts. GNSSReceiver's hardware contract wants the count per
# record, so one stream can carry both: a GPS L1 C/A channel dumps 3 (late,
# prompt, early) next to a Galileo E1 channel dumping 5 (very late, late,
# prompt, early, very early). `NUM_TAPS` is the layout a channel resets to, not
# a property of the build -- see `tap_layouts_mask` for what a build can do.
TAPS_EPL  = 3
TAPS_VEPL = 5
TAP_LAYOUTS = (TAPS_EPL, TAPS_VEPL)
NUM_TAPS = TAPS_EPL

# Tap names, earliest replica first -- the order the gateware's accumulators and
# the per-tap CSRs are in. The wire and the host both want them latest first;
# `tap_accumulators()` does that reversal once, here, rather than in every
# caller.
TAP_NAMES = ("very_early", "early", "prompt", "late", "very_late")
TAP_SHORT = ("ve",         "e",     "p",      "l",    "vl")
# Which of those a layout has: 3 taps is the middle three, 5 taps is all of them.
_TAP_SLICE = {TAPS_EPL: slice(1, 4), TAPS_VEPL: slice(0, 5)}


def tap_names(num_taps=NUM_TAPS):
    """Tap names of a layout, earliest first."""
    try:
        return TAP_NAMES[_TAP_SLICE[num_taps]]
    except KeyError:
        raise ValueError(
            f"num_taps must be one of {TAP_LAYOUTS}, got {num_taps!r}") from None


def tap_short_names(num_taps=NUM_TAPS):
    """Gateware short tap names of a layout, earliest first."""
    tap_names(num_taps)          # validates
    return TAP_SHORT[_TAP_SLICE[num_taps]]


def acc_keys(num_taps=NUM_TAPS):
    """Host-side accumulator field names of a layout (i_early, q_early, ...)."""
    return tuple(f"{iq}_{t}" for t in tap_names(num_taps) for iq in ("i", "q"))


def acc_signals(num_taps=NUM_TAPS):
    """Gateware accumulator signal names of a layout (ie, qe, ip, ...)."""
    return tuple(f"{iq}{t}" for t in tap_short_names(num_taps) for iq in ("i", "q"))


def tap_layouts_mask(num_taps):
    """Capability bitmask of the layouts a build with `num_taps` taps can emit.

    Bit i means 2*i + 3 taps. A five-tap build serves three-tap channels too --
    the extra accumulators are simply not reported -- so it declares both, which
    is what lets one bank mix GPS L1 C/A with Galileo E1.
    """
    tap_names(num_taps)          # validates
    return sum(1 << i for i, n in enumerate(TAP_LAYOUTS) if n <= num_taps)

# Replica modulations the gateware can synthesise, as the bitmask reported by
# the gnss_signal_caps.modulations CSR. They name GNSSReceiver's
# `HardwareCorrelatorCapabilities.modulations` symbols -- which are
# `nameof(typeof(get_modulation(signal)))` on the GNSSSignals type -- so the
# adapter maps a set bit straight onto one.
#
# Bits 0..3 were allocated (reading 0) by record format v2. Bit 1 was reserved
# under the name `:BOCcos`; that name is kept, and `:BOCsin` gets a *new* bit
# rather than taking over bit 1, because every L1 BOC signal GNSSSignals exposes
# is sine-phased (`GalileoE1B_BOC11`, `GPSL1C_D`, `BeiDouB1C_D/P` all report
# `BOCsin(1,1)`). Redefining bit 1 would have made a host that knows the v2
# mapping declare :BOCcos for a build that synthesises :BOCsin -- an
# over-declared capability, which is the failure this file exists to avoid. An
# older host simply does not see bit 4 and refuses the signal instead.
MOD_LOC    = 1 << 0                      # plain +/-1 BPSK code (:LOC)
MOD_BOCCOS = 1 << 1                      # :BOCcos -- cosine-phased BOC(m,1)
MOD_CBOC   = 1 << 2                      # :CBOC   -- amplitude-bearing composite
MOD_TMBOC  = 1 << 3                      # :TMBOC  -- time-multiplexed BOC
MOD_BOCSIN = 1 << 4                      # :BOCsin -- sine-phased BOC(m,1)

# Sub-chips per chip each modulation family needs at its lowest order, i.e. the
# `max_subchips` a build must have before it may declare that family. The host
# still has to check the *specific* order it wants against the reported
# `max_subchips`: BOCsin(1,1) needs 2 sub-chips and BOCsin(6,1) needs 12, and
# one bit cannot say both.
MODULATION_MIN_SUBCHIPS = (
    (MOD_LOC,     1),
    (MOD_BOCSIN,  2),    # BOCsin(1,1)
    (MOD_BOCCOS,  4),    # BOCcos(1,1), on the quarter-sub-chip grid
    (MOD_CBOC,   12),    # CBOC(6,1,1/11)  -- Galileo E1B/E1C
    (MOD_TMBOC,  12),    # TMBOC(6,1,4/33) -- GPS L1C-P
)


def modulations_mask(max_subchips):
    """Modulations a build with `max_subchips` sub-chips per chip can synthesise.

    Declared from what the subcarrier LUT can actually hold, never from what the
    field has a bit for: a build with `max_subchips = 1` has no sub-chip grid at
    all and declares :LOC alone, exactly as record format v2 did.
    """
    return sum(bit for bit, need in MODULATION_MIN_SUBCHIPS if max_subchips >= need)


# The LOC-only build's mask, i.e. what record format v2's gateware declared. The
# bank derives its own from `max_subchips`; this is here for a caller that wants
# to name the baseline.
MODULATIONS = modulations_mask(1)

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
# ... and its very-early/very-late pair, in the tail words version 2 reserved.
# Two antennas x two extra taps is exactly the four words that were left, which
# is why a five-tap record still fits the 128-byte stride the DMA framing needs.
ANT_VERY_WORD   = (12, 14)
ANT_VERY_WORDS  = 2

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
assert len(ANT_VERY_WORD) == N_ANTS_MAX
assert max(ANT_VERY_WORD) + ANT_VERY_WORDS <= RECORD_WORDS, "record is full"
assert DMA_BUFFER_SIZE % RECORD_BYTES == 0, "record must divide the DMA buffer"

FLAG_OVERFLOW      = 1 << 0
FLAG_EPOCH_STROBE  = 1 << 1

# Reserved `channel` id for the periodic timebase marker. 0xFF cannot collide
# with a real channel: the round-robin serializer only reaches n_channels.
STROBE_CHANNEL = 0xFF

ACC_KEYS = acc_keys(TAPS_EPL)
# The gateware's short names for the same six accumulators, in the same order
# (TrackingChannel.acc[n] / ChannelDumpPort.acc[n] are keyed by these).
ACC_SIGNALS = acc_signals(TAPS_EPL)
# The five-tap versions, for a build that has the very-early/very-late taps.
ACC_KEYS_VEPL    = acc_keys(TAPS_VEPL)
ACC_SIGNALS_VEPL = acc_signals(TAPS_VEPL)


def pack_record(sample_index, integrated_samples, channel, prn, seq, flags,
                i_early, q_early, i_prompt, q_prompt, i_late, q_late, code_phase,
                ants=(), num_ants=None, code_phase_chip=0, code_length=0,
                code_step=0, num_taps=NUM_TAPS, version=RECORD_FORMAT_VERSION,
                i_very_early=0, q_very_early=0, i_very_late=0, q_very_late=0):
    """Build the 16 little-endian 64-bit words for one record (for tests).

    The flat accumulator arguments are antenna 0; `ants` holds the additional
    antennas (dicts keyed by ACC_KEYS, plus the very-early/very-late keys for a
    five-tap record), so a single-antenna caller is unchanged.
    `num_ants` defaults to how many blocks were given; pass 0 for a record that
    carries no correlator payload at all, which is what an epoch strobe is.

    `code_phase_chip` / `code_length` / `code_step` are the version-2 signal
    fields; leaving them at 0 and passing `version=1` produces the version-1
    layout byte for byte, which is what the compatibility tests compare against.

    `num_taps` selects the tap layout: 3 leaves the very-early/very-late words
    zero whatever was passed for them, because a three-tap record must read zero
    there (a host that trusted a stale value would hand `dll_disc` two
    accumulators that never saw a replica).
    """
    def u32(x): return x & 0xFFFFFFFF
    blocks = [dict(i_early=i_early, q_early=q_early, i_prompt=i_prompt,
                   q_prompt=q_prompt, i_late=i_late, q_late=q_late,
                   i_very_early=i_very_early, q_very_early=q_very_early,
                   i_very_late=i_very_late, q_very_late=q_very_late)] + list(ants)
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
        if num_taps >= TAPS_VEPL:
            very = ANT_VERY_WORD[n]
            words[very + 0] = (u32(b.get("q_very_early", 0)) << 32) | u32(b.get("i_very_early", 0))
            words[very + 1] = (u32(b.get("q_very_late", 0))  << 32) | u32(b.get("i_very_late", 0))
    return words


def _s32(x):
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def unpack_ant_block(words, n, num_taps=NUM_TAPS):
    """Antenna n's accumulators, as a dict keyed by ACC_KEYS.

    A five-tap record additionally carries the very-early/very-late pair from
    the tail words. They are reported as `None` -- not 0 -- for a three-tap
    record, because a zero accumulator is a value a correlator can legitimately
    produce and "this record has no such tap" is not.
    """
    base = ANT_PROMPT_WORD[n]
    wp, we, wl = words[base:base + ANT_BLOCK_WORDS]
    block = dict(
        i_prompt = _s32(wp), q_prompt = _s32(wp >> 32),
        i_early  = _s32(we), q_early  = _s32(we >> 32),
        i_late   = _s32(wl), q_late   = _s32(wl >> 32),
    )
    if num_taps >= TAPS_VEPL:
        very = ANT_VERY_WORD[n]
        wve, wvl = words[very:very + ANT_VERY_WORDS]
        block.update(
            i_very_early = _s32(wve), q_very_early = _s32(wve >> 32),
            i_very_late  = _s32(wvl), q_very_late  = _s32(wvl >> 32),
        )
    else:
        block.update(i_very_early=None, q_very_early=None,
                     i_very_late=None,  q_very_late=None)
    return block


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
    num_taps = (w9 >> NUM_TAPS_SHIFT) & 0xFF
    ants = [unpack_ant_block(words, n, num_taps) for n in range(num_ants)]
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
        num_taps        = num_taps,
        code_phase_chip = w10 & 0xFFFFFFFF,
        code_length     = (w10 >> CODE_LENGTH_SHIFT) & 0xFFFFFFFF,
        code_step       = words[CODE_STEP_WORD] & 0xFFFFFFFF,
        ants       = ants,
        **ants[0],
    )


def tap_accumulators(rec, antenna=0):
    """Antenna `antenna`'s accumulators as (I, Q) pairs, **latest first**.

    The order Tracking.jl's correlators want: `[late, prompt, early]` for a
    three-tap record and `[very late, late, prompt, early, very early]` for a
    five-tap one. Reading the wire order into `SVector(early, prompt, late)`
    instead inverts the DLL discriminator, so this reversal is done once, here.

    Raises on a record whose `num_taps` is not a layout this format defines: a
    record that says 4 taps is a record whose words cannot be attributed, and
    padding or truncating it hands the loop filters accumulators that never saw
    a replica.
    """
    n = rec["num_taps"]
    if n not in TAP_LAYOUTS:
        raise ValueError(
            f"record reports num_taps={n!r}, not one of {TAP_LAYOUTS}; its "
            f"accumulators cannot be attributed to taps")
    block = rec["ants"][antenna]
    return [(block[f"i_{t}"], block[f"q_{t}"]) for t in reversed(tap_names(n))]


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
