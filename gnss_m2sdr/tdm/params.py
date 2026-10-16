#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Fixed formats of the TDM correlator: number formats, the record layout, the
command words and the capability register. Shared by the gateware, the
bit-exact reference model and the host tests, so the three cannot drift apart.

Number formats
--------------
- Carrier phase: unsigned 32-bit, cycles x 2^32. Carrier frequency word:
  signed 32-bit, cycles/sample x 2^32.
- Code phase: integer chip (``CHIP_BITS``) + unsigned 32-bit fraction. Code
  frequency word: unsigned 32-bit, chips/sample x 2^32 (so the code rate must
  stay below one chip per sample).
- Samples: 12-bit signed I and Q (the AD9361's resolution, sign-extended to 16
  bits in the RX word).
- Carrier table: ``CARRIER_LUT_BITS``-bit phase index, 8-bit signed cos/sin
  with amplitude ``CARRIER_AMPLITUDE``.
- Accumulators: signed ``ACC_BITS``, saturating.

Taps
----
The code NCO runs on the *earliest* tap ("lead" samples ahead of the prompt).
Each sample the replica chip of that earliest phase is pushed into a history
of ``HISTORY`` entries, newest at index 0, together with a flag saying that
the earliest phase wrapped after this sample. A tap with sample shift ``s``
(positive = early, Tracking.jl's convention) reads history entry
``d = lead - s``; the prompt's code period ends where entry ``lead`` carries
the wrap flag. No tap needs its own code read or multiplier.
"""

import math

# Number formats ------------------------------------------------------------------------------------

SAMPLE_BITS       = 12
CARRIER_LUT_BITS  = 6
CARRIER_AMPLITUDE = 127
CHIP_BITS         = 20
FRAC_BITS         = 32
PHASE_BITS        = 32
FREQ_BITS         = 32
ACC_BITS          = 32
INTEG_BITS        = 24
HISTORY           = 32          # replica history entries; tap offsets d in [0, HISTORY)
HISTORY_BITS      = 5
MAX_TAPS          = 5
ASSIGN_BITS       = 16
SAMPLE_INDEX_BITS = 48
CODE_WORD_BITS    = 32          # chips per code-memory word

# Record layout (8 x 64-bit little-endian words = 64 bytes) -------------------------------------------
#
# w0: [15:0] magic, [19:16] version, [23:20] kind, [31:24] channel, [34:32] num_taps,
#     [35] antenna, [41:36] flags, [47:42] reserved, [63:48] assignment id
# w1: [47:0] sample index (one past the last sample integrated), [63:48] reserved
# w2: [23:0] integrated samples, [43:24] code chip, [63:44] code fraction [31:12]
#     (the *earliest* tap's code phase after the last sample)
# w3..w7: tap k (in the order the channel was armed with): [31:0] I, [63:32] Q

RECORD_WORDS   = 8
RECORD_BYTES   = 64
RECORD_MAGIC   = 0x4E47        # "GN" in wire order
RECORD_VERSION = 1

KIND_PAD    = 0                # an all-zero block: buffer padding, skipped by the host
KIND_DUMP   = 1
KIND_STROBE = 2

FLAG_SATURATED       = 1 << 0  # an accumulator hit the rail during this record
FLAG_SAMPLE_OVERFLOW = 1 << 1  # the sample FIFO overflowed since the previous record

# Command register --------------------------------------------------------------------------------

CMD_NOP     = 0
CMD_WORD    = 1                # load the staged carrier/code frequency words
CMD_ARM     = 2                # load the whole staged assignment, start at the staged sample
CMD_RELEASE = 3

CMD_OP_SHIFT      = 0
CMD_CHANNEL_SHIFT = 8

# Engine pipeline ---------------------------------------------------------------------------------

PIPELINE_DEPTH = 9             # issue -> state write-back, in cycles (see engine.py)

def num_slots(n_channels):
    """Slots per round: one per channel, one for the strobe, and enough idle ones
    that a channel's write-back lands before its next read."""
    return max(n_channels + 1, PIPELINE_DEPTH + 2)

# Capability register ------------------------------------------------------------------------------

GNSS_ID = 0x7D4C0001           # TDM correlator, CSR/record interface revision 1

def capabilities_word(n_channels, code_words):
    """[7:0] channels, [15:8] slots per round, [19:16] max taps, [25:20] history,
    [30:26] log2(code words)."""
    return ((n_channels & 0xFF)
            | (num_slots(n_channels) & 0xFF) << 8
            | (MAX_TAPS & 0xF) << 16
            | (HISTORY & 0x3F) << 20
            | (int(math.log2(code_words)) & 0x1F) << 26)

# Carrier table -----------------------------------------------------------------------------------

def carrier_table():
    """(cos, sin) entries of the carrier table, signed integers."""
    n = 1 << CARRIER_LUT_BITS
    cos_t = [int(round(CARRIER_AMPLITUDE * math.cos(2 * math.pi * k / n))) for k in range(n)]
    sin_t = [int(round(CARRIER_AMPLITUDE * math.sin(2 * math.pi * k / n))) for k in range(n)]
    return cos_t, sin_t

# Host-side helpers -------------------------------------------------------------------------------

def tap_offsets(shifts):
    """(lead, offsets, warmup) for tap sample shifts given in record order.

    ``lead`` is the largest shift (the earliest tap, never behind the prompt),
    the offsets are ``lead - s`` per tap, and ``warmup`` the samples the history
    needs before every tap is valid."""
    lead = max(max(shifts), 0)
    offsets = [lead - s for s in shifts]
    if any(d < 0 or d >= HISTORY for d in offsets):
        raise ValueError(f"tap shifts {shifts} do not fit a {HISTORY}-entry history")
    warmup = max(max(offsets), lead)
    return lead, offsets, warmup
