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

Replica
-------
A channel's replica is the code chip times a sub-chip value: the chip's
fraction is cut into ``subchips`` (P, at most ``MAX_SUBCHIPS``) equal parts,
and part ``k = floor(frac * P)`` -- the full-precision product, not the top
bits of the fraction, since 1/12 is not dyadic -- has a sign and one of two
amplitudes, ``amp_hi`` or ``amp_lo`` (unsigned ``AMP_BITS``). So the replica
is ``chip * sign[k] * (amp_hi if level[k] else amp_lo)``:

- BPSK (``LOC``): P = 1, amplitudes 1, the replica +/-1 as before.
- ``BOCsin(m, 1)``: P = 2m, sign[k] = k odd, amplitudes 1.
- Galileo E1 ``CBOC(BOC(1,1), BOC(6,1), 10/11)``: P = 12 and GNSSSignals'
  Int8 levels a1 * BOC(1,1) + a2 * BOC(6,1) with (a1, a2) = (19, 6), i.e.
  +/-25 and +/-13 -- the replica Tracking.jl's integer correlator uses, whose
  RMS is ``get_code_amplitude(GalileoE1B())``.

The amplitude scales the accumulators: a channel's records carry the replica
amplitude the host programmed.

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
MAX_SUBCHIPS      = 12          # sub-chip table entries (Galileo E1 CBOC needs 12)
SUBCHIP_BITS      = 4
AMP_BITS          = 7           # replica amplitudes, unsigned (GNSSSignals' Int8 levels fit)

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
FLAG_SAMPLE_OVERFLOW = 1 << 1  # samples were lost during this record (the sample index jumped:
                               # a full sample FIFO, or the engine disabled), so the NCOs fell
                               # behind the index

# Command register --------------------------------------------------------------------------------

CMD_NOP     = 0
CMD_WORD    = 1                # load the staged carrier/code frequency words
CMD_ARM     = 2                # load the whole staged assignment, start at the staged sample
CMD_RELEASE = 3

CMD_OP_SHIFT      = 0
CMD_CHANNEL_SHIFT = 8

# Engine pipeline ---------------------------------------------------------------------------------

WRITEBACK_STAGE = 7            # a slot issued in cycle t writes its state back in cycle t + 7 (S7,
                               # engine.py), and a read issued from cycle t + 8 on sees it

def num_slots(n_channels):
    """Slots per round: one per channel, one for the strobe, and enough idle ones
    that a channel's write-back lands before its next read. A round is the slots
    plus one cycle to load the next sample, so a slot is issued again
    ``num_slots + 1 >= WRITEBACK_STAGE + 1`` cycles later at the earliest."""
    return max(n_channels + 1, WRITEBACK_STAGE)

# Epoch strobes -----------------------------------------------------------------------------------
#
# A strobe follows sample n (and carries sample index n + 1) when the index passes a multiple of
# the strobe period: n + 1 >= next, where next is the first multiple of the period not yet
# strobed. Strobes therefore sit on the multiples of the period, whatever samples were lost, and
# a gap that spans several multiples gives one strobe. After a reset or a write of the period the
# engine finds the period's phase from the first sample it processes: that sample and the
# STROBE_SYNC_SAMPLES - 1 after it give no strobe, and the first multiple strobed is the first at
# or after that sample's index + 1 + STROBE_SYNC_SAMPLES. A gap so long that ``next`` is still
# behind after a strobe starts that over at the next sample.

STROBE_SYNC_SAMPLES = 8

def strobe_next(index, period):
    """The first multiple of ``period`` at or after ``index``."""
    return -(-index // period) * period

# Capability register ------------------------------------------------------------------------------

GNSS_ID = 0x7D4C0002           # TDM correlator, CSR/record interface revision 2 (revision 1
                               # had no sub-chip replica: no s_subcarrier, s_amplitude)

def check_code_words(code_words):
    """The code memory is addressed modulo its size, so it must be a power of two
    (and its address is 16 bits)."""
    if not (0 < code_words <= 1 << 16 and code_words & (code_words - 1) == 0):
        raise ValueError(f"code_words must be a power of two up to 65536, not {code_words}")

def capabilities_word(n_channels, code_words):
    """[7:0] channels, [15:8] slots per round, [19:16] max taps, [25:20] history,
    [30:26] log2(code words)."""
    check_code_words(code_words)
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

def subcarrier_word(subchips, signs, levels):
    """``s_subcarrier``: [11:0] sign bits (1 = negative), [23:12] level bits
    (1 = ``amp_hi``), [27:24] sub-chips, for per-sub-chip ``signs`` and ``levels``."""
    if not (1 <= subchips <= MAX_SUBCHIPS and len(signs) == len(levels) == subchips):
        raise ValueError(f"{subchips} sub-chips; a channel has 1 to {MAX_SUBCHIPS}")
    sign = sum(int(bool(b)) << k for k, b in enumerate(signs))
    level = sum(int(bool(b)) << k for k, b in enumerate(levels))
    return sign | level << MAX_SUBCHIPS | subchips << (2 * MAX_SUBCHIPS)

def amplitude_word(amp_hi, amp_lo):
    """``s_amplitude``: [6:0] the high amplitude, [14:8] the low one."""
    if not (0 <= amp_hi < 1 << AMP_BITS and 0 <= amp_lo < 1 << AMP_BITS):
        raise ValueError(f"amplitudes {amp_hi}, {amp_lo} do not fit {AMP_BITS} bits")
    return amp_hi | amp_lo << 8

SUBCARRIER_BPSK = subcarrier_word(1, [0], [0])   # the reset values: a BPSK replica, amplitude 1
AMPLITUDE_UNIT  = amplitude_word(1, 1)

def subchip_index(code_frac, subchips):
    """The sub-chip a code fraction (32-bit) falls in: floor(frac * P)."""
    return (code_frac * subchips) >> FRAC_BITS

def cboc_table(m1, m2, a1, a2):
    """(subchips, signs, levels, amp_hi, amp_lo) of a1 * BOCsin(m1, 1) + a2 * BOCsin(m2, 1)
    (a2 < 0 for CBOC(-)), the per-sub-chip values GNSSSignals' Int8 code table holds."""
    P = 2 * math.lcm(m1, m2)
    boc = lambda m, k: -1 if (k * 2 * m // P) % 2 else 1
    values = [a1 * boc(m1, k) + a2 * boc(m2, k) for k in range(P)]
    mags = sorted({abs(v) for v in values})
    if len(mags) > 2 or 0 in mags:
        raise ValueError(f"CBOC levels {values} are not two non-zero amplitudes")
    hi, lo = mags[-1], mags[0]
    return P, [v < 0 for v in values], [abs(v) == hi for v in values], hi, lo

def tap_offsets(shifts):
    """(lead, offsets, warmup) for tap sample shifts given in record order.

    ``lead`` is the largest shift (the earliest tap, never behind the prompt),
    the offsets are ``lead - s`` per tap, and ``warmup`` the samples the history
    needs before every tap is valid."""
    if not 1 <= len(shifts) <= MAX_TAPS:
        raise ValueError(f"{len(shifts)} taps; a channel has 1 to {MAX_TAPS}")
    lead = max(max(shifts), 0)
    offsets = [lead - s for s in shifts]
    # lead, the offsets and warmup are HISTORY_BITS-wide fields of the arm, and
    # the prompt reads history entry ``lead``: all must index the history.
    if lead >= HISTORY or any(d < 0 or d >= HISTORY for d in offsets):
        raise ValueError(f"tap shifts {shifts} do not fit a {HISTORY}-entry history")
    warmup = max(max(offsets), lead)
    return lead, offsets, warmup
