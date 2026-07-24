#
# This file is part of gnss-m2sdr.
#
# Correlator-dump wire format shared by gateware and host driver.
# SPDX-License-Identifier: BSD-2-Clause

"""Fixed-size correlator-dump record streamed over DMA1.

One record = 6 x 64-bit words = 48 bytes, little-endian on the wire:

  word 0 : sample_index                       [63:0]   free-running input-sample counter at dump
  word 1 : integrated_samples [63:32] | channel [31:24] | prn [23:16] | flags [15:8] | seq [7:0]
  word 2 : q_prompt [63:32] | i_prompt [31:0]  (signed)
  word 3 : q_early  [63:32] | i_early  [31:0]  (signed)
  word 4 : q_late   [63:32] | i_late   [31:0]  (signed)
  word 5 : code_phase [31:0] | reserved [63:32]

`seq` is a per-channel record counter (wraps at 256) for host-side loss
detection; `flags` bit 0 = overflow (a dump was dropped before this one).
These map onto Tracking.jl's
CorrelatorOutput(correlator=[early, prompt, late], integrated_samples,
sample_index; code_phase).
"""

RECORD_WORDS = 6
RECORD_BYTES = RECORD_WORDS * 8

FLAG_OVERFLOW = 1 << 0


def pack_record(sample_index, integrated_samples, channel, prn, seq, flags,
                i_early, q_early, i_prompt, q_prompt, i_late, q_late, code_phase):
    """Build the 6 little-endian 64-bit words for one record (for tests)."""
    def u32(x): return x & 0xFFFFFFFF
    w0 = sample_index & ((1 << 64) - 1)
    w1 = ((integrated_samples & 0xFFFFFFFF) << 32) | ((channel & 0xFF) << 24) | \
         ((prn & 0xFF) << 16) | ((flags & 0xFF) << 8) | (seq & 0xFF)
    w2 = (u32(q_prompt) << 32) | u32(i_prompt)
    w3 = (u32(q_early)  << 32) | u32(i_early)
    w4 = (u32(q_late)   << 32) | u32(i_late)
    w5 = u32(code_phase)
    return [w0, w1, w2, w3, w4, w5]


def _s32(x):
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


def unpack_record(words):
    """Inverse of pack_record: dict of fields from 6 words (host-side)."""
    assert len(words) == RECORD_WORDS
    w0, w1, w2, w3, w4, w5 = words
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
    )
