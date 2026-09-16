#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Symmetric Early/Late tap offsets, for tests written before the taps split.

The gateware has one signed offset register per tap -- a five-tap layout is not
describable by a single spacing number, which is why GNSSReceiver hands over the
whole `tap_sample_shifts` array. The many tests that only ever wanted "E and L
half a chip either side of the prompt" say so through these helpers rather than
open-coding the two's complement of the late offset in every bench.
"""


def _tap_index(dut, name):
    names = dut.tap_names if hasattr(dut, "tap_names") else dut.code.tap_names
    return names.index(name)


def set_el_offsets(dut, word):
    """E at +`word` chips and L at -`word`, on a CodeReplica/TrackingChannel."""
    yield dut.tap_offset[_tap_index(dut, "e")].eq(word)
    yield dut.tap_offset[_tap_index(dut, "l")].eq(-word)


def set_el_offsets_csr(chan, word, frac_bits):
    """The same, through a ChannelWithCSR's per-tap storages."""
    mask = (1 << (frac_bits + 1)) - 1
    yield chan._tap_offset_e.storage.eq(word)
    yield chan._tap_offset_l.storage.eq(-word & mask)
