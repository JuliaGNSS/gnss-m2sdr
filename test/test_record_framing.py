#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Framing of the record stream over a stock litepcie DMA channel.

litepcie's kernel driver writes fixed 8192-byte buffers and drops whole
buffers on overrun, so a record size that does not divide 8192 leaves the
stream permanently misaligned after a drop, and a host that attaches to a
running stream starts mid-record. These tests pin the two properties that
make the stream self-framing: the record divides the DMA buffer exactly, and
every record carries a magic word the host can resynchronise on.
"""

import struct
import unittest

from migen import *
from migen.sim import run_simulation

from gnss_m2sdr.gateware.record import CorrelatorRecorder
from gnss_m2sdr.record_format import (
    DMA_BUFFER_SIZE, MAGIC_OFFSET, RECORDS_PER_DMA_BUFFER, RECORD_BYTES,
    RECORD_MAGIC, RECORD_WORDS, find_record_offset, pack_record,
    parse_records, unpack_record,
)

from test.test_record import collect_records, drive_dump


VALS = dict(sample_index=0x0011223344556677, integrated_samples=4092,
            code_phase=0x00ABCDEF, prn=5,
            i_early=111, q_early=-222, i_prompt=333333, q_prompt=-444444,
            i_late=555, q_late=-666)


def record_bytes(**over):
    """One record as it appears on the wire (little-endian 64-bit words)."""
    v = {**VALS, **over}
    words = pack_record(sample_index=v["sample_index"],
                        integrated_samples=v["integrated_samples"],
                        channel=v.get("channel", 0), prn=v["prn"],
                        seq=v.get("seq", 0), flags=v.get("flags", 0),
                        i_early=v["i_early"],   q_early=v["q_early"],
                        i_prompt=v["i_prompt"], q_prompt=v["q_prompt"],
                        i_late=v["i_late"],     q_late=v["q_late"],
                        code_phase=v["code_phase"])
    return struct.pack("<%dQ" % RECORD_WORDS, *words)


class TestRecordFraming(unittest.TestCase):
    def test_record_divides_dma_buffer(self):
        # 8192 % 48 = 32: records straddle buffers and a dropped buffer shifts
        # the stream by 32 bytes forever. A power-of-two record cannot.
        self.assertEqual(DMA_BUFFER_SIZE % RECORD_BYTES, 0)
        self.assertEqual(RECORD_BYTES * RECORDS_PER_DMA_BUFFER, DMA_BUFFER_SIZE)
        self.assertEqual(RECORD_BYTES & (RECORD_BYTES - 1), 0)  # power of two

    def test_gateware_emits_magic(self):
        dut = CorrelatorRecorder(n_channels=1)
        words = []

        def bench():
            yield from drive_dump(dut.ports[0], VALS)
            yield from collect_records(dut, words, n_words=1)

        run_simulation(dut, bench())
        self.assertEqual(len(words), RECORD_WORDS)
        rec = unpack_record(words)
        self.assertEqual(rec["magic"], RECORD_MAGIC)
        # The magic must not eat the payload it shares a word with.
        self.assertEqual(rec["code_phase"], VALS["code_phase"])
        # ... and it must sit where the host looks for it.
        raw = struct.pack("<%dQ" % RECORD_WORDS, *words)
        self.assertEqual(len(raw), RECORD_BYTES)
        self.assertEqual(struct.unpack_from("<I", raw, MAGIC_OFFSET)[0], RECORD_MAGIC)

    def test_gateware_and_host_packing_agree(self):
        """pack_record() must produce exactly what the FSM puts on the wire."""
        dut = CorrelatorRecorder(n_channels=1)
        words = []

        def bench():
            yield from drive_dump(dut.ports[0], VALS)
            yield from collect_records(dut, words, n_words=1)

        run_simulation(dut, bench())
        self.assertEqual(struct.pack("<%dQ" % RECORD_WORDS, *words), record_bytes())

    def test_resync_after_dropped_buffer(self):
        # A full buffer of records, one dropped buffer, then more records:
        # what survives must still be record-aligned at offset 0.
        stream = b"".join(record_bytes(seq=i & 0xFF)
                          for i in range(RECORDS_PER_DMA_BUFFER))
        stream += b"".join(record_bytes(seq=(i + 2) & 0xFF, sample_index=i)
                           for i in range(4))
        self.assertEqual(len(stream) % RECORD_BYTES, 0)

        # litepcie drops whole 8192-byte buffers -> chop one out.
        after_drop = stream[DMA_BUFFER_SIZE:]
        self.assertEqual(find_record_offset(after_drop), 0)
        recs = parse_records(after_drop)
        self.assertEqual(len(recs), 4)
        self.assertEqual([r["sample_index"] for r in recs], [0, 1, 2, 3])

    def test_resync_from_mid_record_attach(self):
        # Host opens DMA1 while the recorder is already streaming: the first
        # bytes it sees are the tail of a record.
        stream = b"".join(record_bytes(sample_index=i) for i in range(6))
        for skip in (1, 17, RECORD_BYTES - 4):
            with self.subTest(skip=skip):
                partial = stream[skip:]
                off = find_record_offset(partial)
                self.assertEqual(off, RECORD_BYTES - skip)
                recs = parse_records(partial)
                self.assertEqual([r["sample_index"] for r in recs], [1, 2, 3, 4, 5])

    def test_parse_records_resyncs_on_garbage(self):
        # Torn stream: a partial record's worth of junk between good records.
        stream = (record_bytes(sample_index=0) + b"\xa5" * 37 +
                  record_bytes(sample_index=1) + record_bytes(sample_index=2))
        recs = parse_records(stream)
        self.assertEqual([r["sample_index"] for r in recs], [0, 1, 2])


if __name__ == "__main__":
    unittest.main(verbosity=2)
