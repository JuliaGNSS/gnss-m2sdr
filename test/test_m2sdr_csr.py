#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Host-side CSR driver (software/m2sdr_csr.py).

`software/` is a plain script directory (no package), so the module is loaded
by path. The ioctl layer is stubbed out: what is under test is the
subregister packing, which used to hardcode 32 bits per word while parsing
`config_csr_data_width` and never using it -- on a csr_data_width=8 build every
multi-word CSR read would have come back as garbage.
"""

import importlib.util
import os
import tempfile
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PATH = os.path.join(REPO, "software", "m2sdr_csr.py")

_spec = importlib.util.spec_from_file_location("m2sdr_csr", _PATH)
m2sdr_csr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m2sdr_csr)


class FakeCSR(m2sdr_csr.LiteXCSR):
    """LiteXCSR with the char device replaced by a dict of 32-bit slots."""
    def __init__(self, regs, csr_data_width):
        self.fd   = None
        self.regs = regs
        self.mems = {}
        self.csr_data_width = csr_data_width
        self.words = {}

    def _readl(self, addr):
        return self.words.get(addr, 0)

    def _writel(self, addr, val):
        self.words[addr] = val & 0xFFFFFFFF


def write_csv(rows):
    fd, path = tempfile.mkstemp(suffix=".csv")
    with os.fdopen(fd, "w") as fp:
        fp.write("\n".join(rows) + "\n")
    return path


class TestSubregisterPacking(unittest.TestCase):
    def test_32bit_bus_unchanged(self):
        csr = FakeCSR({"gnss_sample_count": (0x1000, 2)}, csr_data_width=32)
        csr.write("gnss_sample_count", 0x0011223344556677)
        # LiteX big-endian: most-significant subregister at the lowest address.
        self.assertEqual(csr.words[0x1000], 0x00112233)
        self.assertEqual(csr.words[0x1004], 0x44556677)
        self.assertEqual(csr.read("gnss_sample_count"), 0x0011223344556677)

    def test_8bit_bus_uses_csr_data_width(self):
        # A 32-bit CSR on a csr_data_width=8 bus: 4 subregisters, one per
        # 32-bit MMIO slot, carrying 8 bits each.
        csr = FakeCSR({"reg": (0x2000, 4)}, csr_data_width=8)
        csr.write("reg", 0xDEADBEEF)
        self.assertEqual([csr.words[0x2000 + 4 * i] for i in range(4)],
                         [0xDE, 0xAD, 0xBE, 0xEF])
        self.assertEqual(csr.read("reg"), 0xDEADBEEF)

    def test_8bit_bus_read_ignores_upper_slot_bits(self):
        # Only the low csr_data_width bits of each slot belong to the CSR.
        csr = FakeCSR({"reg": (0x3000, 2)}, csr_data_width=8)
        csr.words[0x3000] = 0xFFFFFF12
        csr.words[0x3004] = 0xFFFFFF34
        self.assertEqual(csr.read("reg"), 0x1234)

    def test_single_word_roundtrip(self):
        csr = FakeCSR({"reg": (0x4000, 1)}, csr_data_width=32)
        csr.write("reg", 0x80000001)
        self.assertEqual(csr.read("reg"), 0x80000001)
        self.assertEqual(csr.read_signed("reg"), -0x7FFFFFFF)


class TestCsvAndLifetime(unittest.TestCase):
    """csr.csv parsing + fd handling. A regular file stands in for
    /dev/m2sdr0: only open/close are exercised here."""

    def _open(self, csv_rows):
        csv_path = write_csv(csv_rows)
        self.addCleanup(os.unlink, csv_path)
        fd, dev = tempfile.mkstemp()
        os.close(fd)
        self.addCleanup(os.unlink, dev)
        return csv_path, dev

    def test_parses_csr_data_width_constant(self):
        csv_path, dev = self._open([
            "csr_register,gnss_control,0x1000,1,rw",
            "csr_base,gnss,0x2000,,",
            "constant,config_csr_data_width,8,,",
        ])
        with m2sdr_csr.LiteXCSR(csv_path, device=dev) as csr:
            self.assertEqual(csr.csr_data_width, 8)
            self.assertEqual(csr.regs["gnss_control"], (0x1000, 1))
            self.assertEqual(csr.mems["gnss"], 0x2000)

    def test_defaults_to_32_without_the_constant(self):
        csv_path, dev = self._open(["csr_register,gnss_control,0x1000,1,rw"])
        with m2sdr_csr.LiteXCSR(csv_path, device=dev) as csr:
            self.assertEqual(csr.csr_data_width, 32)

    def test_context_manager_closes_the_fd(self):
        csv_path, dev = self._open(["csr_register,gnss_control,0x1000,1,rw"])
        with m2sdr_csr.LiteXCSR(csv_path, device=dev) as csr:
            fd = csr.fd
            os.fstat(fd)                      # still open inside the block
        with self.assertRaises(OSError):
            os.fstat(fd)
        csr.close()                           # idempotent, must not raise


if __name__ == "__main__":
    unittest.main(verbosity=2)
