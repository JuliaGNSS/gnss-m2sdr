#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Minimal CSR access to the LiteX-M2SDR over the litepcie char device, using
# the LITEPCIE_IOCTL_REG ioctl (no litex / C build required). Addresses come
# from the gateware's csr.csv so this tracks rebuilds automatically.

import os
import csv
import fcntl
import struct


def _IOWR(t, nr, size):
    return (3 << 30) | (size << 16) | (t << 8) | nr


# struct litepcie_ioctl_reg { uint32 addr; uint32 val; uint8 is_write; } -> 12 bytes.
_REG_STRUCT_SIZE = 12
LITEPCIE_IOCTL_REG = _IOWR(ord("S"), 0, _REG_STRUCT_SIZE)


class LiteXCSR:
    """Read/write LiteX CSRs via /dev/m2sdrN, resolving names from csr.csv."""
    def __init__(self, csr_csv, device="/dev/m2sdr0"):
        self.fd = os.open(device, os.O_RDWR)
        self.regs = {}       # name -> (addr, size_words)
        self.mems = {}       # name -> base_addr
        self.csr_data_width = 32
        with open(csr_csv) as fp:
            for row in csv.reader(fp):
                if not row:
                    continue
                kind = row[0]
                if kind == "csr_register":
                    _, name, addr, size, _rw = row
                    self.regs[name] = (int(addr, 0), int(size))
                elif kind == "csr_base":
                    _, name, addr, _, _ = row
                    self.mems[name] = int(addr, 0)
                elif kind == "constant" and row[1] == "config_csr_data_width":
                    self.csr_data_width = int(row[2])

    # Low-level 32-bit word access.
    def _readl(self, addr):
        buf = bytearray(_REG_STRUCT_SIZE)
        struct.pack_into("<IIB", buf, 0, addr & 0xFFFFFFFF, 0, 0)
        fcntl.ioctl(self.fd, LITEPCIE_IOCTL_REG, buf, True)
        return struct.unpack_from("<I", buf, 4)[0]

    def _writel(self, addr, val):
        buf = bytearray(_REG_STRUCT_SIZE)
        struct.pack_into("<IIB", buf, 0, addr & 0xFFFFFFFF, val & 0xFFFFFFFF, 1)
        fcntl.ioctl(self.fd, LITEPCIE_IOCTL_REG, buf, True)

    # Named CSR access. A CSR wider than the CSR bus is split into `nwords`
    # subregisters in LiteX big-endian order (most-significant first). Each
    # subregister carries `csr_data_width` bits but occupies one full 32-bit
    # slot in the MMIO map, so the *address* stride is always 4 bytes while the
    # *shift* per subregister is csr_data_width -- assuming 32 there silently
    # returns garbage on a csr_data_width=8 build. Matches
    # litex.tools.remote.csr_builder.
    def read(self, name):
        addr, nwords = self.regs[name]
        mask = (1 << self.csr_data_width) - 1
        val = 0
        for i in range(nwords):
            val = (val << self.csr_data_width) | (self._readl(addr + 4 * i) & mask)
        return val

    def write(self, name, value):
        addr, nwords = self.regs[name]
        mask = (1 << self.csr_data_width) - 1
        for i in range(nwords):
            shift = self.csr_data_width * (nwords - 1 - i)
            self._writel(addr + 4 * i, (value >> shift) & mask)

    def read_signed(self, name, bits=32):
        v = self.read(name)
        return v - (1 << bits) if v & (1 << (bits - 1)) else v

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    # Usable as `with LiteXCSR(csv) as csr:` so scripts stop leaking the fd.
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
