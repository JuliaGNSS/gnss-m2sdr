#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Reading the correlator-record stream off DMA1, with no litex/liblitepcie build.

"""Capture correlator records from the second litepcie DMA channel.

`gnss_m2sdr/record_format.py` can already *parse* a raw DMA1 byte stream; what
was missing on the host was the half that gets the bytes. The M2SDR kernel
driver exposes one char device per DMA channel (`/dev/m2sdr0` = DMA0, the RFIC
I/Q stream; `/dev/m2sdr1` = DMA1, the records), and a channel's writer has to be
armed through `LITEPCIE_IOCTL_DMA_WRITER` before `read()` returns anything --
otherwise the ring never fills and the read blocks forever with no indication
why. Everything below is that ioctl and a blocking read, in the same pure-Python
style as m2sdr_csr.py, so validating the record path needs no C build on the
target.

Two properties of the driver shape the API and are worth knowing before reading
a capture:

  * `read()` copies whole DMA buffers only (`DMA_BUFFER_SIZE`, 8192 bytes) and
    returns short if fewer are ready, so a capture is always a whole number of
    buffers -- which is a whole number of records, since 8192 / 128 = 64 exactly
    (see docs/dma1_record_path.md).
  * a buffer completes only when it is *full*, and `hw_count` only advances in
    the MSI handler, so the first records of a capture can be up to a few
    hundred milliseconds old. Capture for long enough to fill buffers rather
    than expecting the newest dump immediately.
"""

import fcntl
import os
import struct

DMA_BUFFER_SIZE = 8192          # kernel/config.h; one read() unit


def _IOWR(t, nr, size):
    return (3 << 30) | (size << 16) | (t << 8) | nr


def _IOW(t, nr, size):
    return (1 << 30) | (size << 16) | (t << 8) | nr


# struct litepcie_ioctl_dma        { uint8 loopback_enable; }          -> 1 byte
# struct litepcie_ioctl_dma_writer { uint8 enable; int64 hw; int64 sw; }
# The two int64s are 8-aligned, so the struct is 1 + 7 pad + 8 + 8 = 24 bytes.
LITEPCIE_IOCTL_DMA        = _IOW(ord("S"), 20, 1)
LITEPCIE_IOCTL_DMA_WRITER = _IOWR(ord("S"), 21, 24)


class RecordStream:
    """Armed DMA1 writer + blocking reads of raw record bytes.

    Use as a context manager: the writer is disabled again on exit, so a capture
    that raises does not leave the ring running and overflowing behind it.
    """

    def __init__(self, device="/dev/m2sdr1"):
        self.fd = os.open(device, os.O_RDWR)
        self.device = device

    # -- lifecycle ---------------------------------------------------------
    def _dma_writer(self, enable):
        buf = bytearray(24)
        struct.pack_into("<Bxxxxxxxqq", buf, 0, 1 if enable else 0, 0, 0)
        fcntl.ioctl(self.fd, LITEPCIE_IOCTL_DMA_WRITER, buf, True)
        _, hw, sw = struct.unpack_from("<Bxxxxxxxqq", buf, 0)
        return hw, sw

    def start(self):
        # loopback off: the records must come from the gateware, not from the
        # DMA reader looped back into the writer.
        fcntl.ioctl(self.fd, LITEPCIE_IOCTL_DMA, struct.pack("<B", 0))
        self._dma_writer(True)
        return self

    def stop(self):
        try:
            self._dma_writer(False)
        finally:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False

    # -- capture -----------------------------------------------------------
    def counts(self):
        """(hw_count, sw_count) of the writer ring, in buffers."""
        return self._dma_writer(True)

    def read_bytes(self, n_buffers=8):
        """Block until at least one DMA buffer is ready; return what is.

        Short reads are normal -- the driver hands over only the buffers that
        have completed -- so callers accumulate rather than assuming a length.
        """
        want = n_buffers * DMA_BUFFER_SIZE
        return os.read(self.fd, want)

    def capture(self, n_records, timeout=10.0):
        """Raw bytes holding at least `n_records` records, or fewer on timeout.

        Returns the byte stream as read; hand it to
        `gnss_m2sdr.record_format.parse_records`, which does its own framing and
        resynchronisation, rather than slicing it here.
        """
        import time
        from gnss_m2sdr.record_format import RECORD_BYTES

        deadline = time.monotonic() + timeout
        data = bytearray()
        while len(data) < n_records * RECORD_BYTES:
            if time.monotonic() > deadline:
                break
            chunk = self.read_bytes()
            if not chunk:
                time.sleep(0.01)
                continue
            data += chunk
        return bytes(data)
