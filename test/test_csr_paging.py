#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""The CSR paging invariant behind `attach_channels`.

A LiteX CSR bank pages at 0x800 bytes = 512 CSR words, and LiteX does **not**
fail a build whose bank outgrows its page: it emits a csr.csv whose upper
registers silently overlap the next bank. Measured on a 20-channel build with
every channel inside the one `gnss` bank: `gnss_ch12_...` and up landed on top
of the `header` bank — reads went to the wrong module with no error anywhere.

The layout that scales is one SoC-level bank per channel (soc.py names them
`gnss_ch<i>`, so the emitted register names do not change). These tests pin
the two sides of that invariant:

  * the shared `gnss` bank must stay within one page no matter how many
    channels are built (its per-channel content is one `dropped<i>` counter);
  * a single channel must stay within one page (it is its own bank now);
  * and the default all-in-one layout really does outgrow a page past 17
    channels — the measurement that motivates all of the above. If this ever
    stops holding (channel CSRs got slimmer), the split is no longer load-
    bearing and this file should be revisited.
"""

import unittest

from gnss_m2sdr.gateware.bank import GNSSTracking

PAGE_WORDS = 0x800 // 4      # LiteX csr_paging (bytes) / 32-bit slot
CSR_DATA_WIDTH = 32


def words(csrs):
    """CSR words the bank allocator will spend on `csrs` (multi-word CSRs
    occupy one slot per csr_data_width chunk)."""
    return sum((c.size + CSR_DATA_WIDTH - 1) // CSR_DATA_WIDTH for c in csrs)


class TestCSRPaging(unittest.TestCase):
    def test_all_in_one_bank_outgrows_a_page_past_17_channels(self):
        # The trap this file exists for: 20 channels in one bank do not fit,
        # and LiteX would emit an overlapping map rather than an error.
        bank = GNSSTracking(n_channels=20, attach_channels=True)
        self.assertGreater(words(bank.get_csrs()), PAGE_WORDS)

    def test_shared_bank_fits_one_page_even_at_64_channels(self):
        # PRNs repeat: the build-time ROM PRN only seeds the code RAM (it is
        # runtime-loadable); the C/A table has 32 entries.
        bank = GNSSTracking(
            n_channels=64,
            prns=[(i % 32) + 1 for i in range(64)],
            attach_channels=False,
        )
        self.assertLessEqual(words(bank.get_csrs()), PAGE_WORDS)

    def test_one_channel_fits_one_page(self):
        bank = GNSSTracking(n_channels=1, attach_channels=False)
        (chan,) = bank.channels
        self.assertLessEqual(words(chan.get_csrs()), PAGE_WORDS)

    def test_detached_channels_own_their_csrs(self):
        # With attach_channels=False the channels' CSRs must NOT be collected
        # through the bank (they would land in its page again) — each channel
        # carries them itself for the SoC to attach as its own bank.
        n = 4
        bank = GNSSTracking(n_channels=n, attach_channels=False)
        self.assertEqual(len(bank.channels), n)
        # Identity, not name: the bank and a channel both have a CSR *named*
        # `control` — legitimately, they live in different banks.
        bank_csrs = set(map(id, bank.get_csrs()))
        for chan in bank.channels:
            chan_csrs = chan.get_csrs()
            self.assertGreater(len(chan_csrs), 0)
            for c in chan_csrs:
                self.assertNotIn(id(c), bank_csrs)


if __name__ == "__main__":
    unittest.main()
