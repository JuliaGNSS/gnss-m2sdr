#
# This file is part of gnss-m2sdr.
#
# GNSS tracking SoC: litex_m2sdr BaseSoC + on-FPGA tracking bank (no fork).
# SPDX-License-Identifier: BSD-2-Clause

"""GNSSSoC subclasses the litex_m2sdr BaseSoC via the upstream
add_rx_datapath_processing() hook (enjoy-digital/litex_m2sdr#152):

  * observes the RX I/Q sample stream (non-intrusive; DMA0 path unchanged),
  * feeds it to a GNSSTracking channel bank,
  * streams correlator-dump records to the host over a second DMA channel
    (pcie_dmas=2 -> pcie_dma1).

BaseSoC lives in the top-level litex_m2sdr.py script, so it is loaded by file
path (LITEX_M2SDR_DIR) rather than imported as a package.
"""

import os
import importlib.util

LITEX_M2SDR_DIR = os.environ.get(
    "LITEX_M2SDR_DIR", "/workspace/litex_m2sdr-worktrees/create-new-gateware")


def load_base_module():
    path = os.path.join(LITEX_M2SDR_DIR, "litex_m2sdr.py")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"litex_m2sdr.py not found at {path}; set LITEX_M2SDR_DIR.")
    # Ensure the litex_m2sdr package + platform module resolve from that tree.
    import sys
    if LITEX_M2SDR_DIR not in sys.path:
        sys.path.insert(0, LITEX_M2SDR_DIR)
    spec = importlib.util.spec_from_file_location("litex_m2sdr_build", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_m2sdr = load_base_module()
BaseSoC = _m2sdr.BaseSoC


class GNSSSoC(BaseSoC):
    def __init__(self, gnss_channels=4, gnss_prns=None, gnss_frac_bits=24,
                 gnss_accum_bits=32, **kwargs):
        self._gnss_channels  = gnss_channels
        self._gnss_prns      = gnss_prns
        self._gnss_frac_bits = gnss_frac_bits
        self._gnss_accum_bits = gnss_accum_bits
        kwargs.setdefault("with_pcie", True)
        kwargs["pcie_dmas"] = 2  # DMA0 = RFIC I/Q, DMA1 = correlator records
        super().__init__(**kwargs)

    def add_rx_datapath_processing(self, rx_stream):
        from gnss_m2sdr.gateware.bank import GNSSTracking
        self.gnss = GNSSTracking(
            n_channels     = self._gnss_channels,
            prns           = self._gnss_prns,
            code_frac_bits = self._gnss_frac_bits,
            accum_bits     = self._gnss_accum_bits,
        )
        # Non-intrusive observer: every accepted RX word is one I/Q sample
        # (RX1 I in [0:16], RX1 Q in [16:32]). The main path is unchanged;
        # requires the RX header inserter disabled (default) and DMA0 draining.
        self.comb += [
            self.gnss.sample_i.eq(rx_stream.data[0:16]),
            self.gnss.sample_q.eq(rx_stream.data[16:32]),
            self.gnss.sample_stb.eq(rx_stream.valid & rx_stream.ready),
        ]
        if hasattr(self, "pcie_dma1"):
            self.comb += [
                self.gnss.source.connect(self.pcie_dma1.sink),
                self.pcie_dma1.synchronizer.pps.eq(self.pps_gen.pps_pulse),
            ]
        return rx_stream
