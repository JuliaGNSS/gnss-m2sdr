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
        from gnss_m2sdr.gateware.rx_observer import RXSampleObserver
        self.gnss = GNSSTracking(
            n_channels     = self._gnss_channels,
            prns           = self._gnss_prns,
            code_frac_bits = self._gnss_frac_bits,
            accum_bits     = self._gnss_accum_bits,
        )
        # Non-intrusive observer: de-interleave each accepted RX word into I/Q
        # samples. How many samples a word carries depends on the AD9361 PHY
        # channel mode -- one (RX1) in 2R2T, two consecutive ones in 1R1T -- so
        # the observer follows the PHY's mode CSR (see rx_observer.py). The main
        # path is unchanged; requires the RX header inserter disabled (default)
        # and DMA0 draining.
        self.gnss_rx = RXSampleObserver(data_width=len(rx_stream.data))
        self.comb += [
            self.gnss_rx.rx_data.eq(rx_stream.data),
            self.gnss_rx.rx_stb.eq(rx_stream.valid & rx_stream.ready),
            self.gnss_rx.mode_1r1t.eq(self.ad9361.phy.control.fields.mode),
            self.gnss.sample_i.eq(self.gnss_rx.sample_i),
            self.gnss.sample_q.eq(self.gnss_rx.sample_q),
            self.gnss.sample_stb.eq(self.gnss_rx.sample_stb),
        ]
        if hasattr(self, "pcie_dma1"):
            self.comb += [
                self.gnss.source.connect(self.pcie_dma1.sink),
                self.pcie_dma1.synchronizer.pps.eq(self.pps_gen.pps_pulse),
            ]
        return rx_stream
