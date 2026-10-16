#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""TDMSoC: the upstream litex_m2sdr BaseSoC with the TDM correlator on its RFIC
datapath hook (``add_rfic_datapath``, enjoy-digital/litex_m2sdr#185).

  * RX: RFIC -> RXTap (counts, optionally tags) -> pipeline buffer -> host (DMA0).
    The correlator sees exactly the words the host path accepts, so the host's
    raw stream and the correlator's sample index count the same samples.
  * Records: correlator -> DMA1 (``pcie_dmas = 2``), whose buffer geometry the
    kernel module sets per channel (``dma_buffer_size=8192,512``).
  * TX: unchanged passthrough.
"""

import os
import sys
import importlib.util

from migen import *

from litex.soc.interconnect import stream

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def find_litex_m2sdr_dir(env=None):
    """$LITEX_M2SDR_DIR, else ~/litex_m2sdr, else a checkout next to this repo."""
    env = os.environ if env is None else env
    explicit = env.get("LITEX_M2SDR_DIR")
    tried = [explicit] if explicit else [os.path.expanduser("~/litex_m2sdr"),
                                         os.path.join(os.path.dirname(_REPO_ROOT), "litex_m2sdr")]
    for directory in tried:
        if os.path.exists(os.path.join(directory, "litex_m2sdr.py")):
            return directory
    raise FileNotFoundError("litex_m2sdr.py not found; set LITEX_M2SDR_DIR. Tried: " + ", ".join(tried))


def _load_base_module():
    directory = find_litex_m2sdr_dir()
    if directory not in sys.path:
        sys.path.insert(0, directory)
    spec = importlib.util.spec_from_file_location("litex_m2sdr_build", os.path.join(directory, "litex_m2sdr.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod.BaseSoC, "add_rfic_datapath"):
        raise RuntimeError(f"{directory} predates the add_rfic_datapath() hook; "
                           "use litex_m2sdr at or after enjoy-digital/litex_m2sdr#185")
    return mod


_m2sdr = _load_base_module()
BaseSoC = _m2sdr.BaseSoC


class TDMSoC(BaseSoC):
    def __init__(self, gnss_channels=24, gnss_code_words=4096, **kwargs):
        self._gnss_channels = gnss_channels
        self._gnss_code_words = gnss_code_words
        kwargs.setdefault("with_pcie", True)
        kwargs["pcie_dmas"] = 2          # DMA0 = RFIC I/Q, DMA1 = correlator records
        super().__init__(**kwargs)

    def add_rfic_datapath(self, rfic_sink, rfic_source, tx_source, rx_sink):
        from gnss_m2sdr.tdm.engine import TDMCorrelator, RXTap

        if not hasattr(self, "pcie_dma1"):
            raise AttributeError("TDMSoC needs a second PCIe DMA for the records (with_pcie=True)")
        if len(rfic_source.data) != 64:
            raise ValueError("the TDM correlator expects the 64-bit RFIC datapath "
                             "(no --with-rfic-oversampling)")

        # TX passthrough.
        self.comb += tx_source.connect(rfic_sink)

        # RX: tap, then a buffer that registers both valid and ready, so the DMA's
        # readiness logic ends here instead of in the correlator's sample FIFO.
        self.gnss_tap = tap = RXTap()
        self.gnss_rx_buffer = buf = stream.Buffer([("data", 64)], pipe_valid=True, pipe_ready=True)
        self.comb += [
            rfic_source.connect(tap.sink),
            tap.source.connect(buf.sink),
            buf.source.connect(rx_sink),
        ]

        self.gnss = gnss = TDMCorrelator(n_channels=self._gnss_channels, code_words=self._gnss_code_words)
        self.comb += [
            gnss.sample_valid.eq(tap.valid),
            gnss.sample_data.eq(tap.data),
            gnss.sample_index.eq(tap.index),
            tap.tag_enable.eq(gnss.tag_enable),
            gnss.source.connect(self.pcie_dma1.sink),
            self.pcie_dma1.synchronizer.pps.eq(self.pps_gen.pps_pulse),
        ]
