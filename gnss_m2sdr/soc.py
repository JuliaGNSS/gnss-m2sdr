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

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Where to look for the litex_m2sdr checkout when LITEX_M2SDR_DIR is unset.
# Conventional locations only -- no developer's worktree path.
LITEX_M2SDR_DIR_CANDIDATES = (
    os.path.expanduser("~/litex_m2sdr"),
    os.path.join(os.path.dirname(_REPO_ROOT), "litex_m2sdr"),   # sibling checkout
)


def find_litex_m2sdr_dir(env=None, candidates=None):
    """Directory holding litex_m2sdr.py.

    $LITEX_M2SDR_DIR wins outright when set (and is not silently ignored when
    it is wrong); otherwise the conventional locations are tried. If nothing
    matches, the error names every path that was tried -- the previous default
    pointed at one developer's worktree, so everyone else hit a
    FileNotFoundError for a directory they had never heard of.
    """
    env       = os.environ if env is None else env
    candidates = LITEX_M2SDR_DIR_CANDIDATES if candidates is None else candidates
    explicit  = env.get("LITEX_M2SDR_DIR")
    tried     = [explicit] if explicit else list(candidates)
    for directory in tried:
        if os.path.exists(os.path.join(directory, "litex_m2sdr.py")):
            return directory
    raise FileNotFoundError(
        "litex_m2sdr.py not found; set LITEX_M2SDR_DIR to the litex_m2sdr "
        "checkout (the directory containing litex_m2sdr.py). Tried: "
        + ", ".join(tried))


def require_record_dma(soc):
    """Fail loudly if the correlator-record DMA is missing.

    GNSSSoC forces pcie_dmas=2, so an absent pcie_dma1 is a build bug. Leaving
    the record stream unconnected is the worst possible failure mode: the FIFO
    fills, the recorder stalls in EMIT and every channel starts reporting
    overflow, with nothing anywhere pointing at the actual cause.
    """
    if not hasattr(soc, "pcie_dma1"):
        raise AttributeError(
            "no pcie_dma1: GNSSSoC needs a second PCIe DMA to stream correlator "
            "records (build with with_pcie=True; GNSSSoC sets pcie_dmas=2).")


def load_base_module():
    directory = find_litex_m2sdr_dir()
    path = os.path.join(directory, "litex_m2sdr.py")
    # Ensure the litex_m2sdr package + platform module resolve from that tree.
    import sys
    if directory not in sys.path:
        sys.path.insert(0, directory)
    spec = importlib.util.spec_from_file_location("litex_m2sdr_build", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_m2sdr = load_base_module()
BaseSoC = _m2sdr.BaseSoC


class GNSSSoC(BaseSoC):
    def __init__(self, gnss_channels=4, gnss_prns=None, gnss_frac_bits=24,
                 gnss_accum_bits=32, gnss_num_ants=1, **kwargs):
        self._gnss_channels  = gnss_channels
        self._gnss_prns      = gnss_prns
        self._gnss_frac_bits = gnss_frac_bits
        self._gnss_accum_bits = gnss_accum_bits
        self._gnss_num_ants  = gnss_num_ants
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
            num_ants       = self._gnss_num_ants,
            # The channels become SoC-level CSR banks below. Inside the one
            # `gnss` bank they overflow its 0x800-byte page at 17 channels, and
            # LiteX does not catch that: it emits a csr.csv whose upper
            # registers silently overlap the next bank (see GNSSTracking).
            attach_channels = False,
        )
        # One CSR bank per channel, named so the emitted register names
        # (`gnss_ch<i>_<csr>`) are identical to the attach_channels=True
        # layout — csr.csv-driven hosts cannot tell the difference.
        for i, chan in enumerate(self.gnss.channels):
            setattr(self, f"gnss_ch{i}", chan)
        # Non-intrusive observer: de-interleave each accepted RX word into I/Q
        # samples. What a word's two slots carry depends on the AD9361 PHY
        # channel mode -- RX1/RX2 of one instant in 2R2T, two consecutive samples
        # of the single RX in 1R1T -- so the observer follows the PHY's mode CSR
        # and reports how many antennas it actually sees (see rx_observer.py).
        # The main path is unchanged; requires the RX header inserter disabled
        # (default) and DMA0 draining.
        self.gnss_rx = RXSampleObserver(data_width=len(rx_stream.data),
                                        num_ants=self._gnss_num_ants)
        self.comb += [
            self.gnss_rx.rx_data.eq(rx_stream.data),
            self.gnss_rx.rx_stb.eq(rx_stream.valid & rx_stream.ready),
            self.gnss_rx.mode_1r1t.eq(self.ad9361.phy.control.fields.mode),
            # Static configuration, not per-sample data: no need to pipeline it.
            self.gnss.ants_valid.eq(self.gnss_rx.ants_valid),
        ]
        # Register the observer -> bank sample handoff.
        #
        # The tap's strobe is `rx_stream.valid & rx_stream.ready`, and that
        # `ready` comes combinationally out of the DMA0 buffering FIFO's level
        # comparator. Wiring it straight through put the FIFO level register, the
        # comparator's carry chain and a channel's whole multiply-accumulate on
        # one 8 ns path: the 4-channel m2 build missed setup by ~1 ns on ~2400
        # endpoints in the sys_clk domain, worst path running from
        # `litepciedma0_buffering_syncfifo1`'s level register into a correlator
        # DSP48's cascade input. One register here splits that path in two.
        #
        # Costs nothing functionally: the observer never back-pressures the RX
        # stream, so a uniform one-cycle delay just shifts when the bank sees each
        # sample. It stays self-consistent because the bank's free-running sample
        # counter increments on this same `sample_stb` (bank.py), so samples,
        # counter and dump tags all move together; only the constant offset
        # between a word leaving the AD9361 and the bank counting it changes, and
        # nothing depends on that.
        #
        # Deliberately here rather than inside `RXSampleObserver`: registering the
        # observer's own outputs is functionally identical, but made Vivado 2024.1
        # hang during implementation on the 4-channel build (reproduced 4/4, while
        # 2-channel and unpipelined 4-channel builds completed). Same pipeline
        # stage, expressed at the SoC boundary instead.
        self.sync += [
            *[self.gnss.sample_i_ants[n].eq(self.gnss_rx.sample_i_ants[n])
              for n in range(self._gnss_num_ants)],
            *[self.gnss.sample_q_ants[n].eq(self.gnss_rx.sample_q_ants[n])
              for n in range(self._gnss_num_ants)],
            self.gnss.sample_stb.eq(self.gnss_rx.sample_stb),
        ]
        require_record_dma(self)
        self.comb += [
            self.gnss.source.connect(self.pcie_dma1.sink),
            self.pcie_dma1.synchronizer.pps.eq(self.pps_gen.pps_pulse),
        ]
        return rx_stream
