#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Build the GNSS tracking gateware for the LiteX-M2SDR (m2 variant, PCIe)."""

import os
import argparse

from litex.soc.integration.builder import Builder

from gnss_m2sdr.soc import GNSSSoC


def main():
    p = argparse.ArgumentParser(description="GNSS-M2SDR gateware builder.")
    p.add_argument("--build",    action="store_true", help="Build bitstream.")
    p.add_argument("--channels", default=4, type=int,  help="Number of tracking channels.")
    p.add_argument("--num-ants", default=1, type=int, choices=[1, 2],
                   help="Coherent RX antennas per channel (2 needs the AD9361 in 2R2T).")
    p.add_argument("--variant",  default="m2",         help="Board variant.", choices=["m2", "baseboard"])
    p.add_argument("--pcie-lanes", default=1, type=int, choices=[1, 2, 4])
    p.add_argument("--output-dir", default="build",     help="Build output directory.")
    p.add_argument("--vivado-threads", default=1, type=int,
                   help="Vivado general.maxThreads. Default 1: Vivado 2024.1's "
                        "multithreaded timing engine deadlocks on this design "
                        "(observed reproducibly in report_timing_summary right "
                        "after synthesis — every thread parked in futex_wait; "
                        "the netlist pattern that triggers it comes and goes "
                        "with channel count and pipeline placement). "
                        "Single-threaded runs are slower but complete.")
    args = p.parse_args()

    soc = GNSSSoC(
        gnss_channels = args.channels,
        gnss_num_ants = args.num_ants,
        variant       = args.variant,
        with_pcie     = True,
        pcie_lanes    = args.pcie_lanes,
    )
    soc.platform.toolchain.vivado_max_threads = args.vivado_threads
    build_name = (f"gnss_m2sdr_{args.variant}_x{args.pcie_lanes}"
                  f"_ch{args.channels}_ant{args.num_ants}")
    builder = Builder(soc, output_dir=os.path.join(args.output_dir, build_name),
                      csr_csv=os.path.join(args.output_dir, build_name, "csr.csv"))
    builder.build(build_name=build_name, run=args.build)


if __name__ == "__main__":
    main()
