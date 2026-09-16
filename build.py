#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Build the GNSS tracking gateware for the LiteX-M2SDR (m2 variant, PCIe)."""

import os
import argparse

from litex.soc.integration.builder import Builder

from gnss_m2sdr.gps_ca import CA_CODE_LENGTH
from gnss_m2sdr.soc import GNSSSoC


def main():
    p = argparse.ArgumentParser(description="GNSS-M2SDR gateware builder.")
    p.add_argument("--build",    action="store_true", help="Build bitstream.")
    p.add_argument("--channels", default=4, type=int,  help="Number of tracking channels.")
    p.add_argument("--num-ants", default=1, type=int, choices=[1, 2],
                   help="Coherent RX antennas per channel (2 needs the AD9361 in 2R2T).")
    p.add_argument("--max-code-length", default=CA_CODE_LENGTH, type=int,
                   help="Longest primary code a channel can hold, in chips. This sizes "
                        "the code RAM (3 taps x this many bits per channel) and is the "
                        "one signal capability a runtime write cannot change; the "
                        "gateware reports it as max_primary_code_length. 1023 = GPS L1 "
                        "C/A only, 4092 covers Galileo E1 and GPS L1C, 10230 covers "
                        "every BPSK primary code in scope. See "
                        "docs/signal_configuration.md.")
    p.add_argument("--variant",  default="m2",         help="Board variant.", choices=["m2", "baseboard"])
    p.add_argument("--pcie-lanes", default=1, type=int, choices=[1, 2, 4])
    p.add_argument("--output-dir", default="build",     help="Build output directory.")
    args = p.parse_args()

    soc = GNSSSoC(
        gnss_channels = args.channels,
        gnss_num_ants = args.num_ants,
        gnss_max_code_length = args.max_code_length,
        variant       = args.variant,
        with_pcie     = True,
        pcie_lanes    = args.pcie_lanes,
    )
    build_name = (f"gnss_m2sdr_{args.variant}_x{args.pcie_lanes}"
                  f"_ch{args.channels}_ant{args.num_ants}"
                  f"_code{args.max_code_length}")
    builder = Builder(soc, output_dir=os.path.join(args.output_dir, build_name),
                      csr_csv=os.path.join(args.output_dir, build_name, "csr.csv"))
    builder.build(build_name=build_name, run=args.build)


if __name__ == "__main__":
    main()
