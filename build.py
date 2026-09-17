#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Build the GNSS tracking gateware for the LiteX-M2SDR (m2 variant, PCIe)."""

import os
import argparse

from litex.soc.integration.builder import Builder

from gnss_m2sdr.gps_ca import CA_CODE_LENGTH
from gnss_m2sdr.record_format import TAPS_EPL, TAPS_VEPL
from gnss_m2sdr.soc import GNSSSoC


def main():
    p = argparse.ArgumentParser(description="GNSS-M2SDR gateware builder.")
    p.add_argument("--build",    action="store_true", help="Build bitstream.")
    p.add_argument("--channels", default=4, type=int,  help="Number of tracking channels.")
    p.add_argument("--num-ants", default=1, type=int, choices=[1, 2],
                   help="Coherent RX antennas per channel (2 needs the AD9361 in 2R2T).")
    p.add_argument("--max-code-length", default=CA_CODE_LENGTH, type=int,
                   help="Longest primary code a channel can hold, in chips. This sizes "
                        "the code RAM (3 copies x this many words per channel, "
                        "independent of the tap count) and is the one signal "
                        "capability a runtime write cannot change; the gateware "
                        "reports it as max_primary_code_length. 1023 = GPS L1 "
                        "C/A only, 4092 covers Galileo E1 and GPS L1C, 10230 covers "
                        "every BPSK primary code in scope. See "
                        "docs/signal_configuration.md.")
    p.add_argument("--taps", default=TAPS_VEPL, type=int, choices=[TAPS_EPL, TAPS_VEPL],
                   help="Widest correlator layout the bank produces. 5 adds the "
                        "Very Early / Very Late taps Tracking uses for the BOC-family "
                        "signals; each channel still chooses 3 or 5 at runtime, so a "
                        "5-tap build runs GPS L1 C/A on 3 taps at the same time.")
    p.add_argument("--max-subchips", default=12, type=int,
                   help="Sub-chip subcarrier table depth. 1 = plain BPSK only "
                        "(:LOC); 2 reaches BOC(1,1); 4 reaches BOCcos(1,1); 12 "
                        "covers CBOC(6,1) and TMBOC(6,1), i.e. every L1 modulation "
                        "GNSSSignals exposes. The gateware declares its modulations "
                        "from this number, so it cannot over-declare.")
    p.add_argument("--variant",  default="m2",         help="Board variant.", choices=["m2", "baseboard"])
    p.add_argument("--pcie-lanes", default=1, type=int, choices=[1, 2, 4])
    p.add_argument("--output-dir", default="build",     help="Build output directory.")
    p.add_argument("--timing-effort", default="default",
                   choices=["default", "high", "max"],
                   help="Vivado implementation effort. 'high' asks for "
                        "ExtraTimingOpt placement, Explore routing and an "
                        "AggressiveExplore post-route phys_opt; it roughly "
                        "doubles the run. The five-tap builds need it: at four "
                        "channels the design leaves litex_m2sdr's own AD9361 "
                        "block-floating-point path about 0.26 ns short at the "
                        "default effort, and that path is 52% routing, which is "
                        "what the stronger directives are for. See "
                        "docs/gateware_builds.md.")
    args = p.parse_args()

    soc = GNSSSoC(
        gnss_channels = args.channels,
        gnss_num_ants = args.num_ants,
        gnss_max_code_length = args.max_code_length,
        gnss_num_taps        = args.taps,
        gnss_max_subchips    = args.max_subchips,
        variant       = args.variant,
        with_pcie     = True,
        pcie_lanes    = args.pcie_lanes,
    )
    build_name = (f"gnss_m2sdr_{args.variant}_x{args.pcie_lanes}"
                  f"_ch{args.channels}_ant{args.num_ants}"
                  f"_code{args.max_code_length}"
                  f"_tap{args.taps}_sub{args.max_subchips}")
    builder = Builder(soc, output_dir=os.path.join(args.output_dir, build_name),
                      csr_csv=os.path.join(args.output_dir, build_name, "csr.csv"))
    # Vivado is deterministic for a given netlist and directive set, so a build
    # that misses by picoseconds cannot be "tried again" -- the directives have
    # to change. 'max' escalates the two passes that move a sub-100 ps setup
    # miss: routing explores harder, and the post-place phys_opt matches the
    # post-route one instead of trailing it.
    effort = {}
    if args.timing_effort == "high":
        effort = dict(
            vivado_place_directive               = "ExtraTimingOpt",
            vivado_post_place_phys_opt_directive = "Explore",
            vivado_route_directive               = "Explore",
            vivado_post_route_phys_opt_directive = "AggressiveExplore",
        )
    elif args.timing_effort == "max":
        effort = dict(
            vivado_place_directive               = "ExtraTimingOpt",
            vivado_post_place_phys_opt_directive = "AggressiveExplore",
            vivado_route_directive               = "AggressiveExplore",
            vivado_post_route_phys_opt_directive = "AggressiveExplore",
        )
    builder.build(build_name=build_name, run=args.build, **effort)


if __name__ == "__main__":
    main()
