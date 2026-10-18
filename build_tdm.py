#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Build the TDM-correlator gateware for the LiteX-M2SDR (m2 variant, PCIe x1).

    git submodule update --init      # litex_m2sdr, pinned
    python build_tdm.py --build

Writes the bitstream, csr.csv and the kernel/user headers
(build/<name>/software/kernel/{csr,soc,mem}.h) the host tools must be rebuilt
with."""

import argparse
import os

from litex.soc.integration.builder import Builder

from gnss_m2sdr.tdm.soc import TDMSoC


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--build", action="store_true", help="Run Vivado.")
    p.add_argument("--channels", type=int, default=24, help="Correlator channels (<= 32).")
    p.add_argument("--code-words", type=int, default=4096, help="Code memory, 32-chip words (power of two).")
    p.add_argument("--output-dir", default="build")
    args = p.parse_args()

    soc = TDMSoC(gnss_channels=args.channels, gnss_code_words=args.code_words,
                 variant="m2", with_pcie=True, pcie_lanes=1, sys_clk_freq=int(125e6))
    name = f"gnss_tdm_m2_x1_ch{args.channels}_code{args.code_words}"
    out = os.path.join(args.output_dir, name)
    builder = Builder(soc, output_dir=out, csr_csv=os.path.join(out, "csr.csv"))
    # The directives upstream's own build uses for its 125 MHz sys clock.
    builder.build(build_name=name, run=args.build,
                  vivado_place_directive               = "ExtraTimingOpt",
                  vivado_post_place_phys_opt_directive = "AggressiveExplore",
                  vivado_route_directive               = "AggressiveExplore",
                  vivado_post_route_phys_opt_directive = "AggressiveExplore")

    from litex_m2sdr.software import generate_litepcie_software_headers
    kdir = os.path.join(out, "software", "kernel")
    os.makedirs(kdir, exist_ok=True)
    generate_litepcie_software_headers(soc, kdir)


if __name__ == "__main__":
    main()
