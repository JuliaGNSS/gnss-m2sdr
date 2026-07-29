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
    p.add_argument("--vivado-threads", default=0, type=int,
                   help="Vivado general.maxThreads (0 = tool default). The "
                        "2024.1 post-synthesis deadlock is NOT thread-related "
                        "— see the enableParallelHelperSpawn note below — so "
                        "full threading is safe.")
    p.add_argument("--synth-directive", default="default",
                   help="Vivado synth_design -directive. Perturbing it reshapes "
                        "the synthesized netlist, which has dodged the "
                        "2024.1 post-synthesis deadlock when a particular "
                        "channel count triggers it.")
    p.add_argument("--no-project-mode", action="store_true",
                   help="Run Vivado in non-project (batch) mode. Its "
                        "synth_design epilogue differs from project mode's — "
                        "another lever against the post-synthesis deadlock.")
    args = p.parse_args()

    soc = GNSSSoC(
        gnss_channels = args.channels,
        gnss_num_ants = args.num_ants,
        variant       = args.variant,
        with_pcie     = True,
        pcie_lanes    = args.pcie_lanes,
    )
    # Vivado 2024.1's parallel-synthesis HELPER PROCESSES deadlock this design
    # once it is big enough to use them (≥ ~12 channels here): synth_design
    # farms RTL-mapping jobs to forked helpers (the ".Xil/**/realtime/tmp"
    # *.rtd task system), some jobs never gain their .completed marker, and
    # the parent parks in futex_wait forever — reproducibly, right after
    # "Synthesis finished" and before the next Tcl command. The helpers talk
    # over POSIX shared memory (rtSynthParallelPrep.tcl spawns them only when
    # `isSharedMemoryAvailable`), so a host with a small /dev/shm — 63 MB in
    # the build sandbox this was debugged in — starves them mid-run. None of
    # the surface knobs stop the spawn: general.maxThreads, synth.maxThreads,
    # CPU affinity pinning, non-project mode and every synth directive all
    # still hung. The actual switch is the rt-level parameter the spawn is
    # gated on, reachable through the elaboration hook:
    # (Doubled braces: LiteX str.format()s each command, single {} would be
    # eaten as a format field.)
    soc.platform.toolchain.pre_synthesis_commands.append(
        "set_param synth.elaboration.rodinMoreOptions "
        "{{rt::set_parameter enableParallelHelperSpawn false}}")
    build_name = (f"gnss_m2sdr_{args.variant}_x{args.pcie_lanes}"
                  f"_ch{args.channels}_ant{args.num_ants}")
    builder = Builder(soc, output_dir=os.path.join(args.output_dir, build_name),
                      csr_csv=os.path.join(args.output_dir, build_name, "csr.csv"))
    # NOTE: vivado_max_threads is a toolchain *build()* keyword — build()
    # resets the same-named instance attribute to its default, so setting
    # `soc.platform.toolchain.vivado_max_threads` beforehand is silently lost.
    builder.build(build_name=build_name, run=args.build,
                  vivado_max_threads=args.vivado_threads,
                  vivado_synth_directive=args.synth_directive,
                  project_mode=not args.no_project_mode)


if __name__ == "__main__":
    main()
