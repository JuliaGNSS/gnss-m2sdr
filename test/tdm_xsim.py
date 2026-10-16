#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Run a TDM-correlator ``Scenario`` through Vivado's xsim on the Verilog LiteX's
converter writes -- the artefact the bitstream is built from -- and return the
64-bit words it streamed.

The Migen simulator never sees that Verilog, and two faults have reached
silicon through exactly that gap (docs/gateware_builds.md 5.6e, 5.6g), so the
TDM suite runs every scenario through both.

xvlog/xelab/xsim are taken from PATH, from ``$XILINX_VIVADO/bin``, or run
through ``$XILINX_ENV`` (a wrapper that sets up Vivado's environment and execs
its arguments).
"""

import os
import shutil
import subprocess

from gnss_m2sdr.tdm import params as P
from test.tdm_harness import TDMHarness


def tool_prefix():
    """The command prefix that runs a Vivado tool, or None if there is none."""
    wrapper = os.environ.get("XILINX_ENV")
    if wrapper and os.path.exists(wrapper):
        return [wrapper]
    if shutil.which("xvlog"):
        return []
    root = os.environ.get("XILINX_VIVADO")
    if root and os.path.exists(os.path.join(root, "bin", "xvlog")):
        return ["env", f"PATH={os.path.join(root, 'bin')}:{os.environ.get('PATH', '')}"]
    return None


TESTBENCH = r"""
`timescale 1ns/1ps
module tb;
  reg clk = 0;
  reg rst = 1;
  always #4 clk = ~clk;
  reg        wr_valid = 0;
  reg [7:0]  wr_addr = 0;
  reg [31:0] wr_data = 0;
  reg        sample_valid = 0;
  reg [63:0] sample_data = 0;
  reg [47:0] sample_index = 0;
  reg        out_ready = 1;
  wire       out_valid;
  wire [63:0] out_data;
  top dut(
    .sys_clk(clk), .sys_rst(rst),
    .wr_valid(wr_valid), .wr_addr(wr_addr), .wr_data(wr_data),
    .sample_valid(sample_valid), .sample_data(sample_data), .sample_index(sample_index),
    .out_valid(out_valid), .out_data(out_data), .out_ready(out_ready));
  integer fin, fout, r;
  reg [0:0]  v_wv; reg [7:0] v_wa; reg [31:0] v_wd;
  reg [0:0]  v_sv; reg [63:0] v_sd; reg [47:0] v_si; reg [0:0] v_rdy;
  initial begin
    fin  = $fopen("stim.txt", "r");
    fout = $fopen("out.txt", "w");
    repeat (4) @(posedge clk);
    rst <= 0;
    while (!$feof(fin)) begin
      r = $fscanf(fin, "%h %h %h %h %h %h %h\n", v_wv, v_wa, v_wd, v_sv, v_sd, v_si, v_rdy);
      if (r == 7) begin
        wr_valid <= v_wv; wr_addr <= v_wa; wr_data <= v_wd;
        sample_valid <= v_sv; sample_data <= v_sd; sample_index <= v_si;
        out_ready <= v_rdy;
        @(posedge clk);
        #1;
        if (out_valid) $fwrite(fout, "%h\n", out_data);
      end
    end
    $fclose(fout);
    $finish;
  end
endmodule
"""


def run(scenario, out_dir, n_channels, code_words, ready_pattern=None):
    from litex.gen.fhdl.verilog import convert

    prefix = tool_prefix()
    if prefix is None:
        raise RuntimeError("Vivado's xsim is not available")
    os.makedirs(out_dir, exist_ok=True)
    from migen import ClockDomain
    top = TDMHarness(n_channels=n_channels, code_words=code_words)
    top.cd_sys = ClockDomain("sys")
    out = convert(top, ios=top.ios() | {top.cd_sys.clk, top.cd_sys.rst}, name="top")
    with open(os.path.join(out_dir, "top.v"), "w") as f:
        f.write(out.main_source)
    for fn, content in out.data_files.items():
        with open(os.path.join(out_dir, fn), "w") as f:
            f.write(content)
    with open(os.path.join(out_dir, "tb.v"), "w") as f:
        f.write(TESTBENCH)
    with open(os.path.join(out_dir, "stim.txt"), "w") as f:
        for k, (wv, wa, wd, sv, sw, si) in enumerate(scenario.cycles):
            rdy = 1 if ready_pattern is None else int(ready_pattern(k))
            f.write(f"{wv:x} {wa:x} {wd:x} {sv:x} {sw:x} {si:x} {rdy:x}\n")
    log = os.path.join(out_dir, "xsim.log")
    with open(log, "w") as lf:
        for cmd in (["xvlog", "top.v", "tb.v"], ["xelab", "tb", "-s", "tb_snap", "--debug", "off"],
                    ["xsim", "tb_snap", "-R"]):
            r = subprocess.run(prefix + cmd, cwd=out_dir, capture_output=True, text=True)
            lf.write(f"$ {' '.join(cmd)}\n{r.stdout}\n{r.stderr}\n")
            if r.returncode != 0:
                raise RuntimeError(f"{cmd[0]} failed; see {log}")
    words = []
    with open(os.path.join(out_dir, "out.txt")) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if "x" in line.lower() or "z" in line.lower():
                raise AssertionError(f"undefined bits in the DMA stream: {line}")
            words.append(int(line, 16))
    return words, out.main_source
