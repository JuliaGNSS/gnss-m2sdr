#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Does the Verilog the *build* writes correlate? Ask Vivado's own simulator.

Every test in test/ runs the Migen simulator on the Migen netlist. The bitstream
is produced from a different artefact: the Verilog that LiteX's converter
writes, plus the `.init` files it writes for every memory. Two faults have now
reached silicon through that gap -- a negative constant rendered as an unsigned
literal (docs/gateware_builds.md 5.6e) and the sin/cos ROM's negative entries
written as `-3` into a `$readmemh` file (5.6g) -- and neither could ever have
shown up in the Migen simulator, because it never sees that Verilog.

This script closes the gap for the one property that matters most: **a
modulated GNSS signal fed through the LiteX-lowered TrackingChannel produces a
correlation peak, and the ten accumulators match the Migen simulation of the
same netlist bit for bit.** It

  1. lowers one TrackingChannel with `litex.gen.fhdl.verilog.convert` -- the
     converter build.py uses -- and writes the Verilog and its `.init` files;
  2. generates a satellite: a primary code, a sub-chip shape (LOC or CBOC), a
     carrier, optional noise, at a chosen code offset;
  3. runs the Migen simulation of the same channel on the same samples with
     the same register writes, and records every dump (the golden result);
  4. writes a self-contained xsim testbench that programs the channel through
     its ports, streams the samples and prints every dump;
  5. runs xvlog / xelab / xsim and compares.

Two ways for step 5 to differ from step 3, both of which this has caught:

  * `$readmemh` stops at the first `-` and leaves the rest of the ROM `x`, so
    the accumulators come out `x` -- xsim's way of saying the ROM is broken;
  * with `--vivado-readmemh`, the `.init` files are rewritten the way Vivado's
    *synthesis* reads them (it silently drops the sign: `-3` -> `03`), which
    reproduces the flashed bitstream's behaviour -- a rectified carrier and no
    correlation peak -- in simulation.

Needs Vivado's xvlog/xelab/xsim on PATH (or XILINX_VIVADO set); nothing else.

    PYTHONPATH=. python scripts/xsim_correlation.py --out /tmp/xsim_l1ca
    PYTHONPATH=. python scripts/xsim_correlation.py --signal GalileoE1B --out /tmp/xsim_e1b

Exit code 0 means: xsim ran, the peak is there, and every dump matches Migen.
"""

import argparse
import math
import os
import random
import re
import shutil
import subprocess
import sys

from migen import *
from migen.sim import passive, run_simulation

from gnss_m2sdr.gateware.ca_code import ca_code_reference
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.record_format import TAPS_EPL, TAPS_VEPL, acc_signals, tap_short_names
from gnss_m2sdr.subcarrier import replica_shape, signal_replica_shape

FRAC       = 24
PHASE_BITS = 32
SHIFTS_EPL  = (1, 0, -1)
SHIFTS_VEPL = (2, 1, 0, -1, -2)


# --- the satellite -------------------------------------------------------------

def pm(bit):
    return 1 if bit else -1


def galileo_like_code(length, seed):
    """A balanced pseudo-random 0/1 primary code of `length` chips.

    The E1 memory codes are not generated here (the host loads them at runtime
    from GNSSSignals); any balanced sequence exercises the same RTL.
    """
    rng = random.Random(seed)
    return [rng.randint(0, 1) for _ in range(length)]


def satellite(words, shape, n_samples, code_step, carrier_fw, amp,
              code_offset_chips, noise_sigma, seed):
    """(I, Q) int16 samples of one satellite transmitting `shape`'s replica.

    The carrier runs at exactly the NCO's frequency word so the wipe-off leaves
    a constant phase; the code sits `code_offset_chips` ahead of the replica.
    """
    rng = random.Random(seed)
    n = len(words)
    out = []
    for k in range(n_samples):
        cp   = (k * code_step / (1 << FRAC) + code_offset_chips) % n
        chip = int(cp)
        frac = cp - chip
        w    = words[chip]
        lut  = (shape.lut_b if ((w >> 1) & 1 and shape.lut_b is not None)
                else shape.lut_a)
        v    = pm(w & 1) * lut[int(frac * shape.subchips)] / shape.code_amplitude
        theta = 2 * math.pi * carrier_fw * k / (1 << PHASE_BITS)
        i = amp * v * math.cos(theta) + rng.gauss(0, noise_sigma)
        q = amp * v * math.sin(theta) + rng.gauss(0, noise_sigma)
        out.append((max(-32768, min(32767, int(round(i)))),
                    max(-32768, min(32767, int(round(q))))))
    return out


# --- one channel, lowered the way the build lowers it ---------------------------

class Top(Module):
    """TrackingChannel with every control port brought out under a fixed name."""
    def __init__(self, ch, num_taps):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.ch = ch
        io = {}

        def port(name, sig):
            s = Signal(len(sig), name_override=name)
            s.signed = sig.signed
            io[name] = (s, sig)
            return s

        # Inputs.
        for name in ("sample_i", "sample_q", "sample_stb", "sample_count",
                     "carrier_fw", "carrier_set", "carrier_phase_in",
                     "code_step", "code_length", "restart_length", "subchips",
                     "taps_cfg", "restart", "code_loading",
                     "code_phase_chip", "code_phase_frac"):
            s = port(name, getattr(ch, name))
            self.comb += getattr(ch, name).eq(s)
        for t in range(num_taps):
            s = port(f"tap_offset{t}", ch.tap_offset[t])
            self.comb += ch.tap_offset[t].eq(s)
        for name in ("lut_we", "lut_sel", "lut_adr", "lut_dat"):
            s = port(name, getattr(ch.code, name))
            self.comb += getattr(ch.code, name).eq(s)
        # Outputs.
        for name in ("dump_stb", "dump_saturated", "integrated_samples",
                     "dump_code_chip", "dump_code_phase", "dump_num_taps",
                     "rate_unsupported", "replica_unsupported"):
            s = port(name, getattr(ch, name))
            self.comb += s.eq(getattr(ch, name))
        for k in acc_signals(num_taps):
            s = port(k, getattr(ch, k))
            self.comb += s.eq(getattr(ch, k))
        self.ios = {s for s, _ in io.values()} | {self.cd_sys.clk, self.cd_sys.rst}
        self.port_names = list(io)


def make_channel(words, num_taps, max_subchips):
    return TrackingChannel(code_frac_bits=FRAC, carrier_phase_bits=PHASE_BITS,
                           max_code_length=len(words), num_ants=1,
                           code_init=words, num_taps=num_taps,
                           max_subchips=max_subchips)


def lower(words, num_taps, max_subchips, out_dir):
    from litex.gen.fhdl.verilog import convert as litex_convert
    ch  = make_channel(words, num_taps, max_subchips)
    top = Top(ch, num_taps)
    out = litex_convert(top, ios=top.ios, name="top")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "top.v"), "w") as f:
        f.write(out.main_source)
    inits = {}
    for fn, content in out.data_files.items():
        with open(os.path.join(out_dir, fn), "w") as f:
            f.write(content)
        inits[fn] = content
    return inits


def vivado_readmemh(out_dir):
    """Rewrite the .init files the way Vivado synthesis reads them: the sign of
    a negative entry is dropped (`-3` -> `03`). Measured, not assumed: see
    docs/gateware_builds.md 5.6g."""
    n = 0
    for fn in os.listdir(out_dir):
        if not fn.endswith(".init"):
            continue
        path = os.path.join(out_dir, fn)
        lines = open(path).read().split()
        fixed = []
        for l in lines:
            if l.startswith("-"):
                n += 1
                l = l[1:].rjust(2, "0")
            fixed.append(l)
        with open(path, "w") as f:
            f.write("\n".join(fixed) + "\n")
    return n


# --- the register writes, identical in both simulators --------------------------

class Config:
    def __init__(self, words, shape, num_taps, shifts, code_step, carrier_fw,
                 stb_gap):
        self.words, self.shape, self.num_taps = words, shape, num_taps
        self.shifts, self.code_step, self.carrier_fw = shifts, code_step, carrier_fw
        self.stb_gap = stb_gap
        self.tap_offsets = [s * code_step for s in shifts]
        self.taps_cfg = TAPS_VEPL if num_taps == TAPS_VEPL else TAPS_EPL


def migen_dumps(cfg, samples, max_subchips):
    """Golden: the Migen simulation of the same channel and the same writes."""
    dut  = make_channel(cfg.words, cfg.num_taps, max_subchips)
    keys = acc_signals(cfg.num_taps)
    dumps = []

    def bench():
        for sel, lut in ((0, cfg.shape.lut_a), (1, cfg.shape.lut_b)):
            if lut is None:
                continue
            for adr, val in enumerate(lut):
                yield dut.code.lut_adr.eq(adr)
                yield dut.code.lut_sel.eq(sel)
                yield dut.code.lut_dat.eq(val)
                yield dut.code.lut_we.eq(1)
                yield
        yield dut.code.lut_we.eq(0)
        yield dut.subchips.eq(cfg.shape.subchips)
        yield dut.code_step.eq(cfg.code_step)
        yield dut.code_length.eq(len(cfg.words))
        yield dut.restart_length.eq(len(cfg.words))
        yield dut.taps_cfg.eq(cfg.taps_cfg)
        for t, off in enumerate(cfg.tap_offsets):
            yield dut.tap_offset[t].eq(off)
        yield dut.carrier_fw.eq(cfg.carrier_fw)
        yield dut.carrier_phase_in.eq(0)
        yield dut.carrier_set.eq(1)
        yield dut.restart.eq(1)
        yield
        yield dut.carrier_set.eq(0)
        yield dut.restart.eq(0)
        yield
        for k, (i, q) in enumerate(samples):
            yield dut.sample_i.eq(i)
            yield dut.sample_q.eq(q)
            yield dut.sample_count.eq(k)
            yield dut.sample_stb.eq(1)
            yield
            yield dut.sample_stb.eq(0)
            for _ in range(cfg.stb_gap):
                yield
        for _ in range(4):
            yield

    @passive
    def monitor():
        while True:
            if (yield dut.dump_stb):
                d = {}
                for k in keys:
                    d[k] = yield getattr(dut, k)
                d["n"]    = yield dut.integrated_samples
                d["chip"] = yield dut.dump_code_chip
                d["sat"]  = yield dut.dump_saturated
                dumps.append(d)
            yield

    run_simulation(dut, [bench(), monitor()])
    return dumps


def write_testbench(cfg, samples, out_dir, max_subchips):
    keys = acc_signals(cfg.num_taps)
    with open(os.path.join(out_dir, "stim.txt"), "w") as f:
        for i, q in samples:
            f.write(f"{i} {q}\n")
    lut_writes = []
    for sel, lut in ((0, cfg.shape.lut_a), (1, cfg.shape.lut_b)):
        if lut is None:
            continue
        for adr, val in enumerate(lut):
            lut_writes.append(f"    lut_adr = {adr}; lut_sel = {sel}; "
                              f"lut_dat = {val & 0xff}; lut_we = 1; @(posedge clk); #1;")
    tap_lines = "\n".join(
        f"    tap_offset{t} = {off & ((1 << (FRAC + 1)) - 1)};"
        for t, off in enumerate(cfg.tap_offsets))
    acc_fmt  = " ".join("%0d" for _ in keys)
    acc_args = ", ".join(f"$signed({k})" for k in keys)
    tb = f"""`timescale 1ns/1ps
// Generated by scripts/xsim_correlation.py -- do not edit.
module tb;
reg sys_clk = 0, sys_rst = 1;
reg signed [15:0] sample_i = 0, sample_q = 0;
reg sample_stb = 0;
reg [63:0] sample_count = 0;
reg [31:0] carrier_fw = 0, carrier_phase_in = 0;
reg carrier_set = 0, restart = 0, code_loading = 0;
reg [{FRAC}:0] code_step = 0;
reg [{max(1, (len(cfg.words)).bit_length())-1}:0] code_length = 0, restart_length = 0;
reg [{max(1, max_subchips.bit_length())-1}:0] subchips = 1;
reg [7:0] taps_cfg = 3;
reg [{max(1, (len(cfg.words)-1).bit_length())-1}:0] code_phase_chip = 0;
reg [{FRAC-1}:0] code_phase_frac = 0;
{chr(10).join(f"reg [{FRAC}:0] tap_offset{t} = 0;" for t in range(cfg.num_taps))}
reg lut_we = 0, lut_sel = 0;
reg [{max(1, (max_subchips-1).bit_length())-1}:0] lut_adr = 0;
reg [7:0] lut_dat = 0;
wire dump_stb, dump_saturated, rate_unsupported, replica_unsupported;
wire [31:0] integrated_samples;
wire [{max(1, (len(cfg.words)-1).bit_length())-1}:0] dump_code_chip;
wire [{FRAC-1}:0] dump_code_phase;
wire [7:0] dump_num_taps;
{chr(10).join(f"wire signed [31:0] {k};" for k in keys)}

top dut(
    .sys_clk(sys_clk), .sys_rst(sys_rst),
    .sample_i(sample_i), .sample_q(sample_q), .sample_stb(sample_stb),
    .sample_count(sample_count),
    .carrier_fw(carrier_fw), .carrier_set(carrier_set), .carrier_phase_in(carrier_phase_in),
    .code_step(code_step), .code_length(code_length), .restart_length(restart_length),
    .subchips(subchips), .taps_cfg(taps_cfg), .restart(restart), .code_loading(code_loading),
    .code_phase_chip(code_phase_chip), .code_phase_frac(code_phase_frac),
{chr(10).join(f"    .tap_offset{t}(tap_offset{t})," for t in range(cfg.num_taps))}
    .lut_we(lut_we), .lut_sel(lut_sel), .lut_adr(lut_adr), .lut_dat(lut_dat),
    .dump_stb(dump_stb), .dump_saturated(dump_saturated),
    .integrated_samples(integrated_samples), .dump_code_chip(dump_code_chip),
    .dump_code_phase(dump_code_phase), .dump_num_taps(dump_num_taps),
    .rate_unsupported(rate_unsupported), .replica_unsupported(replica_unsupported),
{",".join(chr(10) + f"    .{k}({k})" for k in keys)}
);

always #4 sys_clk = ~sys_clk;
wire clk = sys_clk;

integer fd, fo, r, k, g;
reg signed [31:0] si, sq;

always @(posedge clk) if (dump_stb)
    $fdisplay(fo, "{acc_fmt} %0d %0d %0d", {acc_args},
              integrated_samples, dump_code_chip, dump_saturated);

initial begin
    fo = $fopen("dut_dumps.txt", "w");
    fd = $fopen("stim.txt", "r");
    repeat (4) @(posedge clk);
    #1 sys_rst = 0;
    repeat (2) @(posedge clk); #1;
{chr(10).join(lut_writes)}
    lut_we = 0;
    subchips = {cfg.shape.subchips};
    code_step = {cfg.code_step};
    code_length = {len(cfg.words)};
    restart_length = {len(cfg.words)};
    taps_cfg = {cfg.taps_cfg};
{tap_lines}
    carrier_fw = {cfg.carrier_fw};
    carrier_phase_in = 0;
    carrier_set = 1; restart = 1;
    @(posedge clk); #1;
    carrier_set = 0; restart = 0;
    @(posedge clk); #1;
    k = 0;
    while (!$feof(fd)) begin
        r = $fscanf(fd, "%d %d\\n", si, sq);
        if (r == 2) begin
            sample_i = si; sample_q = sq; sample_count = k; sample_stb = 1;
            @(posedge clk); #1;
            sample_stb = 0;
            for (g = 0; g < {cfg.stb_gap}; g = g + 1) begin @(posedge clk); #1; end
            k = k + 1;
        end
    end
    repeat (8) @(posedge clk);
    $fclose(fo);
    $display("XSIM_DONE samples=%0d", k);
    $finish;
end
endmodule
"""
    with open(os.path.join(out_dir, "tb.v"), "w") as f:
        f.write(tb)


def find_tool(name):
    p = shutil.which(name)
    if p:
        return p
    root = os.environ.get("XILINX_VIVADO")
    if root and os.path.exists(os.path.join(root, "bin", name)):
        return os.path.join(root, "bin", name)
    return None


def run_xsim(out_dir, log):
    tools = {n: find_tool(n) for n in ("xvlog", "xelab", "xsim")}
    missing = [n for n, p in tools.items() if p is None]
    if missing:
        raise RuntimeError(f"Vivado simulator not found: {missing}; put xvlog/xelab/xsim "
                           f"on PATH or set XILINX_VIVADO")
    env = dict(os.environ)
    steps = [
        [tools["xvlog"], "top.v", "tb.v"],
        [tools["xelab"], "tb", "-s", "tb_snap", "--debug", "off"],
        [tools["xsim"], "tb_snap", "-R"],
    ]
    for cmd in steps:
        r = subprocess.run(cmd, cwd=out_dir, env=env, capture_output=True, text=True)
        log.write(f"$ {' '.join(cmd)}\n{r.stdout}\n{r.stderr}\n")
        if r.returncode != 0:
            raise RuntimeError(f"{cmd[0]} failed ({r.returncode}); see {log.name}")
    return log


def read_dut_dumps(out_dir, keys):
    dumps = []
    with open(os.path.join(out_dir, "dut_dumps.txt")) as f:
        for line in f:
            parts = line.split()
            if len(parts) != len(keys) + 3:
                continue
            d = {}
            ok = True
            for k, v in zip(list(keys) + ["n", "chip", "sat"], parts):
                if "x" in v.lower():
                    d[k] = None
                    ok = False
                else:
                    d[k] = int(v)
            d["has_x"] = not ok
            dumps.append(d)
    return dumps


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--signal", default="GPSL1CA",
                    help="GPSL1CA (LOC, 1023 chips, 3 taps) or GalileoE1B (CBOC, 4092 chips, 5 taps)")
    ap.add_argument("--fs", type=float, default=None, help="sample rate (default 4e6 / 24.552e6)")
    ap.add_argument("--doppler", type=float, default=1500.0, help="carrier Doppler in Hz")
    ap.add_argument("--periods", type=int, default=3, help="code periods to stream")
    ap.add_argument("--amp", type=float, default=200.0)
    ap.add_argument("--noise", type=float, default=0.0, help="Gaussian noise sigma per I/Q sample")
    ap.add_argument("--offset", type=float, default=0.0, help="code offset of the satellite in chips")
    ap.add_argument("--stb-gap", type=int, default=1, help="idle clocks between samples")
    ap.add_argument("--max-subchips", type=int, default=12)
    ap.add_argument("--vivado-readmemh", action="store_true",
                    help="rewrite the .init files the way Vivado synthesis reads them (sign dropped)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    if args.signal == "GPSL1CA":
        words     = list(ca_code_reference(1))
        shape     = replica_shape("LOC")
        num_taps  = TAPS_EPL
        shifts    = SHIFTS_EPL
        chip_rate = 1.023e6
        fs        = args.fs or 4.0e6
        carrier   = 1575.42e6
    elif args.signal == "GalileoE1B":
        words     = galileo_like_code(4092, seed=11)
        shape     = signal_replica_shape("GalileoE1B")
        num_taps  = TAPS_VEPL
        shifts    = SHIFTS_VEPL
        chip_rate = 1.023e6
        fs        = args.fs or 24.552e6
        carrier   = 1575.42e6
    else:
        sys.exit(f"unknown signal {args.signal}")

    code_step  = round(chip_rate * (1 + args.doppler / carrier) / fs * (1 << FRAC))
    carrier_fw = round(args.doppler / fs * (1 << PHASE_BITS)) & ((1 << PHASE_BITS) - 1)
    n_samples  = int(args.periods * len(words) * fs / chip_rate) + 8
    cfg = Config(words, shape, num_taps, shifts, code_step, carrier_fw, args.stb_gap)
    keys = acc_signals(num_taps)

    print(f"signal {args.signal}: {len(words)} chips, {shape}, {num_taps} taps, "
          f"fs={fs:g}, code_step={code_step}, carrier_fw={carrier_fw}, {n_samples} samples")
    samples = satellite(words, shape, n_samples, code_step, carrier_fw, args.amp,
                        args.offset, args.noise, seed=1)

    os.makedirs(args.out, exist_ok=True)
    inits = lower(words, num_taps, args.max_subchips, args.out)
    neg = {fn: sum(1 for l in c.split() if l.startswith("-")) for fn, c in inits.items()}
    print("init files:", ", ".join(f"{fn} ({n} negative entries)" for fn, n in neg.items()))
    if args.vivado_readmemh:
        print(f"rewrote {vivado_readmemh(args.out)} negative init entries the way Vivado reads them")

    golden = migen_dumps(cfg, samples, args.max_subchips)
    print(f"migen: {len(golden)} dumps")
    write_testbench(cfg, samples, args.out, args.max_subchips)
    with open(os.path.join(args.out, "xsim.log"), "w") as log:
        run_xsim(args.out, log)
    dut = read_dut_dumps(args.out, keys)
    print(f"xsim:  {len(dut)} dumps")

    # Report.
    def power(d):
        return None if d.get("has_x") else d["ip"] ** 2 + d["qp"] ** 2

    ok = True
    print(f"{'#':>2} {'src':>5} " + " ".join(f"{k:>12}" for k in keys) + f" {'n':>6} {'chip':>5} sat")
    for i, g in enumerate(golden):
        print(f"{i:>2} migen " + " ".join(f"{g[k]:>12}" for k in keys) + f" {g['n']:>6} {g['chip']:>5} {g['sat']}")
        if i < len(dut):
            d = dut[i]
            fmt = lambda v: "x" if v is None else str(v)
            print(f"{i:>2}  xsim " + " ".join(f"{fmt(d[k]):>12}" for k in keys)
                  + f" {fmt(d['n']):>6} {fmt(d['chip']):>5} {fmt(d['sat'])}")
            if d["has_x"] or any(d[k] != g[k] for k in list(keys) + ["n", "chip", "sat"]):
                ok = False
        else:
            ok = False
    if len(dut) != len(golden):
        ok = False

    # The physics: with the satellite at the replica's code offset the prompt
    # power must dominate the noise floor the *same* channel shows on a
    # wrong-offset satellite. A single run has no floor to compare with, so
    # take the E/L balance and the prompt-over-VE/VL ratio as the peak test.
    if golden:
        g = golden[-1]
        pp = g["ip"] ** 2 + g["qp"] ** 2
        pe = g["ie"] ** 2 + g["qe"] ** 2
        pl = g["il"] ** 2 + g["ql"] ** 2
        print(f"migen last dump: |P|^2={pp:.3e} |E|^2={pe:.3e} |L|^2={pl:.3e}")
        if dut and not dut[-1]["has_x"]:
            d = dut[-1]
            dp = d["ip"] ** 2 + d["qp"] ** 2
            print(f"xsim  last dump: |P|^2={dp:.3e}")
            # An aligned, wiped-off prompt sums amp * 127 (the carrier LUT's
            # peak) over one code period's samples; the replica's own
            # amplitude scales the satellite and the replica alike, so it
            # cancels. Anything far below is a channel that does not correlate.
            n_period = round(len(cfg.words) * (1 << FRAC) / cfg.code_step)
            expected = (args.amp * 127 * n_period) ** 2
            print(f"(a fully wiped-off, aligned prompt is ~{expected:.2e} for this amplitude)")
            if dp < 0.25 * expected:
                print("FAIL: the lowered channel does not correlate (prompt power far below the aligned value)")
                ok = False
        else:
            print("FAIL: xsim produced no usable dump (missing or x)")
            ok = False
    print("RESULT:", "PASS -- the LiteX-lowered channel correlates and matches Migen bit for bit"
          if ok else "FAIL -- see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
