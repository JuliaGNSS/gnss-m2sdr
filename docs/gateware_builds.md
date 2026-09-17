# Gateware builds: measured utilisation and timing

Steps 2–5 sized the tracking bank analytically and said so plainly, because this
repository's CI is board-free and has no Vivado. This page is the other half:
what Vivado actually reports for a real build, on the real part.

Everything here is a **measurement**, copied from the reports the build writes.
Where a number is derived rather than read off a report, it says so.

Part: **XC7A200T-3** (`xc7a200tsbg484-3`), Vivado **2024.1**, `sys_clk` **125 MHz**
(8.000 ns), `litex_m2sdr` at `b10dc4d` (the revision the timing-clean v1 release
was built against).

## 1. Build index

| Build | Taps | Sub-chips | Code chips | Ants | Timing | Flashed |
|---|---:|---:|---:|---:|---|---|
| `RELEASE_ch4_ant1_timing_clean` (v1, 2026-07-28) | 3 | 1 | 1023 | 1 | **WNS +0.005 ns** — met | yes, currently on the board |
| `gnss_m2sdr_m2_x1_ch4_ant2_code10230_tap5_sub12` | 5 | 12 | 10230 | 2 | **WNS −2.875 ns** — *not met* | **no** |

The v1 reference closed with **5 ps** of margin over 73 880 endpoints. That is
the context for everything below: this design was already at the edge of the
part before steps 3–5 added to it.

## 2. Full-coverage v3 build — does not close timing

```
python build.py --channels 4 --num-ants 2 --max-code-length 10230 \
                --taps 5 --max-subchips 12 --build
```

### Utilisation (`*_utilization_place.rpt`, design fully placed)

| Resource | Used | Available | Util% |
|---|---:|---:|---:|
| Slice LUTs | 33 185 | 133 800 | 24.80% |
| — LUT as Logic | 22 769 | 133 800 | 17.02% |
| — LUT as Memory | 10 416 | 46 200 | 22.55% |
| —— LUT as Distributed RAM | 10 412 | | |
| Slice Registers (all FFs) | 25 037 | 267 600 | 9.36% |
| Slice (occupied) | 12 692 | 33 450 | 37.94% |
| — SLICEM | 5 039 | | |
| F7 Muxes | 4 796 | 66 900 | 7.17% |
| F8 Muxes | 159 | 33 450 | 0.48% |
| Block RAM Tile | 48 | 365 | 13.15% |
| DSP48E1 | 112 | 740 | 15.14% |

Distributed-memory primitives: 9 616 `RAMD64E`, 1 140 `RAMD32`, 328 `RAMS32`.

**The part is not full.** Nothing here is close to a capacity limit — the
blocker is timing, not space.

### Measured vs. step 5's analytic estimate

Step 5 ([sub-chip modulation](subchip_modulation.md) §6) quoted no utilisation
figures at all, only an analytic cost. Comparing like for like:

| Quantity | Step 5 estimate | Measured | Verdict |
|---|---|---|---|
| Code memory, 4 ch @ 10230 chips | ≈246 kbit distributed RAM (245 520 logical bits) | 10 412 LUTs as distributed RAM | **consistent** — see below |
| Multiplies, 4-channel bank | 132 (33/channel) | 112 DSP48E1 *for the whole SoC* | **fewer DSPs than multiplies**, as expected |
| Subcarrier tables | 192 bits/channel, in registers not RAM | not separately visible (flat netlist) | not measurable |
| Throughput | unchanged, 128-byte record | unchanged — no datapath change | holds |

*Code memory.* The estimate counts **logical bits**; the report counts **LUTs**.
Reconciling: 245 520 logical bits, as a 1-write/1-async-read distributed RAM,
costs ~2× LUT capacity (a mirrored array per independent read port), i.e.
≈491 kbit ≈ 7 673 LUT6 at 64 bits each. Measured 10 412, the remaining ~2 700
being the recorder FIFO, CSR storage and the rest of the M2SDR SoC. The estimate
was sound; it was simply denominated in bits, and a LUT is not a bit.

*Multiplies.* 132 was a count of **multiply operations**, not DSP blocks. The
measured 112 DSP48E1 is for the entire SoC including the AD9361 datapath, so the
bank uses fewer DSPs than it has multiplies — narrow and constant-coefficient
products map to LUTs. The estimate over-predicts DSP pressure, which is the safe
direction.

*Where the estimate was silent.* It said "throughput is unchanged" and gave no
clock period. That is the gap this build closes: the datapath is indeed no
*longer* in cycles, but it became much *deeper* in combinational logic, and
that is what fails.

### Timing (`*_timing.rpt`, Design State: Physopt postRoute)

```
WNS(ns)      TNS(ns)  TNS Failing Endpoints  TNS Total Endpoints   WHS(ns)   THS(ns)
 -2.875  -17020.270                  18685               167097     0.006      0.000
```

Setup fails. **Hold is clean** (WHS +0.006 ns) and pulse width is clean
(WPWS +0.047 ns). Routing itself succeeded: 0 failed nets, 0 unrouted nets.

Per clock — the failure is confined to one domain:

| Clock | Period | WNS | Failing endpoints |
|---|---:|---:|---:|
| `crg_s7pll_clkout0` (**sys_clk**) | 8.000 ns | **−2.875** | 18 685 / 159 326 |
| `rfic_clk` | 4.069 ns | +0.015 | 0 / 2 000 |
| `clk100` | 10.000 ns | +8.784 | 0 |
| `si5351_clk1` | 10.000 ns | +2.280 | 0 |
| every other clock | | positive | 0 |

`rfic_clk` at 245.76 MHz meets with 15 ps to spare — tight, but met and
unchanged by this work.

### The critical path

Every one of the ten worst paths has the same source and the same shape:

```
Source:      gnsstracking_channelwithcsr0_code_length_act_reg[1]/C     (FDSE)
Destination: gnsstracking_channelwithcsr0_trackingchannel0_raw3/B[14]  (DSP48E1)
Requirement:      8.000 ns
Data Path Delay: 10.458 ns  (logic 3.513 ns 33.6%, route 6.945 ns 66.4%)
Logic Levels:    17  (CARRY4=4 LUT1=1 LUT2=1 LUT3=1 LUT4=1 LUT5=1 LUT6=5 MUXF7=2 RAMD64E=1)
```

Read left to right, this is: the **runtime `code_length` register** (step 4) →
the code-phase wrap comparison and its carry chain (`CARRY4`×4) → the code RAM
address → the **`max_code_length`-deep distributed-RAM read** (`RAMD64E` plus a
`MUXF7` output mux tree, both of which get deeper as `max_code_length` grows) →
the replica/amplitude logic → the **`B` input of the correlator's DSP48E1**.

All of it is combinational, in one 8 ns cycle. Three separate features compound
on this path:

1. **step 4** made `code_length` a runtime register feeding the wrap arithmetic;
2. **step 2/4** made the code RAM `max_code_length` deep — at 10 230 chips the
   distributed-RAM read is a large asynchronous mux tree (hence 4 796 F7 muxes);
3. **step 5** put an amplitude-bearing replica multiply at the end of it.

Note that the path *source* is a step-4 register and the deep RAM is a step-2/4
sizing choice — this is not solely step 5's doing, it is the accumulation.

[signal configuration](signal_configuration.md) §2 already flagged the mechanism:
*"the read is asynchronous, so this is LUTRAM, not block RAM … using [block RAM]
would need a pipeline stage between the chip-index arithmetic and the tap mux
that the channel does not have today."* That missing pipeline stage is exactly
what this build ran out of.

Route delay is 66% of the path, so placement is fighting it too, but 17 logic
levels at 125 MHz is the primary problem: no placement fixes a path that deep.

## 3. What this means

- **This bitstream must not be flashed.** A design that misses setup by 2.9 ns
  does not "mostly work" — it produces wrong correlator results
  non-deterministically, which is worse than not flashing at all.
- The board therefore still runs the **2026-07-29 v1** gateware, and every
  hardware acceptance item across steps 3, 5 and 7 remains untested.
- The fix is a **pipeline stage**, not a smaller build: register the code RAM
  read (or the replica product) and add a cycle to the channel's pipeline. The
  bank is a non-intrusive observer with no back-pressure, so an extra cycle of
  latency costs a constant offset in `sample_index`, not throughput.

## 4. Reproducing a build in this sandbox

Vivado 2024.1 lives at `/opt/Xilinx`, but the image is Nix-built and minimal.
Use the prepared launcher, which supplies `libcrypt.so.1`, `libz.so.1` and a
locale archive:

```bash
export PATH=/root/Code/.vivado-fhs/bin:$PATH
```

Two further things are needed, and neither is obvious from the error messages:

1. **Vivado's own compat libraries.** `bin/ldlibpath.sh` picks a per-distro lib
   directory by reading `/etc/os-release`, which this image does not have, so it
   falls through to `Default` and never adds the `libtinfo.so.5` it ships
   itself. The failure looks like
   `couldn't load file "librdi_commontasks.so": libtinfo.so.5: cannot open shared object file`.
   Add the bundled directory rather than a host library:

   ```bash
   export LD_LIBRARY_PATH=/opt/Xilinx/Vivado/2024.1/lib/lnx64.o/Rhel/9
   ```

2. **X11.** Even `-mode batch` loads `libX11.so.6`; point `LD_LIBRARY_PATH` at
   the image's own X libraries too. Do **not** copy host libraries in blindly —
   the host `libncursesw` needs GLIBC_2.42 while the image has 2.40, and pulling
   it in breaks bash itself.

### The zombie deadlock

Vivado will **hang indefinitely** immediately after
`Synthesis finished with 0 errors`, with every thread in `futex_wait` and no CPU
being consumed.

Cause: `synth_design` spawns parallel worker processes which are orphaned to
PID 1 when their intermediate parent exits. This container's PID 1 is
`sleep infinity`, which never calls `wait()`, so the finished workers stay as
zombies forever — and a zombie still answers `kill(pid, 0)`, so Vivado's
liveness poll never sees them exit.

Fix: run the build under a process that sets `PR_SET_CHILD_SUBREAPER` and reaps
adoptees, so orphaned workers are collected instead of accumulating. Any small
init-style wrapper (`tini`, or a ~20-line Python parent that sets the prctl and
loops on `os.waitpid(-1, 0)`) works. Confirm the fix by checking that no new
`vivado <defunct>` entries appear with `ppid=1` during a run.

Do not "fix" this by lowering `general.maxThreads` unless you have to; reaping
is the actual bug and single-threaded synthesis is much slower.
