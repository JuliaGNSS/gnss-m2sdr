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
| `RELEASE_ch4_ant1_timing_clean` (v1, 2026-07-28) | 3 | 1 | 1023 | 1 | **WNS +0.005 ns** — met | no (see §5) |
| `gnss_m2sdr_m2_x1_ch4_ant2_code10230_tap5_sub12` (as of #32) | 5 | 12 | 10230 | 2 | **WNS −2.875 ns** — *not met* | **no** |
| `gnss_m2sdr_m2_x1_ch4_ant2_code1023_tap5_sub12` (isolation, §2b) | 5 | 12 | 1023 | 2 | **WNS −1.288 ns** — *not met* | **no** |
| same, after the pipeline fixes of §5 | 5 | 12 | 10230 | 2 | **WNS −0.788 ns** — *not met* | no |
| `gnss_m2sdr_m2_x1_ch4_ant1_code4092_tap5_sub12` (§5.6) | 5 | 12 | 4092 | 1 | **WNS +0.015 ns** — met | **flashed, then rolled back** (§5.6b) |
| same RTL + #42 clamp fix, `--timing-effort max` (§5.6f) | 5 | 12 | 4092 | 1 | **WNS +0.012 ns** — met | flashed, did not correlate, rolled back (§5.6f) |
| `…_code4092_tap5_sub12` with the carrier-ROM fix, `max` (§5.9) | 5 | 12 | 4092 | 1 | WNS −0.069 ns — *not met* (14 AD9361 BFP endpoints) | no |
| same, `max` + `--directive place=ExtraPostPlacementOpt` (§5.9) | 5 | 12 | 4092 | 1 | WNS −0.115 ns — *not met* | no |
| same, `max` + `--directive synth=PerformanceOptimized` (§5.9) | 5 | 12 | 4092 | 1 | **WNS +0.000 ns** — met | no |
| `…_code4092_tap5_sub12_placeSpread`: `max` + `--directive place=AltSpreadLogic_high` (§5.9) | 5 | 12 | 4092 | 1 | **WNS +0.003 ns** — met | **flashed 2026-09-18** (§5.9) |

The v1 reference closed with **5 ps** of margin over 73 880 endpoints. That is
the context for everything below: this design was already at the edge of the
part before steps 3–5 added to it.

**What is on the board today** is the last row: the carrier-ROM fix at
`--directive place=AltSpreadLogic_high`, flashed 2026-09-18 10:00 UTC; §5.9
has what it does on sky. Before that it was a **20-channel** build
(`gnss_m2sdr_m2_x1_ch20_ant1`, SoC identifier *built on 2026-07-29 23:42:50*),
which is itself **WNS −0.181 ns** over 507 endpoints. §5.7 has the evidence and
the rollback command. The timing-clean v3 build *was* flashed on 2026-09-17; it
reports its capabilities correctly and runs its NCOs correctly, but its
correlators do not respond to the code RAM, so GPS L1 C/A did not acquire and
the board was restored to the image above. §5.6b is the full measurement.

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

## 2b. Isolating the cause: is it just the code depth?

Before any pipelining, one question had to be answered: is the failure *the
10 230-chip code RAM being too deep*, or *the five-tap sub-chip channel being
too deep*? That decides whether a smaller build is a usable workaround at all,
and the whole all-signal chain was blocked on it. So the identical build was
repeated with only `--max-code-length` reduced:

```
python build.py --channels 4 --num-ants 2 --max-code-length 1023 \
                --taps 5 --max-subchips 12 --build
```

| | 10230 chips | 1023 chips |
|---|---:|---:|
| WNS | −2.875 ns | **−1.288 ns** |
| TNS | −17 020.270 ns | −2 431.957 ns |
| Failing endpoints | 18 685 / 167 097 | 4 572 / 103 169 |
| Slice LUTs | 33 185 (24.80%) | 24 816 (18.55%) |
| LUT as Memory | 10 416 (22.55%) | 3 442 (7.45%) |
| F7 Muxes | 4 796 | 1 421 |
| DSP48E1 | 112 | 112 |

The deep code RAM is worth about **1.59 ns** of the 2.875 ns — a large share,
and exactly where the collapse in LUT-as-memory and F7 muxes says it should be.
**But the shorter build still misses by 1.288 ns**, on a path that has moved off
the code RAM entirely:

```
Source:      litepciedma0_buffering_syncfifo1_readable_reg/C             (FDRE)
Destination: gnsstracking_channelwithcsr2_trackingchannel232_reg/PCIN[0] (DSP48E1)
Data Path Delay: 8.234 ns  (logic 5.121 ns 62.2%, route 3.113 ns 37.8%)
Logic Levels:    10  (CARRY4=5 DSP48E1=1 LUT2=1 LUT4=1 LUT6=2)
```

That is LitePCIe's DMA0 writer FIFO level driving `rx_stream.ready`, through the
RX observer's output mux, into the correlator **accumulator** DSP48E1 cascade —
*logic*-dominated at 62%, and untouched by the code length.

(One caveat, recorded because it was measured: in the 1023-chip build `rfic_clk`
also shows −0.245 ns over 76 endpoints, where the 10 230-chip build met it at
+0.015 ns. Both are near zero and the domain is unrelated to this work, so it is
most likely placement variance rather than a real difference.)

### What this experiment established, and how §5 used it

Two conclusions, and they are the two halves of the eventual fix rather than
opposing claims:

1. **Reducing the configuration alone cannot close this.** At 1023 chips — a
   16:1 LUTRAM output mux instead of 160:1, the most aggressive reduction
   available — the design still missed by 1.288 ns. Anyone hoping to flash a
   smaller v3 build without touching the RTL would have burned a day finding
   that out.
2. **The code RAM depth, not the tap depth, is the expensive half.** 1.59 ns of
   2.875 ns came off for the code length alone, while `DSP48E1` stayed at 112 —
   five taps cost nothing extra in multipliers, exactly as designed.

§5 closed timing with *both* halves: the four pipeline stages **and** a reduced
configuration (4092 chips, one antenna). Neither would have done it alone. The
second path this experiment exposed — the accumulator's DSP48E1 cascade — was
not pipelined at all; it went away because dropping to one antenna halved those
cascades, which is why §5.6 calls the second antenna the expensive concession
rather than the code length.

## 3. What this means

- **That bitstream must not be flashed.** A design that misses setup by 2.9 ns
  does not "mostly work" — it produces wrong correlator results
  non-deterministically, which is worse than not flashing at all.
- The fix is a **pipeline stage**, not a smaller build: register the code RAM
  read (or the replica product). It does not even have to cost a cycle of
  latency — see §5.

A shorter code does **not** rescue it on its own — §2b measured that
directly: the same configuration at `--max-code-length 1023` still misses, at
**WNS −1.288 ns**, on a different path. Two independent paths were over budget,
so trimming the configuration was never going to be enough by itself. It was
still necessary: §5 needed the pipeline stages *and* a reduced build.

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

## 5. Closing it: four pipeline stages, no extra latency

Four paths were over budget, and they had to be fixed in that order because each
one hid the next. Pipelining alone was not enough — §2b had already shown that
reducing the configuration alone was not either, and closure needed both — but
none of these stages cost a cycle of correlator latency: every stage below is computed from a
value that is *already known one cycle early*, so the sample it describes still
arrives on the sample it belongs to. The board-free suite is the check that this
is true rather than merely intended.

All of these are `--channels 4 --num-ants 2 --max-code-length 10230 --taps 5
--max-subchips 12` unless the row says otherwise.

| Iteration | Fix | WNS | TNS | Failing endpoints |
|---|---|---:|---:|---:|
| 0 (as of #32) | — | −2.875 ns | −17 020.3 ns | 18 685 / 167 097 |
| 1 | registered chip window + registered sample stream | −2.309 ns | −5 966.6 ns | 10 808 / 144 714 |
| 2 | + registered `apply_at` comparison | −0.788 ns | −513.9 ns | 1 371 / 144 571 |
| 3 | + registered tap chip/sub-chip selection, from the *next* phase | −1.733 ns | −955.5 ns | 2 892 / 167 727 |
| 4 | + the strobe moved off that datapath onto the register enables | −2.034 ns | −941.8 ns | 2 955 / 168 264 |
| 5 | same RTL at `--max-code-length 4092` | −0.660 ns | −162.4 ns | 637 / 125 416 |
| 6 | + staged restart length, at `--max-code-length 4092 --num-ants 1` | −0.262 ns | −5.3 ns | 43 / 109 813 |
| 7 | same, `--timing-effort high` | **+0.015 ns** | **0.000 ns** | **0 / 109 810** |

Iterations 3 and 4 look like a step backwards and are: with the replica
arithmetic registered, the binding path moved onto `restart` →
`code_length − 1` → the code-RAM address, which at 10 230 chips runs straight
into the 160:1 LUTRAM output mux. **That read is the floor.** Address to
registered output is ~5 ns of an 8 ns cycle on its own at 10 230 chips, so no
amount of arithmetic trimming in front of it closes that configuration — which
is why iteration 5 changes the build rather than the RTL. At 4092 the same tree
is 64:1 and the whole path fits.

### 5.1 The registered chip window (`code_replica.py`)

The code RAM's three read outputs become flops. The index only ever moves one
chip, so an advance *shifts* the three words and pulls in exactly one new one --
the chip two ahead, which the far read port has been addressed at since the
previous advance. A `restart` is the only jump, and the other two ports exist
for it alone: they sit permanently on the chips either side of `restart_chip`,
so a rebase reloads all three in the restart cycle itself.

Removed from the per-sample path: `code_length`'s wrap arithmetic, the code-RAM
address arithmetic, the LUTRAM read and its 160:1 output mux tree.

It also made the design *smaller*: LUT-as-memory fell from 10 416 to 7 806 and
F7 muxes from 4 796 to 3 651, because the read ports no longer need per-address
wrap logic in front of them.

### 5.2 The registered sample stream (`rx_observer.py`, `soc.py`)

`RXSampleObserver.sample_stb` is `rx_stream.valid & rx_stream.ready`, and on the
SoC `ready` comes combinationally out of LitePCIe's DMA0 writer -- a FIFO-level
comparator and its carry chain. It gates the observer's output mux, which lands
on the A input of the carrier wipe-off DSP48E1, whose A→PCOUT cascade is another
2.97 ns. `SampleStreamRegister` puts one flop stage between the observer and the
bank: a pure delay of samples, strobe and antenna count *together*, and the bank
counts the strobes it is handed, so no record, sample index or code phase moves.

This is the path that made the `--max-code-length 1023` build miss by 1.288 ns,
i.e. the one a smaller configuration would never have fixed.

### 5.3 The registered `apply_at` comparison (`bank.py`)

With the chip window registered, `restart` began selecting a code-RAM read
address -- and `restart_pulse` includes the scheduled-commit strobe, which was
`(sample_count + 1) >= apply_at` computed live on 64 bits: an increment and a
compare, ~30 carry stages, 26 logic levels from `sample_count` to a window flop.

`reached` now holds that comparison in a flop, evaluated against the value the
counter will hold *next* cycle (`count_stb` says whether it advances), so on any
strobe it already describes that strobe's sample. The only staleness is an
`apply_at` write in the immediately preceding cycle, which cannot race a commit:
arming needs a 0→1 edge on `apply.arm` after the write, and `armed` only comes
up the cycle after that.

### 5.4 The registered tap chip/sub-chip selection (`code_replica.py`)

What was left was the replica arithmetic itself:
`code_frac` → per-tap phase add → `floor(sub × subchips)` → subcarrier table mux
→ sign mux → DSP48E1 `B`. The sub-chip index is a 24×4 multiply whose *top* bits
are wanted, so every carry in it is on the path; 16 logic levels, 8.493 ns of an
8.000 ns cycle.

Both of the choices a tap makes -- *which* chip word and *which* sub-chip of it
-- are now flops, computed from the phase and window the taps will see next
cycle. Only the two amplitude muxes are left in front of the DSP. Again no
latency: `code_frac` and the window are registers whose next value is known this
cycle.

The one behavioural change in the whole set: a `tap_offset` or `subchips` write
reaches the replica on the next strobe or `restart` rather than on the next
cycle. Both are programmed before the arming `restart`, which is itself a cycle
those registers load on, so the first sample of an integration already has them.

### 5.4b The staged restart length (`bank.py`, `code_replica.py`)

`restart` reloads the whole chip window, and where the chips either side of
`code_phase.chip` *are* depends on the code length — so the length was reaching
the code-RAM address through `restart` → the commit mux → `code_length − 1` →
the wrap compare. Measured: 2.9 ns, in front of a LUTRAM read.

The length a restart wraps with is the **staged** CSR, because the restart is
what commits it, and the staged CSR is a register that has been stable for as
long as the host has left it alone. `ChannelWithCSR` now hands it over as
`restart_length` instead of bypassing the commit register with a mux, so the
address arithmetic starts from a flop rather than from `restart`. Standalone,
`CodeReplica.restart_length` just follows `code_length` and nothing changes.

### 5.5 One that made it worse first

Iteration 3 computed the registered tap selection from a "next phase" signal
that was `stb ? acc_next : code_frac`. That put the **strobe** in front of the
26-bit phase adder and the sub-chip multiply — and `stb` is the RX strobe, gated
by `control.enable` and routed across the whole bank, so it arrives late. WNS
went *backwards*, from −0.788 to −1.733 ns, on a path from the `control` CSR
storage bit to a replica window flop.

The strobe belongs on the register *enables*, not in the datapath: when no
sample is strobed the registers simply hold, which is the same answer because
`code_frac` holds too. Worth recording because the mistake is easy to make twice
— "compute it one cycle early" is right, "select which value to compute from
with the thing that says whether a sample happened" is not.

### 5.6 What closed, and what it cost

```
python build.py --channels 4 --num-ants 1 --max-code-length 4092 \
                --taps 5 --max-subchips 12 --timing-effort high --build
```

`gnss_m2sdr_m2_x1_ch4_ant1_code4092_tap5_sub12` — **timing closed**:

```
WNS(ns)   TNS(ns)  Failing  Total      WHS(ns)  THS(ns)  WPWS(ns)
  0.015     0.000        0  109810       0.006    0.000     0.047
```

| Clock | Period | WNS | Failing |
|---|---:|---:|---:|
| `crg_s7pll_clkout0` (**sys_clk**) | 8.000 ns | **+0.015** | 0 / 102 038 |
| `rfic_clk` | 4.069 ns | +0.061 | 0 / 2 003 |
| `clk100` | 10.000 ns | +8.935 | 0 |
| `si5351_clk0` / `si5351_clk1` | | +21.313 / +4.798 | 0 |

Utilisation (`*_utilization_place.rpt`): 25 550 slice LUTs (19.10%), of which
5 818 LUT-as-memory (12.59%); 20 871 registers (7.80%); 10 174 slices occupied
(30.42%); 48 BRAM tiles (13.15%); **56 DSP48E1** (down from 112 at two
antennas); 2 346 F7 muxes, 0 F8.

At the default implementation effort the same design lands at **−0.262 ns** with
43 failing endpoints, and **every one of them is in litex_m2sdr's own AD9361
datapath** — 17 on the block-floating-point `max_abs` comparator in sys_clk and
26 in the 245.76 MHz `rfic_clk` domain. Nothing in the tracking bank fails at
that point; the bank just makes the die tight enough to push AD9361 paths that
had ~15 ps of margin in the v1 reference over the edge. `--timing-effort high`
(`place ExtraTimingOpt`, `phys_opt Explore`, `route Explore`,
`post-route phys_opt AggressiveExplore`) recovers it: post-routing already
reports WNS −0.019 ns, and the post-route physical-synthesis pass closes it.

Kept, relative to the configuration #32 aimed at:

- **five taps** (VE/E/P/L/VL), the whole point of v3;
- **`max_subchips` 12** — BOCsin, BOCcos, CBOC and TMBOC, i.e. every L1
  modulation GNSSSignals exposes;
- **4 channels**, each independently configurable for 3 or 5 taps;
- **4092-chip code RAM**, which covers every primary code this board can
  actually receive at L1: GPS L1 C/A (1023), Galileo E1B/E1C, GPS L1C and
  BeiDou B1C (4092).

Given up, and why it is affordable here:

- **The second antenna** (`--num-ants 2 → 1`). This is the expensive one: it
  halves the per-antenna accumulators and their DSP48E1s, and those cascades
  (`DSP P → saturating clamp → next DSP C`, and `BRAM → DSP PCIN`) were three of
  the five failing path shapes at iteration 5. Post-correlation beamforming
  (GNSSReceiver.jl#107) needs two antennas and this build cannot do it. Nothing
  in the v3 acceptance set does.
- **10 230-chip codes** (`--max-code-length 10230 → 4092`). Those are GPS L5,
  Galileo E5, BeiDou B2a and GPS L2CM — all on centre frequencies this
  single-front-end L1 board is not tuned to. See §5.5 for why this one is a
  hard floor rather than a preference.

**What a full `ant2 / 10230` build would need**, in the order the evidence
points: (a) the code RAM read has to stop being a one-cycle asynchronous
160:1 mux — block RAM with its synchronous output register, or a registered
two-stage read, with the restart case given a dead cycle it currently does not
have; (b) the correlator's saturating accumulate has to stop cascading two
DSP48E1s combinationally per tap per antenna — clamp a cycle later, or accept
`accum_bits` that fits one DSP; (c) LiteX's own CSR readback mux was already at
−0.644 ns with 4 channels, so it needs looking at before the channel count grows
again. None of these is deep; they are just three more of the same kind of
change as §5.1–§5.4, and each needs its own measured build.

### 5.6b On the board: it was flashed, and it does not correlate

**This build was flashed, tested, and rolled back.** The timing story above is
complete and the board-free suite is green (210/210), but the gateware does not
correlate on hardware, so the board is back on its previous image. What follows
is what was measured, not what was expected.

Flashed to the operational slot; after a host reboot (see §5.8) the board came up
on it — `SoC Identifier ... built on 2026-09-17 12:40:34`, FPGA Operational.

**Verified live, over CSR:**

| Check | Result |
|---|---|
| `gnss_version.csr` | **3** |
| `gnss_version.record` | 2 |
| `gnss_capabilities` | `0xffc202018050104` → n_channels 4, num_ants_max 1, num_taps 5, code_frac_bits 24, carrier_phase_bits 32, accum_bits 32, max_code_length **4092** |
| `gnss_signal_caps` | `0x10187011f` → tap_layouts **[3, 5]**, modulations `0b11111` (**bit 4 `:BOCsin` set**, plus LOC/BOCcos/CBOC/TMBOC), max_subchips **12**, replica_bits 8, reports_code_phase 1 |
| sample counter | 4 006 163 samples/s against fs = 4 MHz |
| `gnss_chN_code_freq` readback | `0x4178d5`, exactly the host's computed word |
| `dump_code_chip` | **1022** on every epoch — the wrap lands on the last chip of a 1023-chip code |
| `dump_num_taps` | 3 for a three-tap channel |
| dump rate | ~1000 dumps/s = one per 1023-chip code period |

So the CSR map, the capability reporting, the code NCO, the epoch detector, the
chip-index reporting and the dump machinery are all correct on silicon.

**Not verified — and the reason this was rolled back:**

*GPS L1 C/A does not acquire.* Every accumulator saturates at the `accum_bits`
rail (−2³¹) and `dump_saturated` is set, on every dump, for every channel. The
acquisition metric is exactly **1.0** (peak = median) for every PRN tried, where
the same host code on the previous image gave 14–66 and detected seven
satellites.

The decisive measurement: load three different codes into the same channel and
read the prompt accumulator.

| Code loaded | Replica it implies | Measured `ip` |
|---|---|---|
| all ones | constant **+1** | ≈ −2.147e9 (rail) |
| all zeros | constant **−1** | ≈ −2.146e9 (rail) |
| real C/A PRN 1 | alternating, zero-mean | ≈ −2.147e9 (rail) |

All-ones and all-zeros should rail with **opposite signs**, and the real code
should not rail at all. They are indistinguishable. **The correlator product is
independent of the code RAM contents**: the replica is not reaching the
multiplier. The code NCO addressing that RAM is demonstrably correct (the chip
index and epoch are right), so the fault is in the registered replica path of
§5.1/§5.4 — between the code RAM read and the DSP `B` input — in a way the
board-free suite does not reproduce.

The saturation is a consequence, not a second fault. A constant replica makes
the accumulator `±amp × Σ sample` over the ~4000 samples of a 1 ms epoch, and
with `replica_bits = 8` (`amp` up to 127) and 16-bit samples that rails on any
DC at all in the RX chain — 4000 × 127 × 4200 already exceeds 2³¹. A *correct*
zero-mean replica cancels that DC, which is exactly why the previous image did
not saturate. So one fault explains both observations, and there is no need to
suspect the sample path as well.

*DMA1 record capture: attempted, not completed.* With the v3 image on the board,
`read()` on `/dev/m2sdr1` blocked and the probe timed out after 200 s. Whether
that is `software/record_stream.py`'s DMA-writer ioctl or the record path itself
was not established. **No record was captured, framed or parsed on hardware.**

### 5.6c Measured on the failing build: what it is not, and what is left

On 2026-09-17 the v3 bitstream was put back on the board for one session, the
two readings of the previous revision of this section were taken, and it was
rolled back. **Everything below is measured on the failing gateware**, which is
the thing none of the earlier simulation and netlist work could reach.

*Reading 1 — the integration window is exact.* `integrated_samples` reads
**4000 on every dump**, which is one 1023-chip epoch at fs = 4 MHz, with
`dump_code_chip` 1022 and `code_length_active` 1023. **The "accumulator is never
cleared" branch is dead**: the accumulator is cleared, every epoch, on time.

*Reading 2 — the samples are fine.* A DMA0 capture taken from the same stream
while the correlators ran: I mean **+0.07**, std **86.8**, |I|max **1148**; Q
mean +0.06, std 86.4. Zero-mean, no DC, nothing unusual.

*And those two together are a contradiction.* With a constant `+1` replica (an
all-ones code, LOC unit table), 4000 samples and a carrier amplitude of at most
127, the largest `|ip|` this stream can possibly produce is
127 × 4000 × 1148 = **5.8 × 10⁸**. The measured `ip` is **−2.147 × 10⁹**, and
`dump_saturated` is 1. Even a worst-case accumulation, every sample pushing the
same way, cannot reach the rail. **The correlator is not summing these samples.**

(A method note, because the first attempt got it wrong: once the accumulator has
saturated, `ip` only bounds the sum, so dividing it by `N × amp` does *not*
measure the mean sample. The resulting "−4226" is a lower bound — and the fact
that it exceeds the stream's own maximum of 1148 is the contradiction, not a
measurement of anything.)

*The decisive test — the replica's sign does not reach the output.* Load an
all-ones code (replica constant **+1**), read the accumulators; load an
all-zeros code (replica constant **−1**), read them again:

| code | replica | `ip` |
|---|---|---:|
| all ones | +1 | −2 147 472 091 … −2 143 939 205 |
| all zeros | −1 | −2 146 150 275 … −2 147 357 537 |

**The same, both railed negative.** Inverting the replica must invert the sum;
it does not. A real C/A code gives the same picture. `gnss_saturation` reads
`0xf` — all four channels — with `gnss_overflow` 0, `gnss_rate_error` 0 and
`code_status` 0.

*What is still alive.* The five taps are not identical: `ip`, `ie` and `il`
differ from each other dump to dump (VE/P/VL coincide only because the channel
was in its three-tap layout, `dump_num_taps` 3). So the tap structure and the
per-tap selection are doing something; it is the sign and magnitude of the
product that are wrong.

**What this leaves.** Measured good *on the failing silicon*: the sample path,
the integration window, the code RAM, the code NCO, the epoch detector, the dump
machinery and the whole CSR map. Cleared earlier in simulation and in the
implemented netlist: the runtime code load, the `soc.py` rewiring and §5.2's
pipeline stage, latch inference, RAM inference and replica constant-propagation.

The fault is in the **multiply / accumulate / saturate stage** — between the
replica and the accumulator — and nowhere else that has been looked at.

One hypothesis the evidence is consistent with but which is **not demonstrated**:
this is the first build with `replica_bits = 8` (every image that ever worked had
2, because `replica_bits_for` returns 2 for `max_subchips <= 1`). The product and
accumulator datapath is therefore sized for an 8-bit replica even though GPS L1
C/A's runtime amplitude is ±1, and `dump_saturated` is set on the very first dump
after a restart. Testing that means reading the accumulate stage in `channel.py`
against `accum_bits`, and then a build — neither of which is in this session's
scope.

### 5.6d The v3-aware host adapter, against real v3 silicon

Taken in the same window because it is otherwise unobtainable: the board is
normally on the 2026-07-29 image, whose v1 CSR layout `M2SDRCorrelator` refuses
by name at construction.

GNSSM2SDR.jl master `67ca254` (PR #11 made `M2SDRCorrelator` speak CSR layout v3,
PR #12 moved the examples onto GNSSReceiver's `HardwareCorrelatorLink` with
`NCOReferencedPLLAndDLL`), with GNSSReceiver at `hardware-correlator-12`:

```
raw stream started: RawStream{SignalChannels.SignalChannel{Complex{Int16}, 1, Matrix{Complex{Int16}}}}
*** CONSTRUCTED against v3 silicon:
    M2SDRCorrelator{1, Tracking.VeryEarlyPromptLateCorrelator{1, ComplexF64}}
```

**It constructs, and it negotiates the five-tap layout** — the correlator type it
selects is `VeryEarlyPromptLateCorrelator`, chosen from the gateware's own
capability registers, so the v3 capability gate and the `tap_layouts` handshake
both work against real hardware. Tracking was not attempted: the correlators do
not correlate (§5.6c above), so there would be nothing to see. **Arming a channel
was not reached** either — two accessor calls in the probe failed on the probe's
own scoping mistakes, and the rollback took priority over fixing them.

### 5.6e The clamp that was lowered to an unsigned compare

§5.6c isolated the fault to the multiply/accumulate/saturate stage. It is in
none of the places that were searched: not the RTL's meaning, not Migen, not
synthesis, but **the Verilog the build writes**.

`litex/gen/fhdl/expression.py::_generate_constant` formats a negative constant
as `"-" + nbits + "'" + hex(abs(value))` and never writes the `'s` signedness
marker. The saturating accumulator's lower bound therefore came out as

```verilog
if ((trackingchannel_raw0 > $signed({1'd0, 31'h7fffffff})))   // upper: signed
if ((trackingchannel_raw0 <  -32'h80000000))                  // lower: UNSIGNED
```

`32'h80000000` is an unsigned literal and unary minus keeps it unsigned. Verilog
evaluates a relational expression as unsigned whenever *either* operand is, so
`raw` — a signed sum — was reinterpreted as unsigned and **every positive
partial sum compared "less than" the negative rail**. The accumulator was
clamped to −2³¹ on the first sample of every integration.

Proven rather than argued, in Vivado's own xsim:

```
raw = 40000000     emitted (raw < -32'h80000000) = 1    signed compare = 0
raw = -40000000    emitted = 0
raw = 2147483647   emitted = 1
```

and running the pre-fix RTL through LiteX's converter reproduces the flashed
bitstream's line byte for byte, ten times — one per tap per I/Q.

**Why the board-free suite could never see it.** The tests convert with
`migen.fhdl.verilog`, which renders the same constant as `32'sd2147483648` —
signed, two's complement, correct. The build converts with
`litex.gen.fhdl.verilog`. *The suite and the bitstream were never produced by
the same backend*, so no amount of simulation could have found this.

**The fix.** Both magnitude comparisons are replaced by the standard range test:
a two's-complement value fits in `accum_bits` exactly when every bit at or above
the sign position equals the sign bit. That is an equality on unsigned slices,
so no signed literal exists for any backend to render wrongly — and it is
cheaper, 25 395 LUTs against 25 550, two 34-bit comparators traded for a few.

`test/test_verilog_lowering.py` checks both backends: that the accumulate sum
never meets a relational operator at all (the shape is what is dangerous, since
how a constant renders is a backend detail), and directly that LiteX's output
carries no negative literal in a comparison. Six of its tests fail against the
pre-fix code.

### 5.6f What the fix changed on silicon — and what it did not

Built at `--timing-effort max` (WNS **+0.012 ns**, TNS 0.000, 0 of 109 826
endpoints; `high` missed at −0.070 with 24 failing, 21 of them in litex_m2sdr's
own AD9361 block-floating-point comparator). Flashed 2026-09-17 19:57.

| | broken build | fixed build |
|---|---|---|
| `dump_saturated` | 1 on every dump | **0 on every dump** |
| `gnss_saturation` | `0xf` (all four channels) | **`0x0`** |
| `ip` | pinned at −2.147×10⁹ | ±10⁵–10⁶, noise-like |
| `integrated_samples` | 4000 | 4000 |
| dump stream | 11 lost-record gaps, 186 skipped epochs | **0 across every counter** |

The clamp does exactly what it was meant to do. **The correlator still does not
correlate.**

Against a satellite Acquisition.jl measures at **53.5 dBHz** (PRN 29, −2200 Hz),
a code-phase sweep of the FPGA channel at that Doppler gives:

```
peak/median = 5.9          (1 ms coherent at 53.5 dBHz predicts ~224)
top bins:  692.5  855.0  921.0  397.0  650.0  611.0   -- scattered
bins within 1 chip of the peak: some as low as 0.4x median
```

A real correlation peak is one to two chips wide, so every half-chip bin beside
it must be elevated too. This one is a single isolated bin among noise. The
sweep's own drift (code Doppler over 15 s ≈ 21 chips) would *move* a peak, not
erase it, and the integration window is the correct 4000 samples.

This also retro-explains the closed-loop run on the same image: channels arm,
NCO words commit, the dump stream is perfectly clean — and C/N₀ reads `-Inf`,
because there is no signal power in the records. (No position fix was expected
regardless: the build has four channels, the link reserves one as its noise
reference, and a fix needs four.)

**So the clamp was masking a second fault, not causing it.** That is worth
recording as a result: §5.6c's isolation was correct as far as it went, and the
remaining question is now bounded to the same stage with the clamp eliminated.

**Hypotheses for the second fault, in the order they should be tested.** None of
these is demonstrated; this is where to start, not what is true.

1. **The replica never reaches the DSP `B` input.** This was the original §5.6b
   reading and the clamp bug does not rule it out. *Test:* the all-ones vs
   all-zeros comparison is uninformative on this board because DMA0 is zero-mean
   (I mean +0.07), so a constant replica integrates to ~0 either way. Give it a
   DC term instead — an AD9361 DC-offset-correction setting, or an in-band CW —
   and the two loads must come out equal and opposite.
2. **The carrier wipe-off.** `i_bb`/`q_bb` are formed one cycle before the
   replica multiply; if the carrier LUT or its phase is wrong the product is
   destroyed without any symptom in the code NCO, which is separately confirmed
   good (`dump_code_chip` 1022 every epoch). *Test:* set `carrier_freq` to 0 and
   read a channel correlating against an all-ones code with a CW injected at the
   LO — the accumulator should then follow the CW's amplitude.
3. **Tap/replica alignment inside the two-stage pipeline.** `rep_r`, `epoch_r`
   and `cphase_r` are registered on `sample_stb` while `i_bb`/`q_bb` are formed
   from the same strobe; a one-cycle skew between the replica and the baseband
   it multiplies would leave every counter healthy and destroy correlation.
   *Test:* a bank-level simulation that drives a *known* modulated signal
   (`bpsk_boc_signal` already exists in `test_five_tap.py`) through
   `GNSSTracking` and asserts the prompt accumulator peaks at the right code
   phase — the suite currently checks accumulator arithmetic against a software
   model, but never that a real signal correlates.
4. A CSR readback artefact is **ruled out**: the same values appear in the DMA1
   record path through GNSSReceiver, which read `-Inf` C/N₀ independently.

Hypothesis 3 is the one to do first: it needs no hardware, and the gap it names
— no test anywhere asserts that a modulated input produces a correlation peak —
is the same shape of hole as the `load_we` gap and this clamp.

**Galileo E1B/E1C: untested.** This build carries what E1 needs — 4092-chip
codes, five taps and the sub-chip machinery — and demonstrating a non-GPS-L1-C/A
signal through the hardware correlator is the point of the whole exercise. It
was not attempted, deliberately: there is nothing to learn from tracking E1
through a correlator that does not correlate GPS L1 C/A, and a result there
could only be noise. It stays untested rather than being inferred from the L1
C/A work either way.

**`--timing-effort max`.** Vivado is deterministic for a given netlist and
directive set, so a build that misses by picoseconds cannot simply be run again
— the directives have to change. `max` escalates the two passes that move a
sub-100 ps setup miss: post-place phys_opt and routing both go to
`AggressiveExplore`. On this design `high` gave −0.070 ns and `max` gave
+0.012 ns, on the same RTL.

### 5.6g The second fault: the carrier ROM was written as `-3`

§5.6f left one bounded question: with the clamp fixed, why does the channel
still not correlate? Hypothesis 2 there — *the carrier wipe-off* — is the
answer, and like the clamp it is a property of the Verilog the build writes,
not of the design.

**What the build writes.** The carrier NCO's sin/cos tables are `Memory`
specials with signed `init` values (`_sincos_tables` returns −127…127). LiteX
emits every memory's contents into a `$readmemh` file, formatting each entry
with `"{:02x}".format(d)`. For a negative `d` that is not hex at all:

```
$ sed -n 127,134p gnss_m2sdr_m2_x1_ch4_ant1_code4092_tap5_sub12_sin_mem.init
06
03
00
-3
-6
-9
-c
-10
```

Generated from `build.py --channels 4 --num-ants 1 --max-code-length 4092
--taps 5 --max-subchips 12` on 2026-09-18 with the toolchain of
requirements-test.txt: **eight `.init` files (sin and cos, four channels), 127
of 256 entries negative in each.** Every bitstream this repository has ever
built shipped those files.

**What the two readers make of them — measured, not inferred.**

| Reader | `00 03 06 -3 -6 7f 81 00` becomes | Effect on the ROM |
|---|---|---|
| xsim `$readmemh` | `00 03 06` then *"Illegal hex digit '-'"* and it stops | entries 3… keep their previous value (`x`) |
| Vivado 2024.1 `synth_design` | `00 03 06 03 06 7f 81 00` — no warning, no error | **the sign is dropped** |

The synthesis row is the one that reached the board: a tiny ROM with exactly
that file, synthesised out of context and its netlist simulated with the
unisim library, reads back `03`/`06` where `-3`/`-6` were written. So the
flashed gateware held **|sin θ| and |cos θ|**. A rectified carrier has no
component at the carrier frequency — only DC and even harmonics — so the
wipe-off product `I·|cos| + Q·|sin|` averages to zero over any integration in
which the NCO phase turns, i.e. for every Doppler but zero. That is exactly the
§5.6f picture: code NCO right, chip index right, integration window right,
accumulators noise-like at ±10⁵–10⁶ (the noise floor a 127-amplitude
"carrier" produces), and no peak anywhere in a code-phase sweep of a 53.5 dBHz
satellite.

**Why the board-free suite was blind, again.** The Migen simulator holds
`Memory.init` as Python integers; it never writes or reads a file. The gap is
the same one §5.6e named — the suite and the bitstream are produced by
different tools — and it is now closed from both sides:

- `test/test_verilog_lowering.py::TestTheMemoryInitFilesAreReadable` lowers the
  bank with LiteX's converter and requires every `.init` entry to be unsigned
  hex within the memory's width. It fails on the pre-fix `carrier_nco.py`.
- `test/test_verilog_lowering.py::TestNoNegativeLiteralAnywhere` requires that
  no unary negative literal (`-N'h…`) appears anywhere in the LiteX output, not
  only in comparisons. That holds with LiteX at or after `37b75bd4`
  (2026-07-24, *"emit negative signed constants as $signed(N'h<pattern>)"*),
  which is why requirements-test.txt now pins that commit: the previous pin,
  `93c8d230`, is the commit **before** it, and rendered the clamp bound as
  `-32'h80000000`. This is very probably the whole story of §5.6e as well —
  the July builds were made against a LiteX that already had the fix, the
  five-tap builds against a pin one day too old.
- `scripts/xsim_correlation.py` runs the LiteX-lowered `TrackingChannel` in
  Vivado's own simulator against a synthetic satellite and compares every dump
  with the Migen simulation of the same netlist, bit for bit. It is the test
  §5.6f asked for under hypothesis 3. `test/test_xsim_correlation.py` runs it
  as part of the suite wherever xvlog/xelab/xsim are on the PATH and skips by
  name where they are not (CI).

**What the simulator says, before and after.** GPS L1 C/A PRN 1 at 4 MS/s,
+1500 Hz, amplitude 200 on σ = 30 noise, two code periods, one idle clock
between samples; the satellite sits exactly on the replica, so the aligned
prompt is `200 × 127 × 4000 ≈ 1.02 × 10⁸`:

| Gateware | `.init` files | xsim `ip` | Migen `ip` | Verdict |
|---|---|---:|---:|---|
| main `8795de1` (#42) | as written (`-3`…) | `2147483647`, `dump_saturated` = 1 | 101 597 180 | ROM is `x` from entry 129; **FAIL** |
| main `8795de1` (#42) | as Vivado reads them (sign dropped) | 17 325 840, then −17 176 541 | 101 597 180 | 6× low and sign-flipping: **no correlation — the board's symptom, reproduced** |
| this fix | as written | **101 597 180** | 101 597 180 | **PASS**, all six accumulators bit-exact over both dumps |

Galileo E1B, CBOC(6,1,1/11) on five taps at 24.552 MS/s, −2500 Hz, one 4092-chip
period (98 209 samples): **PASS**, all ten accumulators bit-exact. One thing the
run showed on the way: at test amplitude 200 the accumulators hit the 32-bit
rail (`dump_saturated` = 1 in both simulators, still bit-exact) — a CBOC
replica peaks at 25, and `25 × 127 × 200 × 98 209` is 6 × 10¹⁰. A live E1
satellite is a few LSB per sample and stays two orders of magnitude below the
rail; the suite's E1 case uses amplitude 20.

**The fix** (`carrier_nco.py`): the tables are stored as their two's-complement
bit patterns (`rom_words`), which is what an 8-bit signed ROM holds anyway and
what every `$readmemh` reader can parse. Nothing about the NCO's behaviour
changes in the Migen simulator — `test_carrier_nco.py` passes unchanged —
because the read port is signed and reinterprets the pattern.

What the July builds did with the same files is not known: no `.init` or
Vivado log from them survives, only the `.bin`. They demonstrably wiped the
carrier off (satellites at −1.6 to −6.8 kHz Doppler tracked for minutes), so
whatever toolchain produced them did not read `-3` as `03`. The fix is right
for every reader, so the question is recorded rather than pursued.

**Status of this fix on hardware:** see §5.9.

### 5.7 What is on the board, and rolling back

The identification in the first version of this page was wrong and it matters
for a rollback, so here is the evidence rather than the conclusion.

The live SoC identifier reads *built on 2026-07-29 23:42:50*. The only `csr.csv`
in the tree with that timestamp is `gnss_m2sdr_m2_x1_ch20_ant1`
(2026-07-29 23:42:51) — a **20-channel** build, not
`RELEASE_ch4_ant1_timing_clean` (2026-07-28 16:41:45). Reading the correlator
CSRs with the ch4 map returns zeros and a `gnss_saturation` of `0xfff80`, which
is a 20-bit field; with the ch20 map the same registers return live correlator
values. Dumping the operational slot confirms the size: content runs to
`0x5c0000` ≈ 6.03 MB, and only the 20-channel image (6 020 228 bytes) is that
large.

That build is itself **WNS −0.181 ns** over 507 endpoints, so the board has not
been running a timing-clean image.

Before flashing, the whole operational slot was read back byte for byte to
`~/gnss-m2sdr/rollback/op_slot_backup.bin` on orin2 (7 MiB,
md5 `8f04c9ecf12711efb76bf2a7dd700219`, a valid bitstream: `aa995566` sync at
offset `0x30`, content ending at `0x5c0000`). That file *is* the rollback — it
restores exactly what was there, which no `.bin` in the tree can:

```bash
ssh orin@orin2 'flock -w 1800 /tmp/gnss-hw.lock bash -s' <<'HW'
cd ~/litex_m2sdr/litex_m2sdr/software/user
./m2sdr_util flash_write -y -c 0 /home/orin/gnss-m2sdr/rollback/op_slot_backup.bin 0x00800000
HW
ssh orin@orin2 'sudo shutdown -r +0'      # see 5.8: reboot, not flash_reload
```

Use `m2sdr_util flash_write` directly, **not** `flash.py`: `flash.py` builds its
command as `cd user && ./m2sdr_util flash_write ... ../$bitstream`, so an
absolute path becomes `..//home/orin/...` and the write fails. And give the
write no timeout — it takes about 80 s for 7 MiB and interrupting it mid-erase
leaves a half-written slot.

The golden image at offset `0x0` is not touched by a flash of the `0x00800000`
operational slot, so it remains the automatic fallback if an operational image
fails to configure at all.

**The v3 build was flashed, measured (§5.6b), found not to correlate, and rolled
back.** `m2sdr_util flash_write` of `op_slot_backup.bin` to `0x00800000`
reported `Success.` and exited 0 at 2026-09-17 13:19 UTC, and the pre-flash
driver headers, kernel module and user tools were restored and rebuilt in the
same session.

**Confirmed on the board after the reboot** (2026-09-17): the SoC identifier
reads *built on 2026-07-29 23:42:50* again, and the whole chain works — see
§5.7b for the measurement. The regression baseline is intact.

The v3 image went back on once more the same evening for the measurement session
of §5.6c, and was rolled back from the same byte-exact backup. Confirmed again
afterwards: identifier *2026-07-29 23:42:50*, FPGA Operational, and GPS L1 C/A
acquiring on five PRNs above a median-38.8 dBHz floor (MAD 0.39) — PRN 24 at
58.0, 32 at 50.2, 25 at 45.8, 29 at 45.3, 28 at 41.4.

One trap worth writing down: the tracking bank is an observer on the RX stream,
so it only sees samples while DMA0 is draining and `m2sdr_record` has to outlive
whatever is running. Its byte limit is not a hint — `m2sdr_record /dev/null
100000000` exits after ~6 s at 4 MSPS, and everything downstream then reads
zeros or noise with no error anywhere. Give it a limit that cannot be reached
and re-check the sample counter at the end. Note also that the CSR map of the
image on the board predates #31: driving it needs the host code from commit
`8220716`, not this branch's.

### 5.7b How the board is confirmed working — and how not to do it

**Acquisition belongs on the CPU. The FPGA does downconversion and correlation
for *tracking*; closing the loop is CPU-side too.** The confirmation therefore
has three steps, and the first draft of this section got it wrong by collapsing
them into one.

*Step 1 — CPU-acquire from the raw DMA0 stream, with Acquisition.jl.* 2.00 s
capture at fs = 4 MHz (sc16, 2T2R, 8 bytes per sample instant, RX1 = words 1 and
2), `min_doppler_coverage = 50 kHz`, 10 coherently integrated code periods, 10
noncoherent accumulations. The wide Doppler span is not optional: the device
TCXO is poor, 1 ppm at L1 is 1.575 kHz, and satellites have been found at
−8.5 kHz — outside any sweep sized for satellite motion alone.

Two captures 40 minutes apart, all 32 PRNs, CN0 in dBHz:

| | floor (median ± MAD) | above floor |
|---|---|---|
| capture A | 39.6 ± 0.25 | PRN 20 (59.2, −6000 Hz), 19 (56.6, −1100), 15 (49.5, −4200), 10 (41.2, −1400) |
| capture B | 38.5 ± 0.42 | PRN 20 (57.4, −6600), 19 (53.4, −1500), 15 (45.3, −5000), 24 (42.2, −2300), 10 (41.6, −2000) |

28 of 32 PRNs inside ~1 dB of the median is what a noise floor looks like; a
handful of satellites 6–20 dB clear of it is what a detection looks like.

*Steps 2 and 3 — hand the code phase and Doppler to an FPGA channel, and close
the loop on the host.* **Name the loop filter, or the run is not reproducible**
— this was the one parameter the first version of this section left out, and it
turned out to change how two of the numbers should be read.

**Run A — `ConventionalAssistedPLLAndDLL`.** 40 s, one FPGA channel per
satellite, on orin2 via `~/hwloop/closed_loop_multi.jl` (a bespoke `M2Bank.jl`
CSR shim, not GNSSM2SDR.jl), Tracking.jl `NPTna`:

| PRN | prompt \|P\|²/floor over 40 s | carrier the loop held | Acquisition.jl said | |
|---:|---|---:|---:|---|
| 19 | 24–32× | −1580 to −1627 Hz | −1100 / −1500 Hz | locked |
| 15 | 26 → 52× | −5210 to −5281 Hz | −4200 / −5000 Hz | locked |
| 20 | 26 → 46 → **7.8×** | −6810 to −6877 Hz | −6000 / −6600 Hz | locked, **fading** |
| 24 | 1.0–1.5× | **diverged to +48 700 Hz** | 42.2 dBHz, marginal | no lock |

`saturation = 0xffff0` (bits 0–3, the channels in use, clear), `overflow = 0x0`.

**Run B — `NCOReferencedPLLAndDLL`**, the delay-aware estimator GNSSReceiver
calls the hardware receiver's default, at GPS L1 C/A's 18 Hz reference
bandwidth. 180 s, same board, same image, through GNSSReceiver's own
`HardwareCorrelatorLink` and GNSSM2SDR.jl
(`~/delay-study/GNSSReceiver/examples/hardware_correlator_m2sdr.jl`,
`feedback_delay_epochs = 1.5`):

```
t=138.0 s  cn0[20:46  15:37  23:44]
t=158.4 s  cn0[20:45  23:43]
t=178.8 s  cn0[20:42  23:43]

NCO commits: 381095 at their scheduled sample, landing 0.085 ms late on
             average (max 17.437 ms); 148 dropped as stale
dump stream: lost-record gaps 11, re-arm gaps 0, device-reported drops 0,
             skipped epochs 186, implausible indices 0, dropped NCO updates 0
```

**PRN 20 held 42–46 dBHz across the whole 180 s with no decay.** Under the
conventional loop the same satellite faded monotonically to 7.8× floor over the
last 20 s of 40 s. The fade was **the loop, not the correlator** — which is
exactly the failure `nco_referenced_loop.jl` documents: a hardware NCO word
lands milliseconds after the record that motivated it, and at 18 Hz a
correction acting 3–4 ms late overshoots and limit-cycles "while C/N₀ and code
lock look perfect". PRN 24's divergence to +48.7 kHz in run A is the same shape
and should be read the same way.

**So the hardware-correlator claim is run B's**, and it is stronger than run
A's: satellites tracked through the FPGA correlator at 42–46 dBHz for three
minutes, 381 095 NCO words committed at their scheduled sample landing 0.085 ms
late on average, and zero device-reported drops, zero implausible indices and
zero dropped NCO updates. Run A stands only as a *comparison*: the prompt power
and the carrier agreement are real, but its two failures are attributable to the
estimator and must not be quoted as correlator behaviour.

Two caveats, recorded because they were observed. Run B reached **no position
fix** — only two to three satellites held ephemeris-long, and four are needed.
And the example **segfaulted at exit**, after the measurement and the summary
lines above were printed; nothing in the run depends on what happened after.

*A note for anyone reusing `~/hwloop/closed_loop_multi.jl`*: it sets
`doppler_estimator = ConventionalAssistedPLLAndDLL()` (line 178), and it also
needs a fix to run at all against Tracking `NPTna` — it calls
`reset_start_sample_and_bit_buffer!` only inside its once-per-second print
branch, so the 128-bit hard-bit buffer overflows before the first print. Prefer
run B's path.

*And the way that does not work.* `software/gnss_tracking.py:acquire()` sweeps
for satellites *through* the FPGA correlator over CSR, scoring peak/median of
prompt power. **It cannot distinguish signal from noise and must not be used to
claim a detection.** Measured against the ground truth above, on the same sky:

- real PRNs (20, 19, 15, 10) median metric **26.65**; known-floor PRNs
  (3, 14, 17, 22, 28) median **24.47** — a separation of **1.09×**;
- PRN 20, the strongest satellite in the sky at 59.2 dBHz with a true Doppler of
  −6000 Hz, scored above the `detect_metric = 8.0` threshold at **all 33 Doppler
  bins** of a ±8 kHz sweep, peaking at **+5500 Hz** — 11.5 kHz from the truth —
  while reading 12.8 at the true Doppler, below its own sweep median;
- PRN 14, which is not there, also scored above threshold at every bin.

The mechanism is in that function's own docstring: the metric has a noise
baseline of the same order as a real 1 ms peak, and the sliding scheme smears
the peak further. An earlier revision of this page reported "GPS L1 C/A acquires
on ten of ten PRNs tried" from exactly this sweep. Ten of ten should have been
the tell — a real sky does not hand over every PRN you ask for.

### 5.9 The carrier-ROM fix, built and flashed (2026-09-18)

Same RTL as §5.6f plus the `carrier_nco.py` fix of §5.6g, LiteX `37b75bd4`,
`litex_m2sdr` `b10dc4d`, Vivado 2024.1, all at `--channels 4 --num-ants 1
--max-code-length 4092 --taps 5 --max-subchips 12 --timing-effort max`:

| Directives on top of `max` | WNS | Failing | Where |
|---|---:|---:|---|
| none | −0.069 ns | 14 | all in litex_m2sdr's AD9361 `bfp8_max_abs` path (a CSR storage bit → 15 logic levels → the comparator's CE) |
| `place=ExtraPostPlacementOpt` | −0.115 ns | 87 | same, worse |
| `synth=PerformanceOptimized` | **+0.000 ns** | 0 | — |
| `place=AltSpreadLogic_high` | **+0.003 ns** | 0 | — |

The one that is *not* about the tracking bank at all — the block-floating-point
comparator is litex_m2sdr's own, on a sample format (`bfp8`) this receiver does
not use — is the one that decides whether a build is flashable, because
this design leaves it with tens of picoseconds either way. `build.py` grew
`--directive STAGE=DIRECTIVE` so that a miss can be answered with a different
directive set instead of a shrug: Vivado is deterministic for a given netlist
and directive set, so the same command line produces the same miss.

Utilisation is unchanged from §5.6 (25 269 LUTs, 20 935 registers, 5 818
LUT-as-memory, 48 BRAM tiles, 56 DSP48E1). Each build takes ~35 minutes on a
24-core host; four ran in parallel.

Six and eight channels, same directives: **−0.022 ns** (15 endpoints: 8 on the
same `bfp8_max_abs` path, 2 on LiteX's CSR readback mux) and **−0.173 ns** (52).
Six is within a directive set of closing; eight is the CSR readback mux
§5.6 already flagged as the next RTL change.

**Flashed:** `gnss_m2sdr_m2_x1_ch4_ant1_code4092_tap5_sub12_placeSpread`
(md5 `107f9ab635d4bd7210a7dc43b42c6c85`, 4 396 000 bytes) to the operational
slot at 10:00 UTC, after reading the slot back and confirming it was byte for
byte the 2026-07-29 image `op_slot_backup.bin` (md5 `8f04c9ec…`), and after
rebuilding the kernel module and tools against the new headers (the only base
peripherals that move are `pcie_dma1`/`pcie_endpoint`, 0x1f000/0x1f800 →
0x15000/0x15800; `flash`/`icap` do not, so the old tools can flash the new
image). `flash_write` → `flash_reload` → `shutdown -r`.

**The reboot needed a power cycle.** After `flash_reload` + `shutdown -r` the
host answered TCP on port 22 for 100 minutes without sshd ever sending a
banner — the kernel was up, userspace was not — until the board was power
cycled by hand. The 2026-09-17 sessions saw "~5 min, once ~40": count on a
power cycle after every flash, and do not poll the host every few seconds
while it is down. Up again, `m2sdr_util info` read *built on 2026-09-18
09:34:40*, the rebuilt module probed both DMA devices, and
`scripts/hw_accept_v3.py` passed all five checks: CSR layout 3 / record 2,
capabilities decoding to exactly this build, 4 002 507 samples/s on the
counter, GPS L1 C/A acquired on the FPGA sweep, and 512 DMA1 records framed
with a three-tap and a five-tap channel side by side on the wire.

**On sky, through GNSSReceiver** (`examples/analysis/hardware_live_m2sdr.jl`
there; its field record has every counter):

- *GPS L1 C/A, 300 s.* PRN 14 at 47–52 dBHz for the whole run, PRN 21 and 20
  at 34–45 dBHz; 359 439 NCO commits landing 0.04 ms late on average, 3
  lost-record gaps (one 71 ms event at the first acquisition merge), 0 device
  drops. **The correlator correlates.** No fix: four channels are one satellite
  short once the acquisition's false alarms have had their turn.
- *Galileo E1, five taps, BOC(1,1) replica on the 4092-chip code, 4 MS/s.* The
  hardware side was right from the first run — E1C PRN 16 at 48.7 dBHz — and
  the host side was not: every lock decayed within ten seconds because
  GNSSM2SDR held one pending NCO word per channel and let the next word
  supersede one not yet due, which starves any signal whose folds (4 ms) come
  faster than its words fall due (8 ms). GPS never noticed (2 ms and 2 ms).
  With the words queued per channel (GNSSM2SDR `fix/nco-queue`), an E1B-only
  run held four Galileo satellites at 36–47 dBHz, decoded their I/NAV pages
  and produced a **Galileo-only position fix after 39.8 s** (68 338 commits,
  0.01 ms late on average). **The first non-GPS signal through this
  correlator on sky.**
- *GPS L1 C/A next to Galileo E1B in one bank, 200 s*, the GPS search limited
  to two PRNs so the bank had channels for both: GPS PRN 14 on three taps at
  42–46 dBHz next to E1B PRN 34, 16 and 15 on five taps at 26–45 dBHz for the
  last 110 s, no tap-layout mismatch, C/N₀s agreeing with the
  single-constellation runs — the step-5 "mixed operation" criterion, on sky.
- Not run: CBOC (`GalileoE1B`) — the software acquisition's replica needs
  12.276 MS/s and this board's raw stream was left at 4 MS/s; the pilot's
  secondary-code synchronisation on hardware; any position accuracy statement.

**Six channels** close timing too, with `--directive synth=PerformanceOptimized
--directive place=AltSpreadLogic_high` on top of `max`: **WNS +0.007 ns**,
0 failing of 141 430 endpoints (`…_ch6_ant1_code4092_tap5_sub12_synthPerfSpread`,
5 374 172 bytes). Its `pcie_dma1`/`pcie_endpoint` bases are the same as the
four-channel build's, so the driver on orin2 already fits it; it is staged
under `~/gnss-m2sdr/build/` and not flashed. Eight channels miss by 0.173 ns on
52 endpoints (the CSR readback mux, as §5.6 predicted).

### 5.8 Three things that cost hours on the hardware side

*`flash_reload` is required, and it wedges PCIe on this host.* Both halves
matter, and an earlier revision of this page got the first one wrong.

**A warm reboot does not reconfigure the FPGA.** `shutdown -r` does not drop
power to the M.2 card, so the FPGA keeps the configuration it already holds and
never re-reads the flash. Measured on 2026-09-17: `flash_write` reported
`Success.` and exited 0, the host rebooted, and `m2sdr_util info` still showed
the *old* SoC identifier. The bitstream was in the flash the whole time. Anyone
following the old advice would flash, reboot, see no change and conclude the
write had failed.

`flash_reload` is the ICAP reconfiguration that makes the FPGA re-read the
flash, so it is not optional. It then wedges PCIe: the device answers every
config read with `0xff`, the kernel logs AER `CmpltTO`, and a `remove` +
`rescan` does **not** recover it — the bridge is gone from
`/sys/bus/pci/devices` entirely. So the reboot *after* `flash_reload` is
mandatory too. The working sequence is **flash_write → flash_reload → reboot**,
about 2 minutes end to end; orin2 has come back in ~5 min and, once, in ~40.

*Rebuilding the driver needs three headers, not one.* Copying LiteX's generated
`csr.h` into `litex_m2sdr/software/{kernel,user}` breaks the build — the
generated file `#include`s `generated/soc.h`, `system.h` and `hw/common.h`,
which the M2SDR tree does not have. `scripts/driver_headers.py` strips those
includes and emits the header the tree expects. Keep the originals: the
`make clean` that the rebuild runs deletes `m2sdr.ko` and `m2sdr_util` first, so
a failed build leaves the host with no working tools at all until they are
restored (`~/gnss-m2sdr/rollback/headers_before/`).

*Kill background processes by exact name, and close fd 3.* `pkill -f m2sdr_record`
matches the remote `bash -c` line of your own ssh session and kills it — use
`pkill -x`. And a process backgrounded inside a `flock` session inherits the
lock fd and keeps the board locked after the session exits; start it with
`3>&-`.
