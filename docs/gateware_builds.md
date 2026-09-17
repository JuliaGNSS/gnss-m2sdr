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

The v1 reference closed with **5 ps** of margin over 73 880 endpoints. That is
the context for everything below: this design was already at the edge of the
part before steps 3–5 added to it.

**What is on the board today is none of these.** It is a **20-channel** build
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

### 5.6c Diagnosis so far: what is ruled out, and where it points now

No silicon was used for any of this; it is all from the synthesis reports, the
generated Verilog and the board-free simulator.

**Ruled out.**

| Hypothesis | How | Result |
|---|---|---|
| Latch inference on the `Array(...)[k_r]` subcarrier mux | `grep -i latch` in the synthesis log; read the emitted Verilog | **No.** `checking latch_loops (0)`, and Migen emits a default assignment (`comb_self8 = 8'd0;`) ahead of the case, so the assignment is complete |
| Code RAM mis-inferred or wrong depth | Vivado's Distributed RAM mapping report | **No.** All twelve code RAMs (4 channels × 3 copies) map to `RAM128X1D`/`RAM64X1D`/`RAM32X1D`/`RAM16X1D` totalling exactly 4096 × 2 bits |
| Memory Verilog wrong | Read the emitted block | **No.** Async read at `rp_adr`, synchronous write, `$readmemh` init — a clean LUTRAM template |
| A runtime code load never reaching the replica | New tests, `TestRuntimeCodeLoadReachesTheReplica` and `TestAccumulatorsDependOnTheLoadedCode` | **No.** See below |
| Accumulator headroom lost to `replica_bits` 8 | `replica_shape("LOC")` is a one-entry **unit** table | **No.** GPS L1 C/A's replica is ±1 on a sub-chip build exactly as on v1 |

The load hypothesis was the strongest one and deserved the most care, because
**`load_we` had no test anywhere** — every other test seeds the RAM through
`code_init` at construction and never writes a chip at runtime. A load that
never landed would leave the power-on `init` in place and make all three test
codes produce the same sums, which is precisely the board symptom. It is now
covered at both levels, and it **passes**: the replica follows a code written at
runtime, a second load replaces the first, the subcarrier-select bit is written
beside the chip, and at bank level an all-ones and an all-zeros load produce
accumulators that are *exact negatives* of each other over a full integration.
So the write path is correct in simulation, and a missing load is not the fault.

**Where the arithmetic now points.** The same simulation gives a hard bound that
the earlier guesswork did not have. With `integrated_samples = 4000` per dump,
replica ±1 and carrier amplitude ≤ 127, a constant replica makes the accumulator
`amp × Σ sample`, so a DC offset *d* in the samples gives |sum| ≈ 508 000·*d*.
Simulation confirms it: at *d* = 80 an all-ones code accumulates 40 768 905, and
an all-zeros code −40 768 905. That is a factor of **52 below** the 2³¹ rail.

For the board to rail, therefore, one of these must be true:

- the samples arriving at the correlator are ~50× larger than the ones DMA0
  carries (std ≈ 80 measured), i.e. |sample| ≈ 4200; **or**
- roughly 50× more samples are being integrated per dump than one code period.

Both are in the **sample and accumulate path — not the replica**, which is where
§5.6b and the first version of this section pointed. The one change §5 made in
that path is §5.2's registered sample bundle (`SampleStreamRegister`, and the
rewiring of `soc.py` around it), and that rewiring has no SoC-level test.

**The prime suspect was our own change.** `SampleStreamRegister` and the
`soc.py` rewiring came in with §5.2 as one of the four pipeline stages; the only
v3 image ever flashed carried them, and the §2 build without them was never put
on silicon. So "the timing fix introduced the correlation bug" was a live
hypothesis and, with the replica path cleared, the leading one. That rewiring
had no test either; it now has one (`TestObserverRegisterBankChain`), and the
bank produces bit-identical records with and without the stage in both AD9361
channel modes, with and without DMA0 back-pressure. **The netlist query below
then cleared it in silicon too**, which simulation alone could not do.

**Interrogating the implemented netlist.** The `_route.dcp` checkpoint is the
exact netlist that became the flashed bitstream, so it answers "what did Vivado
actually build" without a rebuild and without the board. Open it with
`open_checkpoint` and query it (~2 min per query on this design):

| Question | Answer |
|---|---|
| Were the replica registers optimised away or tied constant? | **No.** `word_r`, `k_r`, `w_prev`/`w_cur`/`w_next`, `idx_far`, `lut_a` all present; 293 flops in `codereplica0`; every net `TYPE=SIGNAL`, none a constant |
| Did §5.2's pipeline stage survive? | **Yes, but not where you would look.** See below |

Two traps worth knowing before reading such a query:

1. **Migen names signals after the module *class*, not the instance.** The
   attribute is `self.gnss_rx_pipe`, but every net is `samplestreamregister_*`;
   `self.gnss_rx` becomes `rxsampleobserver_*`. A query for `*gnss_rx_pipe*`
   returns zero cells and looks alarming. The instance names appear only in the
   hierarchy *comment* at the top of the generated Verilog.
2. **A register that has vanished may have moved into a DSP.** Querying
   `*samplestreamregister*` finds exactly **one** flop — the strobe — and none
   for the 32 bits of `out_i`/`out_q`. That reads like a bundle delay whose data
   path lost its register while the strobe kept one, which would skew sample
   against strobe and is precisely the shape of fault being hunted. It is not.
   The sample-path DSP48E1s carry **`AREG = 1`** and their `A` pins are driven
   straight from `rxsampleobserver0/1`: Vivado absorbed the data flops into the
   DSP's own input register, which is exactly the placement §5.2 wanted.

The alignment was then checked rather than assumed, because the whole question
is whether both halves of the bundle are delayed equally:

```
trackingchannel0171      AREG=1  CEA1=<const0>  CEA2=<const1>  CLK=sys_clk
trackingchannel017_reg   AREG=1  CEA1=<const0>  CEA2=<const1>  CLK=sys_clk
samplestreamregister_out_stb_reg (FDRE)  CE=<const1>  R=ad9361_rx_cdc_cd_rst
```

One register on the data path and one on the strobe, same clock, both
unconditionally enabled. **§5.2 is correctly implemented in silicon**, which is
a stronger statement than the simulation test above and clears the leading
suspect at the level that matters.

**The next measurements, cheapest first.** The first two are single CSR reads
and settle it — but note that **they need the v3 image reflashed**. The
rolled-back 2026-07-29 gateware has `integrated_samples` too, and reading it
there measures the *working* build: a useful control, not the measurement.
There is no way to diagnose the failing build without putting it back on.

1. **Read `integrated_samples` on a dump.** ~4000 ⇒ the integration window is
   right and the samples are wrong; far more ⇒ the accumulator is not being
   cleared at the epoch. This one number splits the two branches above.
2. **Read the observer's sample registers** and compare their magnitude against
   a DMA0 capture taken at the same moment. A ~50× discrepancy names §5.2.
3. Only then bisect by building: `--max-subchips 1` first (it removes the whole
   `lut_a`/`lut_b` mux and the `nsub × subchips` multiply), then `--taps 3`.
4. Whatever the cause, add a CSR that reads the live `replica[t]` and the
   observer's sample. Everything above is inference from accumulator values; two
   readable registers would have made this minutes rather than a build cycle.

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

### 5.8 Three things that cost hours on the hardware side

*`flash_reload` wedges PCIe on this host.* After the ICAP reload the device
answers every config read with `0xff` and the kernel logs AER `CmpltTO`. A PCIe
`remove` + `rescan` does **not** recover it — the bridge is gone from
`/sys/bus/pci/devices` entirely. A host reboot does. So: flash, then reboot;
budget ~2 min, not the ~10 s `flash_reload` suggests.

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
