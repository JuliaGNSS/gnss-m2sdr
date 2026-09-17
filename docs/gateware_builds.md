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

## 3. What this means

- **That bitstream must not be flashed.** A design that misses setup by 2.9 ns
  does not "mostly work" — it produces wrong correlator results
  non-deterministically, which is worse than not flashing at all.
- The fix is a **pipeline stage**, not a smaller build: register the code RAM
  read (or the replica product). It does not even have to cost a cycle of
  latency — see §5.

A shorter code does **not** rescue it. The same configuration at
`--max-code-length 1023` (a 16:1 LUTRAM output mux instead of 160:1) still
misses, at **WNS −1.288 ns**, and on a *different* path: LitePCIe's DMA0 writer
FIFO level → `rx_stream.ready` → the RX observer's output mux → the A input of
the carrier wipe-off DSP48E1 → its PCOUT/PCIN cascade. Two independent paths
were over budget, so trimming the configuration was never going to be enough.

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

## 5. Closing it: three pipeline stages, no extra latency

Three paths were over budget, and they had to be fixed in that order because
each one hid the next. None of them needed the configuration to shrink, and none
of them cost a cycle of correlator latency: every stage below is computed from a
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

### 5.6c What the next person should try

The simulation suite proves the replica is right *functionally*, and timing
closes, so the gap is something neither covers. In rough order of likelihood:

0. **Establish whether the code RAM is being read at all, or merely not being
   written.** The three loads above produce *identical* sums, which has two
   economical explanations and they need different fixes: either the replica
   never reaches the DSP `B` input (a broken window/tap register), or
   `load_we` never lands in the RAM, so all three "different" codes left the
   power-on `init` in place — in which case the outputs are identical because
   the memory contents were. The discriminator is free: each channel's RAM is
   initialised with *its own* PRN (`prn=i+1`), so read the prompt accumulator of
   channel 0 and channel 1 under identical settings. Different sums ⇒ the RAM is
   read and the *write* path is the bug; identical sums ⇒ the replica is not
   reaching the multiplier.
1. **Build the same RTL with `--max-subchips 1`** (`replica_bits` drops from 8 to
   2, and the whole `lut_a`/`lut_b` `Array` mux and the `nsub * subchips`
   multiply disappear). If that correlates, the bug is in the registered
   sub-chip index `k_r` or the subcarrier-table mux, not in the chip window.
   This is the cheapest discriminator and it isolates §5.4 from §5.1.
2. **Then `--taps 3`**, to separate the five-tap fan-out from the replica logic.
3. Check the `Array(...)[k_r]` mux for an out-of-range index: `k_r` is
   `bits_for(max_subchips - 1)` = 4 bits for `max_subchips = 12`, so it can
   address 16 entries of a 12-entry `Array`. Migen lowers an `Array` read to a
   combinational `if/elif` chain with **no final `else`**, which is total in
   simulation (the target keeps its previous value, and it is re-evaluated every
   delta) but is an incompletely-specified combinational assignment in Verilog —
   the classic latch-inference shape. With `subchips = 1` the index should be 0
   and the first branch should always hit, so this is unlikely to be the fault,
   but it is the one construct in the new code whose hardware and simulation
   semantics are not identical, and it is worth an explicit `else` regardless.
   Note that none of the sub-chip machinery has ever run on hardware: the image
   the board has been running is a three-tap, `subchips = 1`, 1023-chip build.
4. Add a CSR that reads the live `replica[t]` (or `word_r`/`k_r`) for one
   channel. Everything above is inference from accumulator values; one readable
   register would have made this a five-minute diagnosis instead of an hour of
   bisection by correlator output.

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

**Confirmed on the board after the reboot** (2026-09-17 15:0x UTC): the SoC
identifier reads *built on 2026-07-29 23:42:50* again, the sample stream runs at
4 002 068 samples/s against fs = 4 MHz, and GPS L1 C/A acquires on **ten of ten
PRNs tried** — 1, 3, 8, 14, 17, 19, 21, 22, 28, 32, peak/median 16.8 to 39.6,
Dopplers between −4000 and +3500 Hz. The regression baseline is intact.

One trap on the way there, worth writing down: the acquisition sweep reads the
correlators over CSR, but the bank only sees samples while DMA0 is draining, so
`m2sdr_record` has to outlive the whole sweep. Its byte limit is not a hint —
`m2sdr_record /dev/null 100000000` exits after ~6 s at 4 MSPS, and every PRN
after that returns **metric exactly 0.00** with no error anywhere. Give it a
limit that cannot be reached (`100000000000`) and check the sample counter is
still advancing when the sweep ends. Note also that the CSR map of the image on
the board predates #31: driving it needs the host code from commit `8220716`,
not this branch's.

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
