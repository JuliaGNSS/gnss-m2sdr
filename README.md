# gnss-m2sdr

[![tests](https://github.com/JuliaGNSS/gnss-m2sdr/actions/workflows/tests.yml/badge.svg)](https://github.com/JuliaGNSS/gnss-m2sdr/actions/workflows/tests.yml)

On-FPGA GNSS downconversion + correlation for the [LiteX-M2SDR](https://github.com/enjoy-digital/litex_m2sdr)
board. Companion to [GNSSReceiver.jl#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107).

## Concept

Acquisition stays on the CPU; **signal tracking runs on the FPGA**:

1. Every ~10 s the SDR streams ~10 ms of raw IQ to the host (existing DMA0 path)
   for PRN acquisition.
2. The host feeds back code phase, carrier phase and their Dopplers to the FPGA.
3. The FPGA runs the code + carrier NCOs and correlates continuously, emitting one
   correlator dump per code period per channel.
4. Dumps stream losslessly to the host over a dedicated DMA channel; the host runs
   the tracking loops (Doppler/NCO-update estimation) and writes NCO updates back.

The dumps map onto `Tracking.jl`'s `CorrelatorOutput(correlator,
integrated_samples, sample_index)` external-producer contract
(JuliaGNSS/Tracking.jl #205, #207). The record's `code_phase` is *additional
device-side metadata that Tracking.jl does not currently consume*: as of
Tracking.jl v4.1.1 (with #207 merged) the struct has exactly those three fields
and no `code_phase` keyword constructor exists, despite what #207's description
advertises. We keep the field because the CPU side needs it for acquisition
handover and downstream vector tracking.

> **Accumulator order.** `EarlyPromptLateCorrelator.accumulators` is ordered
> *latest first* — `[late, prompt, early]`, since `get_prompt_index` is 2, late
> is index 1 and early is index 3. The host glue must therefore build
> `EarlyPromptLateCorrelator(SVector(late, prompt, early), spacing)`, i.e. the
> reverse of the record's `prompt, early, late` word order (see
> `gnss_m2sdr/record_format.py`). Swapping E and L inverts the sign of the DLL
> discriminator and the loop never converges.

> **Tap offsets.** Tracking.jl quantises its preferred code shifts to whole
> numbers of input samples (`get_correlator_sample_shifts`) and its
> discriminators recover the spacing from the correlator they are handed. The
> host therefore programs each tap as `sample_shift * code_step`, not as the raw
> preferred chip shift (`GNSSChannel.set_tap_offsets`) — at fs = 4 MHz and 0.5
> chips the raw value is a ~2.3 % DLL loop-gain error. There is one register per
> tap and no "spacing": a five-tap layout is not describable by one number, which
> is why the contract hands over the whole `tap_sample_shifts` array.

> **Antennas.** Beamforming in GNSSReceiver.jl is *post-correlation* on the CPU
> (`EigenBeamformer`, adapting from the per-antenna prompt covariance), so the
> device streams **per-antenna** accumulators — one E/P/L block per antenna in
> every record, `T = SVector{N,Complex}` on the host. The replica generation
> (carrier NCO, code NCO, three code RAMs) is shared across antennas, since they
> all track the same signal and only the spatial phase differs; `NCOUpdate` stays
> one per channel. N ≤ 2 on one board: the AD9361 is 2T2R and its shared LO is
> what makes the two RX chains phase-coherent. Larger arrays need
> phase-synchronised multi-board setups and are out of scope.

## Integration with litex_m2sdr (no fork)

The base SoC (PCIe, clocking, SI5351, time) is reused via the upstream
`add_rx_datapath_processing()` hook + configurable `pcie_dmas`
(enjoy-digital/litex_m2sdr#152). This repo's SoC subclasses `BaseSoC`, taps the
RX stream losslessly into the tracking block, and drives `pcie_dma1` with the
correlator dumps — DMA0's I/Q path is untouched.

`BaseSoC` lives in litex_m2sdr's top-level `litex_m2sdr.py` script, so it is
loaded by file path rather than imported. Point `LITEX_M2SDR_DIR` at your
checkout; without it, `~/litex_m2sdr` and a checkout sitting next to this repo
are tried, and the error lists every path attempted.

```
export LITEX_M2SDR_DIR=/path/to/litex_m2sdr
```

## Status

- [x] GPS L1 C/A code generator — validated vs IS-GPS-200, autocorrelation/balance,
      and GNSSSignals.jl `gen_code` (all 32 PRNs, exact).
- [x] Carrier NCO + sin/cos LUT (SinCosLUT.jl amplitude convention) — matches ideal
      within quantization; frequency and phase-set verified.
- [x] Code NCO + multi-tap replica with **independently placed taps** — prompt
      reproduces the code, epoch period exact, each tap lands exactly on the
      whole-sample offset it was programmed with.
- [x] Correlators + integrate-and-dump (`TrackingChannel`), 3 or 5 taps.
- [x] Full single-channel Migen simulation: locks on a synthetic L1 C/A signal
      (prompt peaks, E/L balanced, DLL discriminator sign correct, wrong-PRN rejects).
- [x] Correlator-dump record builder + FIFO + DMA1 (record.py, record_format.py)
- [x] Multi-channel bank + CSR control (bank.py: carrier/code freq words, per-tap
      offsets, runtime PRN code load, per-channel dump readback)
- [x] Deterministic apply point: NCO updates and acquisition handover (carrier
      freq/phase + code freq/phase) commit atomically on a host-chosen sample
      index (`apply_at`), giving `NCOUpdate.apply_at_epoch` a hardware meaning
      and a fixed feedback delay instead of PCIe jitter.
- [x] SoC integration via litex_m2sdr#152 hook (soc.py, pcie_dmas=2, RX observer)
- [x] Host software: pure-Python CSR access (ioctl) + sliding-correlator acquisition
- [x] Periodic **epoch-strobe records** (`gnss_epoch_period` CSR): a timebase marker
      on the shared sample counter, so the host closes epochs even with no channel
      locked (GNSSReceiver.jl#107).
- [x] **Runtime signal configuration** (`docs/signal_configuration.md`): per-channel
      primary-code length (330 … `--max-code-length`, default 1023) and chipping
      rate, committed atomically with the code load and the code phase by the
      arming `restart`; an unrepresentable rate (≥ 1 chip/input sample) is
      reported (`gnss_rate_error`) and stops that channel's records instead of
      being truncated into a plausible one. Records carry the **complete** code
      phase (chip *and* fraction), the code length and the code step, plus a
      format version and a tap count; `gnss_version` / `gnss_capabilities` /
      `gnss_signal_caps` let the host discover the build instead of assuming it.
- [x] **Multi-antenna (N≤2, the AD9361's 2T2R limit)**: `num_ants` per channel —
      one carrier/code NCO and one replica set shared, `num_ants × 2 × num_taps`
      accumulators, one block per antenna in the record (`--num-ants 2`).
- [x] **Sub-chip modulation and five-tap correlation**
      (`docs/subchip_modulation.md`): per-channel BOC(1,1) (sine and cosine),
      Galileo E1 CBOC — amplitude-bearing, at GNSSSignals' own (19, 6) integer
      table, so `replica_code_amplitude` needs no override — and GPS L1C TMBOC,
      all checked sample for sample against a GNSSSignals.jl golden reference.
      Very Early / Very Late taps at independent whole-sample offsets, reported
      in the four record words version 2 reserved, with `num_taps` **per
      channel** so one bank runs GPS L1 C/A on three taps next to Galileo E1 on
      five. `GalileoE1B_BOC11` and friends stay separate signals, never a silent
      substitution for CBOC.
- [x] **Hardware validation on orin2: on-FPGA correlators acquired a live GPS
      satellite (PRN 24, peak/median >> 100) from the antenna.**

### Hardware notes (learned bringing this up on a Jetson Orin)

- Requires a litex_m2sdr gateware whose SI5351 uses litei2c **before** commit
  `ce0bb5d` (the stuck-low bus-error check spuriously fails on this board over
  PCIe). This repo pins litei2c to `19417d6`. Symptom otherwise: `m2sdr_rf`
  fails at `SI5351 SYS_INIT ... status 0x00`.
- The RX observer only sees samples while DMA0 is draining — run a continuous
  `m2sdr_record /dev/null &` during tracking.
- On the Orin the PCIe device is `0004:01:00.0`; after `flash_reload`, re-enumerate
  manually (rescan.py mis-formats the domain): `rmmod m2sdr; echo 1 >
  /sys/bus/pci/devices/0004:01:00.0/remove; echo 1 > /sys/bus/pci/rescan; modprobe m2sdr`.

## Layout

```
gnss_m2sdr/gateware/   Migen/LiteX gateware (ca_code.py, ...)
docs/                  record path, hardware bring-up, signal configuration
test/                  Migen simulations + software-reference tests
test/data/             committed golden vectors (e.g. GNSSSignals.jl C/A codes)
julia/                 GNSSSignals.jl project used to regenerate golden vectors
```

## Running tests

Everything is pure simulation — no board, no Vivado, no numpy:

```
pip install -r requirements-test.txt            # migen + LiteX + LitePCIe, pinned
PYTHONPATH=. python test/run_all.py             # the whole suite (auto-discovers test/test_*.py)
PYTHONPATH=. python -m unittest test.test_record -v   # a single module
```

`run_all.py` exits non-zero on failure and is what CI runs
(`.github/workflows/tests.yml`) on every push and pull request.

**Toolchain.** `requirements-test.txt` pins the exact commits CI runs: migen
0.9.2 (`e19524c`), LiteX 2026.4 (`93c8d23`), LitePCIe 2026.4 (`e84e0b9`). One
upstream quirk is worth knowing about: migen's *simulator* cannot lower the
write-only `Memory` port that `stream.SyncFIFO` creates, so every simulation
containing the recorder's FIFO dies with `AttributeError: 'NoneType' object has
no attribute 'eq'` before its first cycle. `test/migen_compat.py` patches that on
import — simulation only, gateware generation is untouched — which is why the
suite must be started through `test/run_all.py` or `python -m unittest
test.<module>` and not by running a test file as a bare script.
