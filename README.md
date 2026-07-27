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

> **E/L spacing.** Tracking.jl quantises the preferred Early/Late chip shift to a
> whole number of input samples (`get_correlator_sample_shifts`) and `dll_disc`
> normalises with that quantised spacing. The host therefore programs the spacing
> CSR as `sample_shift * code_step`, not as the raw preferred chip shift
> (`GNSSChannel.spacing_word`) — at fs = 4 MHz and 0.5 chips the raw value is a
> ~2.3 % DLL loop-gain error. The Julia glue must quantise the same way and hand
> the correlator the same integer sample shift it programmed.

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
- [x] Code NCO + E/P/L replica with **runtime-configurable spacing** — prompt
      reproduces the code, epoch period exact, E leads / L trails by the spacing.
- [x] E/P/L correlators + integrate-and-dump (`TrackingChannel`).
- [x] Full single-channel Migen simulation: locks on a synthetic L1 C/A signal
      (prompt peaks, E/L balanced, DLL discriminator sign correct, wrong-PRN rejects).
- [x] Correlator-dump record builder + FIFO + DMA1 (record.py, record_format.py)
- [x] Multi-channel bank + CSR control (bank.py: carrier/code freq words, spacing,
      runtime PRN code load, per-channel dump readback)
- [x] Deterministic apply point: NCO updates and acquisition handover (carrier
      freq/phase + code freq/phase) commit atomically on a host-chosen sample
      index (`apply_at`), giving `NCOUpdate.apply_at_epoch` a hardware meaning
      and a fixed feedback delay instead of PCIe jitter.
- [x] SoC integration via litex_m2sdr#152 hook (soc.py, pcie_dmas=2, RX observer)
- [x] Host software: pure-Python CSR access (ioctl) + sliding-correlator acquisition
- [x] Periodic **epoch-strobe records** (`gnss_epoch_period` CSR): a timebase marker
      on the shared sample counter, so the host closes epochs even with no channel
      locked (GNSSReceiver.jl#107).
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
