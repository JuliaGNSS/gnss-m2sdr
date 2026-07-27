# gnss-m2sdr

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

The dumps map 1:1 onto `Tracking.jl`'s `CorrelatorOutput(correlator,
integrated_samples, sample_index; code_phase)` external-producer contract
(JuliaGNSS/Tracking.jl #205, #207).

> **Accumulator order.** `EarlyPromptLateCorrelator.accumulators` is ordered
> *latest first* — `[late, prompt, early]`, since `get_prompt_index` is 2, late
> is index 1 and early is index 3. The host glue must therefore build
> `EarlyPromptLateCorrelator(SVector(late, prompt, early), spacing)`, i.e. the
> reverse of the record's `prompt, early, late` word order (see
> `gnss_m2sdr/record_format.py`). Swapping E and L inverts the sign of the DLL
> discriminator and the loop never converges.

## Integration with litex_m2sdr (no fork)

The base SoC (PCIe, clocking, SI5351, time) is reused via the upstream
`add_rx_datapath_processing()` hook + configurable `pcie_dmas`
(enjoy-digital/litex_m2sdr#152). This repo's SoC subclasses `BaseSoC`, taps the
RX stream losslessly into the tracking block, and drives `pcie_dma1` with the
correlator dumps — DMA0's I/Q path is untouched.

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
- [x] SoC integration via litex_m2sdr#152 hook (soc.py, pcie_dmas=2, RX observer)
- [x] Host software: pure-Python CSR access (ioctl) + sliding-correlator acquisition
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

```
PYTHONPATH=. python test/test_ca_code.py
PYTHONPATH=. python test/test_ca_code_vs_gnsssignals.py
```
