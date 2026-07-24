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

## Integration with litex_m2sdr (no fork)

The base SoC (PCIe, clocking, SI5351, time) is reused via the upstream
`add_rx_datapath_processing()` hook + configurable `pcie_dmas`
(enjoy-digital/litex_m2sdr#152). This repo's SoC subclasses `BaseSoC`, taps the
RX stream losslessly into the tracking block, and drives `pcie_dma1` with the
correlator dumps — DMA0's I/Q path is untouched.

## Status

- [x] GPS L1 C/A code generator — validated vs IS-GPS-200, autocorrelation/balance,
      and GNSSSignals.jl `gen_code` (all 32 PRNs, exact).
- [ ] Carrier NCO + complex mixer (carrier wipe-off)
- [ ] Code NCO + E/P/L tap generation
- [ ] E/P/L correlators + integrate-and-dump
- [ ] Correlator-dump record builder + FIFO + DMA1
- [ ] Full single-channel Migen simulation (lock on synthetic L1 C/A)
- [ ] SoC integration + hardware validation on orin2

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
