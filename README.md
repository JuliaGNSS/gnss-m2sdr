# gnss-m2sdr

[![tests](https://github.com/JuliaGNSS/gnss-m2sdr/actions/workflows/tests.yml/badge.svg)](https://github.com/JuliaGNSS/gnss-m2sdr/actions/workflows/tests.yml)

GNSS downconversion and correlation on the FPGA of the
[LiteX-M2SDR](https://github.com/enjoy-digital/litex_m2sdr).

The gateware is a time-multiplexed (TDM) correlator: one pipelined datapath
steps every channel once per input sample, reading each channel's state from a
RAM and writing it back. It needs 10 DSPs at any channel count. The host
acquires on the raw samples, arms channels at an exact device sample, reads
the correlator records off a second DMA channel and writes the NCO words back.
[GNSSM2SDR.jl](https://github.com/JuliaGNSS/GNSSM2SDR.jl) is that host side,
and [GNSSReceiver.jl](https://github.com/JuliaGNSS/GNSSReceiver.jl)'s
`receive(::HardwareLoop, …)` is the receiver on top of it.

The design, the host interface, the record format and the verification are
described in [docs/tdm_correlator.md](docs/tdm_correlator.md).

## Status

- GPS L1 C/A and Galileo E1B, 24 channels at 4 MS/s, up to five taps per
  channel. The replica is the code times a per-channel sub-chip table of up
  to 12 entries and two amplitudes, so Galileo E1's CBOC is replicated as
  GNSSSignals' Int8 code table has it (±25/±13), the one Tracking.jl
  correlates with ([the replica](docs/tdm_correlator.md#the-replica)).
- `gnss_tdm_m2_x1_ch24_code4096` meets timing at upstream's default
  directives (WNS +0.118 ns at 125 MHz; 10 DSP48E1, 11.7 % of the LUTs).
- On a board with an antenna (2026-10-16), every one of 3 890 hardware records
  matched the bit-exact model; on the current build (2026-10-19), all 4 313. GNSSReceiver.jl tracked and decoded up to eight
  satellites and computed position fixes from them.
- Not yet: TMBOC (GPS L1C pilot), codes longer than the code memory, more than one antenna per channel or more
  than one band. See [Not yet](docs/tdm_correlator.md#not-yet).

## litex_m2sdr

gnss-m2sdr is not a fork of litex_m2sdr. It uses an unmodified upstream
checkout as a dependency:

- `gnss_m2sdr/tdm/soc.py` subclasses litex_m2sdr's `BaseSoC` and puts the
  correlator on its RFIC datapath hook, `add_rfic_datapath()`
  (enjoy-digital/litex_m2sdr#185), with a second PCIe DMA channel
  (`pcie_dmas=2`) for the records.
- It is pinned as a git submodule at `third_party/litex_m2sdr`, currently
  `main` at [`973669b`](https://github.com/enjoy-digital/litex_m2sdr/commit/973669b)
  (2026-10-15). The same checkout provides the gateware's `BaseSoC` and the
  kernel module and user tools the host rebuilds, so both match.
- `BaseSoC` lives in litex_m2sdr's top-level `litex_m2sdr.py` script, which
  its `setup.py` does not install, so it is loaded by file path. Set
  `LITEX_M2SDR_DIR` to build against another checkout, such as an upstream
  branch; a checkout without the hook is refused.
- The host needs litex_m2sdr's kernel module with per-DMA-channel ring
  geometry (#187), so DMA1 can run small buffers with an interrupt per buffer.

The bitstream above was built with Vivado 2024.1 and the LiteX ecosystem as of
2026-10-15 (litex `faff1b21e`, litepcie `586d7a4`, litei2c `73ac32e`, migen
`4c2ae8d`). To move the pin, check out the new commit in the submodule, rebuild,
and commit the submodule.

## Building and bring-up

```sh
git clone --recurse-submodules https://github.com/JuliaGNSS/gnss-m2sdr
# or, in an existing clone: git submodule update --init
python build_tdm.py --build      # --channels, --code-words
```

This writes the bitstream, `csr.csv` and the kernel and user headers to
`build/gnss_tdm_m2_x1_ch24_code4096/`. Then:

1. Rebuild the kernel module and user tools in
   `third_party/litex_m2sdr/litex_m2sdr/software/` with
   `build/<name>/software/kernel/{csr,soc,mem}.h`. The headers shipped in
   litex_m2sdr describe upstream's own gateware, and tools built with them
   hang on this one.
2. Load the module with a small ring for DMA1:
   `options m2sdr dma_buffer_size=8192,512 dma_buffer_per_irq=8,1`.
3. Write `*_operational.bin` to the operational flash slot (0x800000), reload
   the FPGA and reboot the host.
4. Configure the RF front end:
   `m2sdr_rf --sample-rate 4e6 --bandwidth 4e6 --rx-freq 1575.42e6 --rx-gain 60 --channel-layout 2t2r`.

GNSSM2SDR.jl's README continues from there: the loop process and the receiver.

## Layout

```
gnss_m2sdr/tdm/   the gateware: params.py (formats, record layout), model.py
                  (bit-exact Python model), engine.py (engine, CSRs, record
                  serialiser, RX tap), soc.py (the SoC on litex_m2sdr)
build_tdm.py      the build
third_party/      litex_m2sdr (submodule)
test/             Migen and xsim tests against the model
docs/             the design
```

## Tests

```sh
pip install -r requirements-test.txt     # migen + LiteX, pinned
PYTHONPATH=. python test/run_all.py      # what CI runs
PYTHONPATH=. python -m unittest test.test_tdm_engine -v
```

- `test_tdm_engine` runs the engine in the Migen simulator against the model,
  record for record. `test_tdm_rxtap` checks the RX tap under a stalling host.
- `test_tdm_xsim` runs the engine scenarios on the Verilog LiteX writes, in
  Vivado's xsim, and checks that Verilog for two lowering faults that have
  reached silicon before. It finds xsim on `PATH`, in `$XILINX_VIVADO/bin` or
  through `$XILINX_ENV` (a wrapper that sets up Vivado's environment), and
  skips without it, as in CI.
- The tests do not use litex_m2sdr, so CI does not fetch the submodule.
- `requirements-test.txt` pins migen `e19524c` and LiteX `37b75bd4`; it says
  why. migen's simulator cannot lower the write-only `Memory` port that
  `stream.SyncFIFO` creates, and `test/migen_compat.py` patches that for
  simulation only. Start tests through `test/run_all.py` or
  `python -m unittest test.<module>`, not as bare scripts, so the patch is
  installed.

## History

An earlier design instantiated one complete tracking channel per satellite
(`gnss_m2sdr/gateware/`, `build.py`). It supported BOC-family modulations and
two antennas per channel, but needed 56 DSPs for four channels, closed timing
at six only with directive tuning, and was built on a litex_m2sdr hook that
upstream did not merge. It was retired. Its last version, with its host
software and bring-up notes, is at
[`c5c6085`](https://github.com/JuliaGNSS/gnss-m2sdr/tree/c5c60855666580e0dd8d934fa3a46970aa88c62e).
