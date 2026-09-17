# Hardware bring-up on orin2 (LiteX-M2SDR + antenna)

End-to-end procedure to flash the GNSS tracking gateware, bring up the RF
front-end, and validate that the on-FPGA correlators lock onto a real GPS L1
C/A satellite. CSR access is pure-Python over the litepcie char device
(`software/m2sdr_csr.py`), so no litex install is required on the Orin.

Paths below assume the litex_m2sdr checkout at `~/litex_m2sdr` and this repo at
`~/gnss-m2sdr` on orin2.

## 0. Artifacts (built on the Vivado host)

- `build/gnss_m2sdr_m2_x1_ch4_ant1_code1023/gateware/gnss_m2sdr_m2_x1_ch4_ant1_code1023.bin`  (flash image)
- `build/gnss_m2sdr_m2_x1_ch4_ant1_code1023/csr.csv`                           (CSR map for the host)
- `build/gnss_m2sdr_m2_x1_ch4_ant1_code1023/software/include/generated/{csr,soc,mem}.h`
  (regenerate the M2SDR driver + tools so their base-peripheral CSR offsets
  match this gateware)

Copy them + this repo to orin2 (see `scripts/deploy_orin.sh`).

## 1. Rebuild the M2SDR driver + tools with this gateware's headers

**Take the lock first, and keep the old headers.** `make clean` deletes
`m2sdr.ko` and `m2sdr_util` before it rebuilds them, so a build that fails
leaves the host with no working tools at all.

```bash
cd ~/litex_m2sdr/litex_m2sdr/software
mkdir -p ~/gnss-m2sdr/rollback/headers_before
cp kernel/{csr,soc,mem}.h user/csr.h ~/gnss-m2sdr/rollback/headers_before/

B=~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch4_ant1_code1023
cp $B/software/include/generated/{soc,mem}.h kernel/
# csr.h needs adapting -- see below
python3 ~/gnss-m2sdr/scripts/driver_headers.py $B/software/include/generated/csr.h kernel/csr.h
cp kernel/csr.h user/csr.h
cd kernel && make clean all && sudo make install && sudo ./init.sh
cd ../user  && make clean all                         # rebuild m2sdr_util, m2sdr_rf, ...
```

LiteX's generated `csr.h` cannot be copied in as-is: it `#include`s
`generated/soc.h`, `system.h` and `hw/common.h`, none of which exist in the
M2SDR software tree, and the build fails on the first file that pulls it in.
`scripts/driver_headers.py` strips those three includes and substitutes the
handful of accessors the tree expects. Restore
`~/gnss-m2sdr/rollback/headers_before/` and rebuild if anything goes wrong.

## 2. Flash the gateware (multiboot operational slot) and reload

```bash
cd ~/litex_m2sdr/litex_m2sdr/software/user
./m2sdr_util flash_write -y -c 0 \
  /home/orin/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch4_ant1_code1023/gateware/gnss_m2sdr_m2_x1_ch4_ant1_code1023.bin \
  0x00800000
./m2sdr_util flash_reload     # ICAP: makes the FPGA re-read the flash. REQUIRED.
sudo shutdown -r +0           # mandatory: flash_reload wedges PCIe
# after the host is back:
./m2sdr_util info             # expect the new SoC identifier
```

Three things that are not obvious and each cost an hour:

- **Call `m2sdr_util flash_write` directly, not `flash.py`.** `flash.py` builds
  its command as `cd user && ./m2sdr_util flash_write ... ../$bitstream`, so an
  **absolute** path turns into `..//home/orin/...` and the write fails.
- **Give the write no timeout.** 7 MiB takes ~80 s. A timeout that fires
  mid-erase leaves a half-written operational slot.
- **`flash_reload` is required, *and* the reboot after it is required.** A warm
  `shutdown -r` does not drop power to the M.2 card, so the FPGA keeps its
  current configuration and never re-reads the flash — a write that reported
  `Success.` then looks like it did nothing, because `m2sdr_util info` still
  shows the old SoC identifier. `flash_reload` is the ICAP reconfiguration that
  makes it re-read. It then wedges the PCIe link (every config read `0xff`, AER
  `CmpltTO`, the device gone from `/sys/bus/pci/devices`, and `remove` +
  `rescan` cannot bring it back), so the reboot is mandatory too. Sequence:
  **flash_write → flash_reload → reboot**, ~2 minutes.

### Recovery / rollback

`flash.py` writes the **operational** multiboot slot at `0x00800000` only. The
golden/fallback image at offset `0x0` is untouched by a normal flash and still
boots if the operational image is bad, so a bad operational image cannot brick
the board. (Verified against `litex_m2sdr_platform.py`: the fallback bitstream
is built with `BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x00800000` and written to `0x0`,
the operational one with `CONFIGFALLBACK Enable` and a watchdog `TIMER_CFG`.)

**Do not assume you know what is on the board.** The image that was actually
running here was *not* any of the release `.bin`s in this tree — it was a
20-channel build whose `.bin` had been deleted. The only rollback you can trust
is one you read back off the flash yourself, **before** you overwrite it:

```bash
cd ~/litex_m2sdr/litex_m2sdr/software/user
mkdir -p ~/gnss-m2sdr/rollback
./m2sdr_util flash_read -c 0 ~/gnss-m2sdr/rollback/op_slot_backup.bin 0x00800000 0x700000
md5sum ~/gnss-m2sdr/rollback/op_slot_backup.bin
# 8f04c9ecf12711efb76bf2a7dd700219  = what was on the board on 2026-09-17
```

Sanity-check the dump before trusting it: a valid bitstream has the `aa995566`
sync word near the start (offset `0x30` here) and a long `0xff` tail after the
content ends. To restore it:

```bash
cd ~/litex_m2sdr/litex_m2sdr/software/user
./m2sdr_util flash_write -y -c 0 /home/orin/gnss-m2sdr/rollback/op_slot_backup.bin 0x00800000
./m2sdr_util flash_reload && sudo shutdown -r +0
./m2sdr_util info        # expect the 2026-07-29 23:42:50 SoC identifier back
```

The `~/gnss-m2sdr/build/...` `.bin`s are still worth keeping as a second
fallback, but identify what you are restoring by SoC identifier and content
size, not by filename.

**Do not flash a bitstream that misses timing.** See
[gateware builds](gateware_builds.md): a design that fails setup does not
degrade gracefully, it produces non-deterministically wrong correlator results.

## 3. Configure the RF front-end for GPS L1

```bash
cd ~/litex_m2sdr/litex_m2sdr/software
# GPS L1 = 1575.42 MHz. Use ~4-8 MSPS and high RX gain for the weak signal.
./user/m2sdr_rf -samplerate 4000000 -rx_freq 1575420000 -rx_gain 60 -bandwidth 2000000
```

(Confirm the exact `m2sdr_rf` flag names with `./user/m2sdr_rf -h`; adjust
`-samplerate`. Note the sample rate `fs` — the host uses it for the NCO words.)

Either channel mode works. The RX observer follows the AD9361 PHY `mode` CSR:
in 2T2R (the default) one 64-bit RX word is one RX1 sample, in 1T1R
(`-chan_mode 1t1r`, which halves the DMA0 bandwidth for a single GNSS antenna)
the word's two slots are two consecutive samples and both are fed to the bank.
`fs` is the per-antenna sample rate either way.

## 4. Keep the RX sample stream flowing

The tracking bank is a non-intrusive observer on the RX stream, so it only sees
samples while DMA0 is draining. Run a continuous RX in the background:

```bash
./user/m2sdr_rx /dev/null &        # or the appropriate continuous-RX tool
```

## 5. Acquire on the CPU, track on the FPGA

**Acquisition belongs on the host.** The FPGA does downconversion and
correlation for *tracking*, and the loop filters run CPU-side too. Do not sweep
for satellites through the FPGA correlator: `software/gnss_tracking.py:acquire()`
scores peak/median of prompt power, whose noise baseline is the same order as a
real 1 ms peak, and measured against Acquisition.jl it fires on every PRN at
every Doppler (see [gateware builds](gateware_builds.md) 5.7b for the numbers).

### 5.1 CPU-acquire from the raw DMA0 stream

Capture and run Acquisition.jl -- the project's own, already a GNSSM2SDR.jl
dependency. The capture is sc16, 2T2R, **8 bytes per sample instant**, with RX1
in the first two `Int16` of each 4-word group:

```julia
raw   = reinterpret(Int16, read(CAPTURE_FILE))
words = reshape(view(raw, 1:4(length(raw) ÷ 4)), 4, :)
signal = ComplexF32.(Float32.(view(words, 1, :)), Float32.(view(words, 2, :)))

results = acquire(GPSL1CA(), signal, 4e6Hz, collect(1:32);
                  min_doppler_coverage = 50_000.0Hz,
                  num_coherently_integrated_code_periods = 10,
                  num_noncoherent_accumulations = 10)
```

`min_doppler_coverage` of 50 kHz is not optional. The device TCXO is poor -- 1
ppm at L1 is 1.575 kHz -- and satellites have been observed at -8.5 kHz, outside
any window sized for satellite motion alone.

Read the result as a distribution, not a threshold: the noise floor is tight
(32 PRNs with a median-absolute-deviation of 0.25-0.42 dBHz in measured
captures), and a satellite stands 6-20 dB clear of it. If most of the
constellation "detects", the threshold is in the noise.

### 5.2 Hand over to an FPGA channel and close the loop

`~/hwloop/closed_loop_multi.jl` on orin2 does all three steps and is the
reference: CPU-acquire, sweep the code phase to refine the handover, commit it,
then feed every FPGA dump into Tracking.jl. A good run holds prompt power tens
of times the noise floor with a carrier Doppler that agrees with the CPU
estimate:

```
   t(s)   PRN19 |P|^2/fl  carr(Hz)   PRN15 |P|^2/fl  carr(Hz)
    1.0            29.11   -1596.6            26.19   -5221.8
   20.0            27.83   -1591.9            53.21   -5237.3
   39.0            25.62   -1614.2            53.64   -5260.5
```

A channel that reads ~1x floor with a carrier running away by tens of kHz never
locked -- that is what a failed handover looks like, and it is unambiguous.

**DMA0 must keep draining throughout.** The tracking bank is a non-intrusive
observer on the RX stream, so it sees nothing unless something is reading DMA0,
and `m2sdr_record`'s byte-count argument is a hard limit that silently ends the
capture when reached.

## 6. Next: the lossless record path

Step 5 polls the correlator CSRs, and that readback is lossy -- a dump can be
replaced before it is read, which is why a 40 s run folds ~40 000 dumps and
skips ~19 700 of them. The DMA1 record path (`record_format.py`) has neither
problem and replaces CSR polling once the kernel driver exposes the second DMA
channel. It has not yet been exercised on hardware.
