# Hardware bring-up on orin2 (LiteX-M2SDR + antenna)

End-to-end procedure to flash the GNSS tracking gateware, bring up the RF
front-end, and validate that the on-FPGA correlators lock onto a real GPS L1
C/A satellite. CSR access is pure-Python over the litepcie char device
(`software/m2sdr_csr.py`), so no litex install is required on the Orin.

Paths below assume the litex_m2sdr checkout at `~/litex_m2sdr` and this repo at
`~/gnss-m2sdr` on orin2.

## 0. Artifacts (built on the Vivado host)

- `build/gnss_m2sdr_m2_x1_ch4/gateware/gnss_m2sdr_m2_x1_ch4.bin`  (flash image)
- `build/gnss_m2sdr_m2_x1_ch4/csr.csv`                           (CSR map for the host)
- `build/gnss_m2sdr_m2_x1_ch4/software/include/generated/{csr,soc,mem}.h`
  (regenerate the M2SDR driver + tools so their base-peripheral CSR offsets
  match this gateware)

Copy them + this repo to orin2 (see `scripts/deploy_orin.sh`).

## 1. Rebuild the M2SDR driver + tools with this gateware's headers

```bash
cd ~/litex_m2sdr/litex_m2sdr/software
cp ~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch4/software/include/generated/{csr,soc,mem}.h kernel/
# user tools include the same headers
cd kernel && make clean all && sudo ./init.sh        # rebuild + load litepcie driver
cd ../user  && make clean all                         # rebuild m2sdr_util, m2sdr_rf, ...
```

## 2. Flash the gateware (multiboot operational slot) and reload

```bash
cd ~/litex_m2sdr/litex_m2sdr/software
./flash.py ~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch4/gateware/gnss_m2sdr_m2_x1_ch4.bin
# flash.py runs: m2sdr_util flash_write ... 0x00800000 ; flash_reload
sudo rmmod litepcie 2>/dev/null; sudo ./kernel/init.sh   # rescan/reload after reflash
./user/m2sdr_util info                                   # expect the new SoC identifier
```

Recovery: the golden/fallback image at offset 0x0 boots if the operational
image is bad; reflash a known-good `.bin` to recover.

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

## 5. Acquire + observe a satellite (FPGA sliding-correlator)

```bash
cd ~/gnss-m2sdr
PYTHONPATH=. python3 -c "
from software.m2sdr_csr import LiteXCSR
from software.gnss_tracking import GNSSChannel, GNSSBank, acquire
csr  = LiteXCSR('build/gnss_m2sdr_m2_x1_ch4/csr.csv')
fs   = 4_000_000
bank = GNSSBank(csr); chan = GNSSChannel(csr, fs, index=0)
best = acquire(chan, bank, prn=1, fs=fs)   # try PRNs known to be visible
print('best doppler/power/codephase:', best)
"
```

A clear prompt-power peak at a particular Doppler for a visible PRN is the
hardware-in-the-loop validation: the on-FPGA carrier/code NCOs + correlators
locked onto a real GPS satellite.

## 6. Closed-loop tracking (next)

Hold `carrier_freq`/`code_freq` at the acquired peak, then run the DLL/FLL/PLL
loop on the host: each dump -> discriminators -> NCO updates, and feed
`CorrelatorOutput` into Tracking.jl via `append_correlator_output!` +
`estimate_dopplers_and_filter_prompt!`. The lossless DMA1 record path
(record_format.py) replaces CSR polling once the kernel driver exposes the 2nd
DMA channel.
