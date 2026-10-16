# The TDM correlator

`gnss_m2sdr/tdm/` is a second gateware design for the LiteX-M2SDR: one
pipelined correlator datapath, time-multiplexed over every channel, instead of
one correlator instance per channel. It is what the hardware-correlator
receiver (GNSSReceiver.jl `receive(::HardwareLoop, …)`, GNSSM2SDR.jl's
`M2SDRLoop`) drives.

## Why time-multiplexed

At 4 MS/s and a 125 MHz system clock there are 31 clock cycles per input
sample. The per-channel bank (`gnss_m2sdr/gateware/bank.py`) used one of them
per channel and spent the rest idle, with a complete datapath — carrier NCO and
table, code RAMs, five correlators, DSP cascades — per channel. Six channels
closed timing only at `--timing-effort max` with directive tuning, 56 DSP48E1
for four channels ([gateware builds](gateware_builds.md)).

The TDM engine runs the datapath once per channel per sample: a round of
`max(n_channels + 1, 11)` cycles per sample, one slot per channel plus one for
the epoch strobe. Each slot reads the channel's state from a RAM, advances it
through an eight-stage pipeline and writes it back before the channel's next
slot. Every register of the datapath is a pipeline stage, so the design is
easy on timing, and its size barely depends on the channel count:

| build | channels | LUTs (whole SoC) | DSP48E1 | BRAM tiles | WNS (sys_clk) |
|---|---:|---:|---:|---:|---:|
| `gnss_tdm_m2_x1_ch24_code4096` | 24 | 15 178 (11.3 %) | **4** | 72.5 | +0.134 ns (worst path in litex_m2sdr's AD9361 BFP8 block; the engine's own worst path +0.23 ns) |

built with upstream litex_m2sdr at `973669b` and the directives of upstream's
own build (`build_tdm.py`), in about seven minutes.

Capacity: a round of 26 cycles (25 slots and one cycle to load the next
sample) is 4.8 MS/s at 125 MHz — 20 % headroom over 4 MS/s, used to drain the
sample FIFO after a burst. Higher sample rates need fewer slots per round, a
second engine, or the engine in a faster clock domain; the datapath has no
cross-slot dependency, so all three are straightforward.

## The datapath

Per slot (`engine.py`, stages S0–S7, mirrored operation for operation by
`model.py`):

1. **S0–S1** read the channel's state (block RAM), its NCO words, its static
   configuration and its pending arm (distributed RAM), select the antenna.
2. **S2** decides a pending arm: start at the staged sample, or mark it missed
   if that sample has passed. Adds the carrier and code NCO increments.
3. **S3–S4** the shared carrier table (64 entries, ±127) and the code memory
   (32-chip words, block RAM, one memory for all channels); the code-chip
   carry and the wrap at the code length.
4. **S4–S5** the carrier wipe-off: four 12 × 8-bit products — the only four
   DSPs of the engine.
5. **S5** the replica history: the code NCO runs on the *earliest* tap, and
   each sample pushes that tap's chip, and whether its code period ended, into
   a 32-entry history. Tap `k` with sample shift `s` reads entry `lead - s`;
   the prompt's period ends where entry `lead` carries the wrap flag. No tap
   needs its own code read or multiplier.
6. **S6** five tap pairs accumulated (33 bits), **S7** saturated to 32 bits,
   written back; at the prompt's period end the record is emitted and the
   accumulators restart.

Number formats are in `params.py`: 32-bit carrier phase and signed frequency
word, a 20-bit chip index with a 32-bit fraction and an unsigned code word
(chips per sample × 2³²), 12-bit samples, 32-bit accumulators.

## The host interface

- **Commands** (`gnss_cmd`): `WORD` loads the staged carrier and code
  frequency words for a channel in one cycle, applied from that channel's next
  sample; `ARM` loads a whole assignment (code memory base and length, tap
  offsets, antenna, an assignment id, the NCO words and phases) to start at a
  staged sample; `RELEASE` stops a channel. Command races with the engine are
  resolved by a per-channel generation counter: the host's write always wins.
- **Code memory** (`gnss_code_addr`, `gnss_code_data`): 32-chip words written
  with an auto-incrementing address; the host keeps a code per slot and points
  an arm at it.
- **Flags** (`gnss_enabled`, `gnss_pending`, `gnss_missed`): bitmaps the
  driver confirms its arms from.
- **Records** on DMA1: 64-byte blocks — magic, kind (dump / strobe / pad),
  channel, tap count, assignment id, the sample index one past the last sample
  integrated, the integrated count, the earliest tap's code phase, and five tap
  pairs. The serialiser pads a part-filled DMA buffer once its first record is
  `gnss_flush_cycles` old, so with the kernel module's per-channel geometry
  (`dma_buffer_size=8192,512 dma_buffer_per_irq=8,1`, upstream #187) a record
  reaches the host within ~0.1 ms of its end. Epoch strobes
  (`gnss_strobe_period`) keep the loop's epoch clock moving.

## The RX tap

`RXTap` sits on the RFIC datapath hook between the AD9361 and the host path.
It **never stalls the RFIC**: the AD9361 PHY free-runs and drops samples under
backpressure, so a correlator on the backpressured path loses samples whenever
the host is not reading DMA0 (or reads it slowly) — and with them the meaning
of its sample index. The tap feeds the correlator every sample and gives the
host its copy through a FIFO; what the host path cannot take is dropped from the
host's copy only (`gnss_tap_host_drops`).

With `gnss_control.tag` set, each host word carries its device sample index in
the four sign nibbles the 12-bit samples leave free: the low 12 bits in three
of them, and in the fourth one nibble of the index's upper bits, chosen by
`index[8:12]`, so that every aligned 4096-sample block spells out the whole
index. The receiver therefore knows which device sample each raw sample is —
the reference every acquisition handover is stated against — and sees a lost
DMA buffer as a hole in the tags.

## Verification

- `test/test_tdm_engine.py`: the engine in the Migen simulator against the
  bit-exact model, record for record, with arms, missed starts, words,
  releases and a stalling DMA.
- `test/test_tdm_xsim.py`: the same scenario on the Verilog **LiteX's
  converter writes** (the artefact the bitstream is built from), in Vivado's
  xsim, plus checks that no negative literal and no signed `.init` entry is in
  it — the two lowering faults that reached silicon with the per-channel bank
  ([gateware builds](gateware_builds.md) §5.6e, §5.6g).
- `test/test_tdm_rxtap.py`: the tap under a stalling host — the correlator
  sees every sample on consecutive indices, the host's tags name its own
  samples, the drop count is exact.
- On the board (orin1, 2026-10-16): with a channel armed inside a capture,
  every one of 3 890 hardware records matched the emulator (`M2SDRLoop`'s
  `TDMEmulator`, a port of the model) fed the same captured samples, bit for
  bit; the trimmed `gnss_loop` closed the loops on live satellites at
  41–45 dB-Hz with bit sync, 0 bytes allocated and records at most 0.6 ms old
  at the fold.

## Bring-up on a board

1. `LITEX_M2SDR_DIR=../litex_m2sdr python build_tdm.py --build`.
2. Rebuild litex_m2sdr's kernel module and user tools with the build's
   `software/kernel/{csr,soc,mem}.h`; load the module with
   `dma_buffer_size=8192,512 dma_buffer_per_irq=8,1`.
3. Write `*_operational.bin` to the operational slot (0x800000), ICAP-reload,
   reboot the host.
4. Configure the RF front end (`m2sdr_rf --sample-rate 4e6 --bandwidth 4e6
   --rx-freq 1575.42e6 --rx-gain 60 --channel-layout 2t2r`).

## Not yet

- Sub-chip modulations (BOC, CBOC, TMBOC): the replica history holds one bit
  per sample; a modulated replica needs a few bits per entry and a small
  multiply per tap. The tap layout already reaches five taps for them.
- Codes longer than the code memory (GPS L2CL) need a generator instead of a
  table.
- More than one antenna per channel and more than one band.
- 1R1T mode (the tap assumes one 2R2T word per sample instant).
