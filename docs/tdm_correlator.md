# The TDM correlator

`gnss_m2sdr/tdm/` is the gateware for the LiteX-M2SDR: one pipelined
correlator datapath, time-multiplexed over every channel. It replaced an
earlier design with one correlator instance per channel ([the per-channel
bank](#the-per-channel-bank-retired)). It is what the hardware-correlator
receiver (GNSSReceiver.jl `receive(::HardwareLoop, …)`, GNSSM2SDR.jl's
`M2SDRLoop`) drives.

## Why time-multiplexed

At 4 MS/s and a 125 MHz system clock there are 31 clock cycles per input
sample. The per-channel bank used one of them per channel and spent the rest
idle, with a complete datapath — carrier NCO and table, code RAMs, five
correlators, DSP cascades — per channel. Six channels closed timing only at
`--timing-effort max` with directive tuning, 56 DSP48E1 for four channels.
(The TDM engine's DSPs are the wipe-off at the replica's two amplitudes and
the carrier table's scaling by them; without sub-chip replicas it needed 4.)

The TDM engine runs the datapath once per channel per sample: a round of
`max(n_channels + 1, 7)` slots per sample, one per channel plus one for the
epoch strobe, and one cycle to load the next sample. Each slot reads the
channel's state from a RAM, advances it through an eight-stage pipeline and
writes it back (in S7, seven cycles after the read) before the channel's next
slot; seven slots are the fewest that guarantee that (`params.num_slots`). Every register of the datapath is a pipeline stage, so the design is
easy on timing, and its size barely depends on the channel count:

| build | channels | LUTs (whole SoC) | DSP48E1 | BRAM tiles | WNS (sys_clk) |
|---|---:|---:|---:|---:|---:|
| `gnss_tdm_m2_x1_ch24_code4096` | 24 | 15 628 (11.7 %) | **10** | 75 | +0.118 ns |

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

1. **S0–S1** read the channel's NCO and history state (block RAM), its NCO
   words, its static configuration and its pending arm (distributed RAM),
   select the antenna. The accumulators live in a second state RAM that is
   read in S4, so they join the pipeline only where S6 needs them.
2. **S2** decides a pending arm: start at the staged sample, or mark it missed
   if that sample has passed. Adds the carrier and code NCO increments. A
   starting channel's accumulators are cleared on their way into S6.
3. **S3–S4** the shared carrier table (64 entries, ±127), scaled by the
   channel's two replica amplitudes ([the replica](#the-replica)), and the code
   memory (32-chip words, block RAM, one memory for all channels); the
   sub-chip index `floor(frac · P)`; the code-chip carry and the wrap at the
   code length.
4. **S4–S5** the chip's sign times the sub-chip's sign, and the sub-chip's
   level; the carrier wipe-off: eight 12 × 16-bit products, at each amplitude
   — the only DSPs of the engine.
5. **S5** the replica history: the code NCO runs on the *earliest* tap, and
   each sample pushes that tap's replica sign and level, and whether its code
   period ended, into a 32-entry history. Tap `k` with sample shift `s` reads
   entry `lead - s`; the prompt's period ends where entry `lead` carries the
   wrap flag. No tap needs its own code read or multiplier.
6. **S6** five tap pairs accumulated (33 bits), each with the wiped-off sample
   at the amplitude its history entry names, **S7** saturated to 32 bits,
   written back; at the prompt's period end the record is emitted and the
   accumulators restart. A record's flags say whether an accumulator
   saturated and whether samples were lost while it integrated.

Number formats are in `params.py`: 32-bit carrier phase and signed frequency
word, a 20-bit chip index with a 32-bit fraction and an unsigned code word
(chips per sample × 2³²), 12-bit samples, 32-bit accumulators.

## The replica

A channel's replica is the code chip times a sub-chip value: the chip is cut
into `P ≤ 12` equal sub-chips, and sub-chip `k = floor(frac · P)` — the
full-precision product of the 32-bit fraction, not its top bits, since a
sub-chip edge at `k/12` chip is not dyadic — has a sign and one of two
amplitudes. So the replica is `chip · sign[k] · (level[k] ? amp_hi : amp_lo)`,
programmed per arm through `s_subcarrier` (signs, levels, `P`) and
`s_amplitude` (7-bit `amp_hi`, `amp_lo`):

| signal | `P` | sub-chips | amplitudes |
|---|---:|---|---|
| GPS L1 C/A (BPSK, the reset values) | 1 | + | 1 |
| BOC(1,1) | 2 | + − | 1 |
| Galileo E1B, CBOC(+) | 12 | `25 13 25 13 25 13 −13 −25 −13 −25 −13 −25` | 25, 13 |
| Galileo E1C, CBOC(−) | 12 | `13 25 13 25 13 25 −25 −13 −25 −13 −25 −13` | 25, 13 |

The CBOC levels are GNSSSignals' Int8 code table, `a1 · BOC(1,1) + a2 ·
BOC(6,1)` with `(a1, a2) = (19, 6)` — what Tracking.jl's integer correlator
correlates with — whose RMS, `√397 = 19.92`, is `get_code_amplitude`; the
receiver declares it as the code amplitude, and the loop core divides it out.
The host derives both words from that table (GNSSM2SDR.jl's
`subchip_replica`, `params.cboc_table` here), so the replica cannot drift from
the software receiver's. A replica correlated with the wrong shape is no small
loss: a BPSK replica of E1B's code correlates to zero at the prompt
(GNSSM2SDR.jl#16).

At 4 MS/s the BOC(6,1) component is aliased, as it is in the software
receiver, which samples the same table at the same phases.

The amplitude scales the accumulators by up to 25: a full-scale input that
correlates perfectly reaches the 32-bit rail after about 200 samples, but a
real integration is noise: at σ = 300 LSB a 4 ms E1B record
(16 000 samples) sums to about `√16000 · 300 · 127 · 19.9 ≈ 10⁸`, 21 times
below the rail.

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
- **Lost samples.** Each sample carries its device index into the engine.
  When the index jumps — the sample FIFO overflowed (`gnss_sample_overflows`)
  or the engine was disabled — the channels' NCOs have fallen behind the
  index, and every record integrating across the jump carries
  `FLAG_SAMPLE_OVERFLOW`.
- **Records** on DMA1: 64-byte blocks — magic, kind (dump / strobe / pad),
  channel, tap count, assignment id, the sample index one past the last sample
  integrated, the integrated count, the earliest tap's code phase, and five tap
  pairs. The serialiser pads a part-filled DMA buffer once its first record is
  `gnss_flush_cycles` old, so with the kernel module's per-channel geometry
  (`dma_buffer_size=8192,512 dma_buffer_per_irq=8,1`, upstream #187) a record
  reaches the host within ~0.1 ms of its end. Clearing
  `gnss_control.record_enable` finishes the record in flight and pads its
  buffer out before the stream stops, so the host's buffers always hold whole
  records.
- **Epoch strobes** (`gnss_strobe_period`) keep the loop's epoch clock moving.
  They sit on the multiples of the period in the sample index, not on a count
  of samples, so lost samples and enable toggles do not move them: a strobe
  follows the sample whose index + 1 reaches the next multiple (a gap over one
  or more multiples gives a single strobe, at the first sample after it). After a reset or a write of the period the engine finds the
  period's phase in the first `STROBE_SYNC_SAMPLES` (8) samples it processes,
  without a strobe (`params.py`, "Epoch strobes").
- **Reset** (`gnss_control.reset`) clears the channels, the FIFOs, the
  serialiser, the strobe phase and the counters (`records`, `record_drops`,
  `sample_overflows`). `gnss_sample_count` is the sample index one past the
  last processed sample, not a count, and the RX tap's `host_drops` is the tap's
  own; neither is cleared.

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
  releases and a stalling DMA; rounds back to back at the fewest slots; epoch
  strobes over lost samples and period changes; a real sample-FIFO overflow
  (the model replayed on the samples the engine took); `record_enable`
  toggled mid-record; the reset's counters.
- `test/test_tdm_xsim.py`: the main, back-to-back and strobe scenarios on the
  Verilog **LiteX's converter writes** (the artefact the bitstream is built
  from), in Vivado's xsim, plus checks that no negative literal and no signed `.init` entry is in
  it — the [two lowering faults](#two-lowering-faults) that reached silicon
  with the per-channel bank.
- `test/test_tdm_rxtap.py`: the tap under a stalling host — the correlator
  sees every sample on consecutive indices, the host's tags name its own
  samples, the drop count is exact.
- On the board (orin1, 2026-10-16): with a channel armed inside a capture,
  every one of 3 890 hardware records matched the emulator (`M2SDRLoop`'s
  `TDMEmulator`, a port of the model) fed the same captured samples, bit for
  bit; the trimmed `gnss_loop` closed the loops on live satellites at
  41–45 dB-Hz with bit sync, 0 bytes allocated and records at most 0.6 ms old
  at the fold.
- On the board (orin1, 2026-10-19, the build in the table): 4 313 of 4 313
  records bit-exact against the emulator again; strobes on the multiples of
  the period, also across a 5 ms disable of the engine (one strobe at the gap,
  one `FLAG_SAMPLE_OVERFLOW` dump per channel); no malformed DMA block through
  670 000 back-to-back `record_enable` toggles (the previous build garbled
  the stream within them); a position fix with `receiver.jl` on RX2.
- On the board (orin1, 2026-10-19, the sub-chip replica build): with Galileo
  E1B channels armed as CBOC (±25/±13, 12 sub-chips) and as BOC(1,1) next to
  GPS L1 C/A ones on the antenna's samples, all 4 198 dump records received
  were bit-exact against the emulator fed the same capture; GPS L1 C/A
  tracking and position fixes with `receiver.jl` were unchanged.

### Two lowering faults

The Migen simulator runs the design, the bitstream is built from the Verilog
LiteX writes, and twice the per-channel bank's hardware differed from its
simulation because of that Verilog:

- **A negative constant written unsigned.** LiteX before `37b75bd4`
  (2026-07-24) rendered a negative constant as `-32'h80000000`, an unsigned
  literal. Verilog compares unsigned when either operand is, so the saturating
  accumulator's lower-bound test was true for every positive sum and clamped it
  to −2³¹. `requirements-test.txt` pins LiteX at that commit, and the TDM
  engine saturates on the sum's top bits instead of comparing against a
  constant at all.
- **A signed memory initialiser.** LiteX writes a `Memory`'s contents to a
  `$readmemh` file with `"{:02x}".format(d)`, which for a negative `d` is
  `-3`, not hex. The carrier sine table came out unreadable and the channel
  never correlated. The TDM carrier table is a case statement on unsigned
  8-bit patterns, and none of its memories has a signed initialiser.

`test/test_tdm_xsim.py` checks both on the lowered Verilog, and runs the
engine scenario on it in xsim against the model.

## Bring-up on a board

1. `git submodule update --init`, then `python build_tdm.py --build`
   (litex_m2sdr is pinned at `third_party/litex_m2sdr`; see the README).
2. Rebuild that checkout's kernel module and user tools with the build's
   `software/kernel/{csr,soc,mem}.h`; load the module with
   `dma_buffer_size=8192,512 dma_buffer_per_irq=8,1`.
3. Write `*_operational.bin` to the operational slot (0x800000), ICAP-reload,
   reboot the host.
4. Configure the RF front end (`m2sdr_rf --sample-rate 4e6 --bandwidth 4e6
   --rx-freq 1575.42e6 --rx-gain 60 --channel-layout 2t2r`).

## Not yet

- TMBOC (GPS L1C pilot): its sub-chips change from chip to chip, which the
  per-channel sub-chip table cannot follow; it needs a pattern bit beside each
  chip in the code memory.
- Codes longer than the code memory (GPS L2CL) need a generator instead of a
  table.
- More than one antenna per channel and more than one band.
- 1R1T mode (the tap assumes one 2R2T word per sample instant).

## The per-channel bank (retired)

The first gateware (`gnss_m2sdr/gateware/`, `build.py`) instantiated a
complete tracking channel per satellite and supported features the TDM engine
does not have yet: sub-chip modulations (BOC(1,1), Galileo E1 CBOC, GPS L1C
TMBOC, checked against GNSSSignals.jl) and two antennas per channel. It was
built on the `add_rx_datapath_processing()` hook of litex_m2sdr#152, which
upstream did not merge (it took the RFIC datapath hook, #185, instead), so it
no longer builds against upstream litex_m2sdr. It was removed together with its
host software, scripts and design notes; its last version, including the
build log and bring-up notes, is at [`c5c6085`](https://github.com/JuliaGNSS/gnss-m2sdr/tree/c5c60855666580e0dd8d934fa3a46970aa88c62e).
