# Signal configuration: code length, code rate, capabilities

The correlator bank started life as a GPS L1 C/A machine with the constants
inlined — 1023 chips, 1.023 Mchip/s, a code phase reported as a fraction whose
integer chip "must be" 1022. This page is what replaced them: what a channel can
be told at runtime, what is fixed at build time, how the host discovers which is
which, and what the records say about the dump they carry.

It is the gateware half of step 2 of
[GNSSReceiver.jl#130](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/130);
the host-side contract it serves is GNSSReceiver's
[hardware-correlator contract](https://github.com/JuliaGNSS/GNSSReceiver.jl/blob/main/docs/src/hardware_contract.md).

## 1. What is runtime, what is build time

| Property | Where it lives | Changed by |
|---|---|---|
| Primary code (the chips) | channel code RAM | `gnss_chN_code_load`, committed by `restart` |
| Primary code length | `gnss_chN_code_length` | staged, committed by `restart` |
| Chipping rate | `gnss_chN_code_freq` (+ `code_freq_next`) | immediately, or at `apply_at` |
| Carrier frequency / phase | `gnss_chN_carrier_freq` / `carrier_phase` | immediately, or at `apply_at` |
| Code phase | `gnss_chN_code_phase` | loaded by `restart` |
| Tap offsets (VE/E/L/VL) | `gnss_chN_tap_offset_*` | immediately |
| Sub-chip modulation | `gnss_chN_subcarrier_load` + `replica.subchips` | staged, committed by `restart` |
| Reported tap layout (3 or 5) | `gnss_chN_replica.taps` | staged, committed by `restart` |
| **Longest code a channel can hold** | code-RAM depth | **rebuild** (`--max-code-length`) |
| **Widest correlator layout** | 3 or 5 taps | rebuild (`--taps`) |
| **Sub-chip table depth** | `max_subchips` | rebuild (`--max-subchips`) |
| **Antennas, channels, accumulator width** | build parameters | rebuild |

Everything in the first block is per channel, so one bank can track GPS L1 C/A
on one channel and Galileo E1 on the next — on three taps and five taps
respectively, in the same record stream (see
[sub-chip modulation](subchip_modulation.md)). Everything in bold is reported
through `gnss_capabilities`, because a host that assumes it is a host that arms
a channel the gateware cannot serve.

## 2. Code lengths and the code RAM

`--max-code-length` sizes the per-channel code RAM. The lengths in scope for
BPSK primary codes, and what each costs:

| Chips | Signals (GNSSSignals names) | Code bits per channel | 4-channel bank |
|---:|---|---:|---:|
| 330 | short test / staging codes | 990 | 3 960 |
| 1023 | `GPSL1CA` | 3 069 | 12 276 |
| 2046 | `BeiDouB1I` | 6 138 | 24 552 |
| 4092 | `GalileoE1B`, `GalileoE1C`, `GPSL1C_*`, `BeiDouB1C_*` | 12 276 | 49 104 |
| 5115 | `GalileoE6B`, `GalileoE6C` | 15 345 | 61 380 |
| 10230 | `GPSL5I/Q`, `GalileoE5*`, `BeiDouB2a*`, `GPSL2CM` | 30 690 | 122 760 |

"Code bits per channel" is exact for a `--max-subchips 1` build: there are three
copies of the code RAM, so it is `3 × max_code_length` bits. The copies are one
per *address* (`idx − 1`, `idx`, `idx + 1`), which is what lets every tap read a
different chip in the same cycle from a plain one-write/one-async-read RAM —
and, because no tap offset reaches further than a chip, it is three copies
whether the bank has three taps or five. A build with a subcarrier makes each
word two bits wide (the chip plus the TMBOC table-select bit), so the table
above doubles; see [sub-chip modulation](subchip_modulation.md) §6.

**The read is asynchronous, so this is LUTRAM, not block RAM.** Block RAM on
7-series reads synchronously; using it would need a pipeline stage between the
chip-index arithmetic and the tap mux that the channel does not have today. At
10230 chips a 4-channel bank is therefore ≈123 kbit of distributed RAM without a
subcarrier and ≈246 kbit with one — a real but not alarming fraction of an
XC7A200T's SLICEM capacity, and the first thing to revisit if the channel count
grows. A five-tap bank costs neither more nor less: the taps share a
three-address window rather than owning a copy each.

Synthesis and timing numbers are deliberately **not** quoted here: this
repository's CI is board-free and has no Vivado, so any figure would be an
estimate dressed as a measurement. Measured figures for a real build live in
[gateware builds](gateware_builds.md) — including the finding that a
4-channel / 10230-chip / five-tap build **misses timing by 2.875 ns**, on a
critical path that runs straight through the asynchronous LUTRAM read this
section describes. Reproduce them with

```
python build.py --channels 4 --max-code-length 10230 --build
```

and record the utilisation report against the build name, which carries the code
length, the tap count and the sub-chip depth
(`gnss_m2sdr_m2_x1_ch4_ant1_code10230_tap5_sub12`).

A shorter build is not a lesser one: `--max-code-length 1023` is the right
choice for an L1 C/A-only deployment, and the capability CSR then says 1023, so
GNSSReceiver refuses a 4092-chip signal before arming instead of after.

## 3. Code rate: the representable range

The code NCO is a `code_frac_bits` fractional accumulator that crosses **at most
one chip boundary per input sample**. So the representable chip rates at a
sample rate `fs` are

```
fs / 2**code_frac_bits   ...   fs * (2**code_frac_bits - 1) / 2**code_frac_bits
```

i.e. anything strictly between "one chip per 2²⁴ samples" and "one chip per
sample". `gnss_chN_code_freq` is `round(f_chip / fs * 2**code_frac_bits)`.

This is a real constraint, not a formality: GPS L5 and Galileo E5 chip at
10.23 Mchip/s, which needs `fs > 10.23 MHz`. At the L1 C/A bring-up rate of
4.092 MHz the ratio is 2.5 chips/sample and there is no step word for it.

The register is therefore **one bit wider than the fraction**, so an
unrepresentable rate arrives as a value the gateware can see:

  * bit `code_frac_bits` set ⇒ `gnss_chN_code_status.rate_unsupported`, the
    sticky `gnss_rate_error` bit for that channel, and **no records at all** from
    that channel until it is re-armed with a representable rate;
  * the host driver's `GNSSChannel.code_word()` raises `ValueError` on the same
    condition.

Before this, the host masked the word to `code_frac_bits`. 10.23 Mchip/s at
fs = 4.092 MHz became `0x800000` — exactly 0.5 chips/sample — and the channel
armed, correlated a plausible-looking nothing and never locked, with no status
bit anywhere pointing at the cause.

## 4. Arming: why the restart is the commit

A channel is re-assigned by writing a new code into its RAM. That takes
thousands of CSR writes, during which the replica is part one satellite and part
another. So:

1. `code_load.reset_addr` — or any `subcarrier_load` write — opens the window
   and sets `code_status.loading`. **From this point the channel emits no
   records.**
2. The host streams the chips, writes the subcarrier table, and stages
   `code_length`, `code_phase` and `replica`.
3. `restart` — immediate, or scheduled on a sample through `apply_at` — closes
   the window, commits `code_length` and the replica shape, loads the code
   phase, clears the accumulators and the sticky health bits, and lets records
   flow again.

The result is that no record ever describes a half-written code, a code read at
the wrong length, or a replica whose shape changed under the integration, and
the first record after the restart is the new satellite's. `code_length_active` reads back what is actually in force, so the
host can confirm the commit landed without waiting for a dump.

The wrap comparison is `chip_index >= code_length - 1` rather than `==` as a
second line of defence: a chip index left beyond the end by a shortened length
wraps on the next chip instead of running a full lap of the RAM, which would be
a code period of lost lock rather than one chip.

## 5. What a record says about itself

See `gnss_m2sdr/record_format.py` for the full 16-word layout. The fields this
step added, all in words version 1 left reserved:

| Field | Word | Meaning |
|---|---|---|
| `version` | 9 [15:8] | `RECORD_FORMAT_VERSION`, currently 2 |
| `num_taps` | 9 [23:16] | Correlator taps in this record (3, or 5 with words 12–15) |
| `code_phase_chip` | 10 [31:0] | Integer chip index on the last integrated sample |
| `code_length` | 10 [63:32] | Primary-code chips the channel was configured for |
| `code_step` | 11 [31:0] | Code NCO step the integration ran at |

`code_phase_chip` is the point of the exercise. The old host reconstructed it as
`code_length - 1` on the grounds that a dump fires on the wrap — which hard-codes
1022 for GPS L1 C/A, is wrong for every other length, and stops being true at all
once dumps get shorter than a primary period
([GNSSReceiver.jl#133](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/133)).
`code_phase_chips(rec, frac_bits)` returns the complete phase and **raises** on a
version-1 record rather than reporting a confident chip 0.

`num_taps` is there because GNSSReceiver's contract requires it per record: one
stream carries both a 3-tap and a 5-tap layout, and a record whose tap count is
not the tracked correlator's is dropped rather than reshaped. It is now set per
*channel*, and a five-tap record fills the four tail words version 2 reserved —
on the same wire, with no version bump, exactly as this field was allocated for.
See [sub-chip modulation](subchip_modulation.md) §3.

**The magic does not change.** `RECORD_MAGIC` is the framing anchor
(`find_record_offset`): a host that cannot frame the stream cannot read the
version byte that would explain why. Version 2 only fills reserved words, so a
version-1 host keeps parsing correctly and simply does not see the new fields.
Bump the magic only when a field moves or changes size.

## 6. Capability discovery

Three read-only CSRs, all build-time constants:

`gnss_version`

| Field | Bits | Value |
|---|---|---|
| `csr` | [7:0] | `CSR_LAYOUT_VERSION` (3) |
| `record` | [15:8] | `RECORD_FORMAT_VERSION` (2) |

`gnss_capabilities`

| Field | Bits | Meaning |
|---|---|---|
| `n_channels` | [7:0] | Tracking channels in the bank |
| `num_ants_max` | [15:8] | Antenna blocks a dump can carry |
| `num_taps` | [23:16] | Widest correlator layout this build produces |
| `code_frac_bits` | [31:24] | Fixed-point scale of `code_freq` / `code_phase.frac` |
| `carrier_phase_bits` | [39:32] | Fixed-point scale of `carrier_freq` / `carrier_phase` |
| `accum_bits` | [47:40] | Accumulator width (sums saturate here) |
| `max_code_length` | [63:48] | Code-RAM depth = `max_primary_code_length` |

`gnss_signal_caps`

| Field | Bits | Meaning |
|---|---|---|
| `modulations` | [7:0] | bit0 `:LOC`, bit1 `:BOCcos`, bit2 `:CBOC`, bit3 `:TMBOC`, bit4 `:BOCsin` — derived from `max_subchips` |
| `max_secondary_code_length` | [15:8] | 1 = primary code only |
| `reports_code_phase` | [16] | 1 = records carry a complete code phase |
| `tap_layouts` | [20:17] | bit *i* ⇒ 2*i*+3 taps (bit0 = 3, bit1 = 5) |
| `max_subchips` | [28:21] | Sub-chip table depth (1 = no subcarrier) |
| `replica_bits` | [36:29] | Signed width of a subcarrier table entry |

`GNSSBank.capabilities(fs)` reads all three and returns them as a dict, with
`code_frequency_limits` derived from `code_frac_bits` and `fs` and
`max_tap_offset_chips = 1.0` (the E/L taps address chip index ±1). It raises if
the gateware's layout revision is newer than the driver's — an unknown layout
read as if it were this one is an over-declared capability, which GNSSReceiver's
contract calls out as the failure mode that is hardest to attribute.

The modulation bits are derived from `max_subchips`, never from what the field
has a bit for: a `--max-subchips 1` build declares `:LOC` alone, as it always
did. See [sub-chip modulation](subchip_modulation.md) §5 for why `:BOCsin` took
a new bit instead of the one version 2 reserved for `:BOCcos`.

## 7. Audit at higher rates

**Record bandwidth.** One record (128 B) per channel per primary-code period.

| Signal | Period | Records/s/channel | Bytes/s/channel |
|---|---:|---:|---:|
| GPS L1 C/A (1023 @ 1.023 Mcps) | 1 ms | 1000 | 128 kB/s |
| BeiDou B1I (2046 @ 2.046 Mcps) | 1 ms | 1000 | 128 kB/s |
| Galileo E1 (4092 @ 1.023 Mcps) | 4 ms | 250 | 32 kB/s |
| Galileo E6 (5115 @ 5.115 Mcps) | 1 ms | 1000 | 128 kB/s |
| GPS L5 (10230 @ 10.23 Mcps) | 1 ms | 1000 | 128 kB/s |
| 330 chips @ 1.023 Mcps | 0.32 ms | 3100 | 397 kB/s |

Every primary code in scope is a 1 ms or 4 ms period, so moving off GPS L1 C/A
does **not** raise the record rate — a 4-channel bank plus a 1 kHz epoch strobe
is ≈0.6 MB/s against a PCIe Gen2 x1 link. Only a sub-millisecond code (the 330
row, and the short tracking dumps of GNSSReceiver.jl#133) changes the picture,
and even that is under half a megabyte per second per channel. The serializer
needs 16 `sys_clk` cycles per record, so the round-robin is nowhere near
saturated either.

**Accumulator width.** The worst case is
`N × |sample|max × 127 × max|replica|`; with 16-bit samples, a ±1 replica and
`accum_bits = 32` that rails at N ≈ 515 samples (an amplitude-bearing CBOC
replica moves it in by its peak — see
[sub-chip modulation](subchip_modulation.md) §6). That bound is
pathological (full-scale input, perfectly correlated with the replica); a normal
integration is noise-dominated and grows as `√N`, so at fs = 30.72 MHz and a 1 ms
period (N = 30720) a well-set AGC leaves more than two orders of magnitude of
headroom. Raising `fs` or the integration length moves towards the rail linearly
in the pathological case and as `√N` in the normal one, and the existing
behaviour covers both: the accumulators **saturate rather than wrap**, the dump
that clamped is marked, and `gnss_saturation` is the sticky per-channel bit. A
wrapped accumulator would reach the host as a plausible correlator value; a
clamped one is recognisable. Nothing here needed changing for longer codes, but
`gnss_saturation` is the bit to watch when the sample rate goes up.

**Overflow reporting** is unchanged and rate-independent: the recorder's holding
register frees on the cycle a record retires, so back-to-back dumps at any of the
rates above fit. `gnss_overflow` / `gnss_droppedN` stay sticky until cleared.

**Epoch strobe.** `gnss_epoch_period` counts *input samples*, in 32 bits — up to
70 s even at fs = 61.44 MHz — and the strobe is a recorder slot like a channel,
so it inherits the same overflow accounting. It is unaffected by code length by
construction: that is exactly why it exists (a host whose channels are all silent
still needs an epoch clock).

## 8. Not covered here

Secondary-code wipe-off (GNSSReceiver.jl#132), primary codes longer than the code
RAM and dumps shorter than a primary period (GNSSReceiver.jl#133), and multi-band
routing (GNSSReceiver.jl#134). BOC/TMBOC/CBOC replicas and five-tap correlation
have their own page: [sub-chip modulation](subchip_modulation.md). Until the rest
land, the capability CSRs declare conservatively — an under-declared capability is
refused, an over-declared one is a channel that arms and never locks.
