# Sub-chip modulation and five-tap correlation

The correlator bank replicated one binary value per chip and correlated it
against three taps. Neither is enough for the BOC-family signals: their replica
changes *inside* a chip, and `Tracking` uses five taps to track them because a
three-tap DLL locks onto a BOC side peak as happily as onto the main one.

This page is the gateware half of step 5 of
[GNSSReceiver.jl#130](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/130)
(gnss-m2sdr#30). Step 2's [signal configuration](signal_configuration.md) page
still describes code length, code rate, arming and record framing; only what
changed is repeated here.

## 1. What is claimed, and what is not

The replica is **exactly the one GNSSSignals models**, for these types:

| GNSSSignals type | `get_modulation` | Sub-chips `P` | Replica amplitude |
|---|---|---:|---:|
| `GPSL1CA` | `LOC()` | 1 | 1 |
| `GalileoE1B` | `CBOC(BOCsin(1,1), BOCsin(6,1), 10/11, +1)` | 12 | 19.9249 |
| `GalileoE1C` | `CBOC(BOCsin(1,1), BOCsin(6,1), 10/11, −1)` | 12 | 19.9249 |
| `GalileoE1B_BOC11`, `GalileoE1C_BOC11` | `BOCsin(1,1)` | 2 | 1 |
| `GPSL1C_D` | `BOCsin(1,1)` | 2 | 1 |
| `GPSL1C_P` | `TMBOC(BOCsin(1,1), BOCsin(6,1), 4/33)` | 12 | 1 |
| `BeiDouB1C_D`, `BeiDouB1C_P` | `BOCsin(1,1)` | 2 | 1 |

Generically the gateware synthesises `LOC`, `BOCsin(m,1)`, `BOCcos(m,1)`,
`CBOC(m1,m2,·)` and `TMBOC(m1,m2,pattern)` for any order whose sub-chip factor
fits the build's table (`gnss_signal_caps.max_subchips`).

**Not claimed.** BeiDou B1C's pilot is QMBOC(6,1,4/33) in the ICD; GNSSSignals
models it as a pure `BOCsin(1,1)` because the minor BOC(6,1) arm is in phase
quadrature and no single real replica captures it. This gateware replicates what
GNSSSignals models — BOC(1,1) — and the capability CSR says `:BOCsin`, not
"QMBOC". Likewise nothing here generates AltBOC, and nothing here generates the
secondary/overlay code: the host removes the overlay
([GNSSReceiver.jl#132](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/132)),
so the gateware replicates the primary code only.

**`GalileoE1B_BOC11` is a different signal, not a cheaper `GalileoE1B`.** It is
a separate row above, a separate entry in `gnss_m2sdr/subcarrier.py`'s
`SIGNAL_MODULATIONS`, and a separate `HardwareChannelConfig.signal` on the host.
The gateware never substitutes it: ask for it by name and the receiver is told
it got it (0.45 dB of correlation loss, amplitude 1), or ask for `GalileoE1B`
and get the amplitude-bearing CBOC table. Nothing in between.

## 2. How a sub-chip replica is evaluated

GNSSSignals' `get_subcarrier_code(modulation, phase)` is a function of the code
phase alone, and it is constant across each of `P` equal slices of a chip. So
the replica factorises exactly:

```
replica(phase) = primary_chip(floor(phase)) · lut[floor(frac(phase) · P)]
```

The gateware does exactly that: the code RAM gives the chip, a per-channel table
of `P` signed entries gives the sub-chip amplitude, and the index is a full
`code_frac_bits × P` multiply — **not** the top bits of the fraction. That
matters: a `P = 12` subcarrier changes at multiples of 1/12 chip, which is not a
dyadic rational, so indexing a power-of-two table would put the replica one
sub-chip out at some phases. That is a plausible wrong answer, not an error.

Why one 12-entry table covers the whole L1 family:

  * `BOCsin(m,1)` at `P = 2m`: entry `k` is `+1` for even `k`.
  * `BOCcos(m,1)` at `P = 4m`: entry `k` is `+1` when `(k+m)÷2` is even. The
    quarter-cycle shift straddles the sine grid, which is why cosine BOC needs
    quarter sub-chips; on that grid it is exact, not rounded.
  * `CBOC(m1,m2,p,s)` at `P = 2·lcm(m1,m2)`: `a1·BOC(m1,1) + s·a2·BOC(m2,1)`.
  * `TMBOC(m1,m2,pattern)` at `P = 2·m2`: `BOC(m1,1)` or `BOC(m2,1)` depending
    on the chip's position in the pattern.

`test/test_subchip_replica.py` checks all of this against
`test/data/l1_subchip_golden.json`, produced by GNSSSignals.jl v4.1.0 (the
generator is committed alongside it). The golden replica is sampled on *this
gateware's own* NCO phase grid, so a simulation sample and a golden entry are
the same phase with no interpolation, and a tap `s` samples early is compared
against golden entry `k + s` — the tap placement is checked against GNSSSignals
too, not against the gateware's own prompt.

### TMBOC without a counter

TMBOC alternates per chip position. Instead of a modulo-33 counter that would
have to stay in step with the code wrap *and* with every acquisition handover,
the code RAM is **two bits wide**: the chip, and a "use the other table" bit
written beside it (`gnss_chN_code_load.sub`). Any pattern, any code length and
any start chip then work by construction.

### Amplitude, and the one way to get C/N₀ wrong

`LOC` and the BOC/TMBOC subcarriers are ±1. **CBOC is not**, and the amplitude
is the signal: GNSSReceiver divides the device's code amplitude out of every
accumulator before the C/N₀ estimator sees it
([hardware contract §5](https://github.com/JuliaGNSS/GNSSReceiver.jl/blob/main/docs/src/hardware_contract.md)),
so a sign-only stand-in reads about 26 dB away from the same satellite tracked
in software — and tracks perfectly well while doing it.

GNSSSignals' own resampled table carries CBOC's irrational `√(10/11)` and
`√(1/11)` amplitudes as the integer pair `(a1, a2) = (19, 6)`, i.e. levels ±25
and ±13, whose RMS is `√((25² + 13²)/2) = √397 = 19.9249` — exactly what
`get_code_amplitude(GalileoE1B)` reports. Program that pair and the gateware's
replica *is* the modelled code, so the host's default `replica_code_amplitude`
is already right and needs no override. `gnss_m2sdr.subcarrier` does so by
default.

The table is the host's to choose; `subcarrier.lut_rms()` is the number to
report if you choose differently. A coarser pair — `(3, 1)`, say — is not a
cheaper CBOC but a different code: its level ratio is 2.00 where the spec's is
1.925, so the correlation function is not the modelled one whatever amplitude is
declared for it.

## 3. Five taps

`gnss_chN_tap_offset_{ve,e,l,vl}` place each tap independently, as a signed
`code_frac_bits + 1` two's-complement value in chips. The prompt has no register:
the contract fixes it at zero, and a register would only be a way to get it
wrong.

Program `sample_shift · code_freq` for each — whole input samples times the code
step, the grid `Tracking` quantises its preferred shifts onto
(`calc_preferred_code_shift_to_sample_shift`). For three taps getting that wrong
is a DLL loop-gain error; for five it is worse, because the VE/VL distance
enters the discriminator separately and there is no single number to re-derive
the array from. That is why GNSSReceiver hands over the whole
`tap_sample_shifts` array and says "program exactly these", and why the single
symmetric `spacing` register is gone. `GNSSChannel.set_tap_offsets()` takes the
contract's array directly (latest first, prompt at zero) and
`set_spacing_chips()` remains as a host-side convenience for the symmetric
three-tap case.

Every offset is below one chip (`max_tap_offset_chips = 1.0`), which is also
what `Tracking`'s defaults need — `VeryEarlyPromptLateCorrelator` prefers 0.15
chips for E/L and 0.6 for VE/VL. An offset of exactly −1.0 chip, the one
out-of-range value the register can hold, raises
`code_status.replica_unsupported` and suppresses that channel's dumps rather
than landing on the wrong chip.

**Taps are cheap.** Because no offset reaches further than a chip, however many
taps there are they only read chip index `idx − 1`, `idx` or `idx + 1`. The code
RAM is replicated three times — one copy per *address*, not one per tap — and
each tap muxes between the three. Five taps cost the same code memory as three.
(step 2's page predicted 5/3 of the memory "unless the taps move to a shared
sliding window"; they did.)

### Which taps a record carries

`num_taps` in the record is **per channel, not per build**
(`gnss_chN_replica.taps`, staged and committed by the same restart as the code
and the replica shape). A five-tap build runs GPS L1 C/A on three taps next to
Galileo E1 on five in the same bank and the same record stream:

  * `num_taps == 3` — words 2..4 and 6..8 carry prompt/early/late as always,
    words 12..15 read zero;
  * `num_taps == 5` — words 12..15 add very-early and very-late for antenna 0
    and antenna 1.

Two antennas × two extra taps is exactly the four words record format v2 left
reserved, so the 128-byte stride the DMA framing depends on is unchanged and the
magic does not move. The record format version stays **2**: word 9's `num_taps`
was allocated in v2 for precisely this, and a v2 host already has to check it —
the contract drops a record whose tap count is not the tracked correlator's
rather than reshaping it, so a three-tap host never reads the tail words.

`record_format.tap_accumulators(rec)` returns the accumulators latest first —
`[very late, late, prompt, early, very early]` — which is the order Tracking's
correlators want. Building it early-first inverts the DLL discriminator.

## 4. Arming a replica

The replica shape joins the code under the same rule: written directly with
dumps suppressed, committed by the restart.

1. `code_load.reset_addr`, or any `subcarrier_load` write, sets
   `code_status.loading`. **From this point the channel emits no records.**
2. The host streams the chips (with `code_load.sub` set on the TMBOC pattern
   positions), writes the subcarrier table, and stages `code_length`,
   `code_phase` and `replica` (`subchips`, `taps`).
3. `restart` — immediate or scheduled on a sample through `apply_at` — commits
   all of it and lets records flow again.

So no record can describe a half-written code, a code read at the wrong length,
a replica whose amplitude changed under the integration, or a tap layout that
does not match the accumulators it carries.

## 5. Capability discovery

`gnss_capabilities.num_taps` is now the **widest** layout the build produces.
`gnss_signal_caps` gained three fields:

| Field | Bits | Meaning |
|---|---|---|
| `modulations` | [7:0] | bit0 `:LOC`, bit1 `:BOCcos`, bit2 `:CBOC`, bit3 `:TMBOC`, **bit4 `:BOCsin`** |
| `max_secondary_code_length` | [15:8] | 1 = primary code only |
| `reports_code_phase` | [16] | 1 = records carry a complete code phase |
| `tap_layouts` | [20:17] | bit *i* ⇒ 2*i*+3 taps: bit0 = 3, bit1 = 5 |
| `max_subchips` | [28:21] | sub-chip table depth |
| `replica_bits` | [36:29] | signed width of a table entry |

`modulations` is derived from `max_subchips`, so the build cannot over-declare:
a `--max-subchips 1` build reads `:LOC` alone, exactly as record format v2 did.
One bit cannot distinguish `BOCsin(1,1)` (needs 2 sub-chips) from `BOCsin(6,1)`
(needs 12), so a host must check `max_subchips` against the specific order it
wants as well as the family bit.

**Bit 4, not bit 1, is `:BOCsin`.** Record format v2 reserved bit 1 under the
name `:BOCcos`. Every L1 BOC signal GNSSSignals exposes is *sine*-phased, so
redefining bit 1 would have made a host that knows the v2 mapping declare
`:BOCcos` for a build that synthesises `:BOCsin` — an over-declared capability,
which is a channel that arms and never locks. A host that does not know bit 4
simply refuses the signal instead, which is the safe direction.

`CSR_LAYOUT_VERSION` is **3**: `spacing` is gone, and `tap_offset_*`, `replica`,
`subcarrier_load`, `dump_num_taps` and the `ive/qve/ivl/qvl` readbacks are new.
A driver written for v2 must refuse v3 rather than address the old names.

### What GNSSM2SDR.jl needs (not changed here)

The adapter is a separate repository, so this is the interface, written down:

  * accept `csr_version == 3`, and program `tap_offset_{ve,e,l,vl}` from
    `HardwareChannelConfig.tap_sample_shifts` instead of computing one
    `spacing`; the five-tap layout it currently refuses by name becomes the
    supported path;
  * add `(1 << 4) => :BOCsin` to `MODULATION_BITS`. Leaving it out is safe (the
    signal is refused) but no Galileo/BeiDou/L1C channel will arm;
  * report `tap_layouts` from `gnss_signal_caps` rather than
    `[caps.num_taps]`, so a five-tap build is declared as `[3, 5]`;
  * check `max_subchips` against `subchip_factor(get_modulation(signal))`
    alongside the modulation bit;
  * write the subcarrier table and `replica` (`subchips`, `taps`) during the
    arming window, and set `code_load.sub` on the TMBOC pattern positions;
  * read `num_taps` off each record (it already does) and fill five accumulator
    slots from words 12..15 when it says 5;
  * leave `replica_code_amplitude` at its default for a channel programmed with
    the `(19, 6)` CBOC table — that default is now correct rather than a
    placeholder. Override it only if the table was programmed differently.

`gnss_m2sdr/subcarrier.py` is importable from the host and produces every table,
select-bit array and amplitude above, so the adapter does not have to restate
them.

## 6. Cost, throughput and numerical range

This repository's CI is board-free and has no Vivado, so there are **no measured
utilisation or timing figures here**. What follows is the analytic cost and the
command that produces a real report:

```
python build.py --channels 4 --num-ants 2 --max-code-length 10230 \
                --taps 5 --max-subchips 12 --build
```

and record the utilisation report against the build name, which now carries the
tap count and sub-chip depth
(`gnss_m2sdr_m2_x1_ch4_ant2_code10230_tap5_sub12`). No such build has been run,
so nothing about place-and-route, timing closure or real device occupancy is
claimed.

**Code memory**, per channel: `3 × word_bits × max_code_length` bits, where
`word_bits` is 1 for a `--max-subchips 1` build and 2 once the subcarrier-select
bit exists. **Independent of the tap count.**

| Chips | 3 taps, no subcarrier (step 2) | any tap count, with subcarrier |
|---:|---:|---:|
| 1023 | 3 069 | 6 138 |
| 4092 | 12 276 | 24 552 |
| 10230 | 30 690 | 61 380 |

A 4-channel bank at 10230 chips is therefore ≈246 kbit of distributed RAM (the
read is asynchronous, so this is LUTRAM, not block RAM — see step 2's page).
That is double step 2's number and the same as step 2's number would have been
at five taps without the shared window.

**Subcarrier tables**: `2 × max_subchips × replica_bits` bits per channel = 192
bits at the default 12 × 8. Registers, not RAM: every tap reads a different
index in the same cycle.

**Multipliers**, per channel: `2 × num_taps × num_ants` replica multiplies
(`replica_bits × (sample_bits + carrier_amp_bits + 1)` = 8 × 25 at the
defaults), plus `4 × num_ants` carrier wipe-off multiplies, plus `num_taps`
sub-chip index multiplies (`code_frac_bits × ceil(log2(max_subchips+1))` = 24 ×
4). At five taps and two antennas that is 20 + 8 + 5 = 33 per channel, 132 for a
4-channel bank. The step-2 build's replica multiply was 2 × 25 (a ±1 sign, i.e.
an add/subtract after synthesis); an amplitude-bearing replica makes it a real
8 × 25 multiply, which is the one place this change costs DSP-shaped logic.

**Throughput is unchanged.** The channel is still one sample per strobe through
a two-stage pipeline with no back-pressure; extra taps widen it, they do not
lengthen it. The record is still 128 bytes and still one per primary-code
period, so the DMA1 bandwidth table in step 2's page is unchanged — a five-tap
two-antenna record carries the same 128 bytes a three-tap one does. The
serializer still needs 16 `sys_clk` cycles per record.

**Numerical range.** The accumulators still saturate rather than wrap, and
`gnss_saturation` is still the sticky bit, but an amplitude-bearing replica
moves the rail in by its peak value. The pathological bound (full-scale input,
perfectly correlated) is

```
N_max = 2**(accum_bits-1) / (|sample|max · carrier_amp · max|lut|)
```

With 16-bit samples, `carrier_amp = 127` and `accum_bits = 32` that is ≈515
samples for a ±1 replica and ≈20 samples for the ±25 CBOC table. Both are
pathological; a real integration is noise-dominated and grows as `√N`:

```
|acc| ≈ √N · σ_sample · carrier_amp · lut_rms
```

For Galileo E1 at fs = 4.092 MHz, a 4 ms primary period is N = 16 368, so
`√N = 128` and `lut_rms = 19.92`: an AGC holding σ = 300 LSB leaves ≈22× of
headroom to the rail, and σ = 1000 LSB leaves ≈6.6×. That is a real reduction —
a ±1 replica at the same settings has 20× more — and it is the number to watch
when raising `fs` or the integration length. Two antennas do not change it: the
accumulators are per antenna and never summed in gateware.

If a deployment cannot afford the headroom, the answer is a smaller *carrier*
amplitude or a lower AGC point, not a smaller CBOC table: a table with the wrong
level ratio is a different correlation function (§2).

## 7. Not covered here

Secondary-code wipe-off in gateware
([GNSSReceiver.jl#132](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/132)),
primary codes longer than the code RAM and dumps shorter than a primary period
([#133](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/133)), multi-band
routing ([#134](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/134)), and
AltBOC / QMBOC replicas. The capability CSRs declare conservatively for all of
them.
