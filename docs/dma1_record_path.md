# DMA1 record path: framing and latency

The correlator-dump records (`gnss_m2sdr/record_format.py`) are streamed to the
host over a second litepcie DMA channel (`pcie_dma1`). The kernel driver writes
the ring in fixed-size buffers and only completes a buffer when it is *full* —
there is no partial flush and no timeout. That granularity decides both how the
stream must be framed and how fast the host can see a dump.

Driver constants (`litex_m2sdr/software/kernel/config.h`; stock litepcie uses
the same buffer size but `DMA_BUFFER_PER_IRQ = 32`):

```c
#define DMA_BUFFER_PER_IRQ     8
#define DMA_BUFFER_COUNT       256
#define DMA_BUFFER_SIZE        8192
```

## Framing (fixed)

The record is **128 bytes** (`RECORD_WORDS = 16`), not the 80 bytes the
two-antenna payload needs:

- `8192 % 48 = 32`, so the original 48-byte records straddled buffer boundaries.
  The driver drops *whole* buffers on ring overrun, so one drop shifted every
  subsequent record by 32 bytes — permanently, with nothing to resynchronise on.
  80 and 96 bytes leave the same remainder, so neither helps.
- `8192 / 128 = 64` exactly: every DMA buffer starts on a record boundary, a
  dropped buffer costs exactly 64 records, and the property survives any
  power-of-two buffer size a future per-DMA length would pick.

The upper half of word 5 carries `RECORD_MAGIC = 0x53534E47` ("GNSS" in wire
order, at byte offset 44 of every record). The stream endpoint's `first`/`last`
are set by the recorder but litepcie's DMA writer ignores them, so the host gets
no framing help from the transport; the magic is the only in-band anchor. Host
side, `find_record_offset()` locks onto the stream (mid-record attach, torn
buffer) and `parse_records()` unpacks a raw byte stream, resynchronising instead
of misparsing.

The padding costs 128 kB/s per channel at 1 kHz dumps — free at these rates. It
buys a record whose size does not depend on gateware build options: words 2–4
and 6–8 are one E/P/L block per antenna (`num_ants` in word 9 says how many are
valid, the rest read zero), so a single-antenna build and a two-antenna one are
parsed identically. Words 10–15 are reserved (zero) and are where a further
extension can go without moving any existing field; bump `RECORD_MAGIC` if a
layout ever changes incompatibly. The epoch strobe (#7) needed no extra words:
it is a normal record with `channel = 0xFF`, `flags` bit 1 set and a zero
payload — including zero antenna blocks and a zero `num_ants`.

## Latency (not fixed here — a driver/transport limit)

A buffer completes only when 64 records have been written, and `hw_count` (the
only thing `read()`/`poll()`/`LITEPCIE_IOCTL_DMA_WRITER` expose) is advanced
*only in the MSI handler* — so polling cannot beat the IRQ cadence either.

| channels | record rate | buffer completes every | MSI (every 8 buffers) |
|---|---|---|---|
| 4 | 4000 rec/s (512 kB/s) | 16 ms | 128 ms |
| 2 | 2000 rec/s (256 kB/s) | 32 ms | 256 ms |
| 1 | 1000 rec/s (128 kB/s) | 64 ms | 512 ms |

(128-byte records make this *better* than the 48-byte figures in issue #4 —
42.7 ms/4 ch — because a buffer holds fewer, larger records. Padding for
framing trades bandwidth, which is free here, for latency, which is not.)

A `doppler_update_interval` of ~1 ms cannot be closed through this: tens to
hundreds of epochs of transport delay will not stabilise at a normal PLL/DLL
bandwidth. Nothing in this repo's gateware can shorten it, because the buffer
length lives in the driver's descriptor writes
(`litepcie_dma_writer_start()` programs `DMA_BUFFER_SIZE` into every descriptor
of both channels, and sets `DMA_IRQ_DISABLE` on all but every
`DMA_BUFFER_PER_IRQ`-th).

Consequences for the design, in preference order:

1. **Per-DMA buffer sizing in the driver.** The descriptor length field is
   per-descriptor and host-programmable; the driver just uses one compile-time
   constant for both channels. A 512-byte DMA1 buffer with
   `DMA_BUFFER_PER_IRQ = 1` gives 4 records → 1 ms at 4 channels. This is the
   real fix and it belongs in the (already required) "expose the 2nd DMA
   channel" driver work, not in gateware.

   enjoy-digital/litex_m2sdr#151 ("liblitepcie: read DMA ring geometry from the
   kernel at runtime", open) is an enabler for this, not a solution to it. It
   stops liblitepcie baking in the compile-time `DMA_BUFFER_SIZE`/`_COUNT` and
   reads them from `LITEPCIE_IOCTL_MMAP_DMA_INFO` instead (the macros stay only
   as a fallback when the kernel reports 0), so a buffer-size change becomes a
   kernel-module rebuild rather than a matched userspace/SoapySDR rebuild — and
   it independently confirms that the descriptor length is programmed at
   runtime. What it does *not* give us is per-channel geometry: the ioctl
   carries one `dma_rx_buf_size`/`dma_rx_buf_count` pair per *direction*
   (`kernel/litepcie.h:55-57`), and DMA0 (I/Q) and DMA1 (records) are both
   writer/RX channels, so shrinking to 512 B for record latency would shrink
   the I/Q buffers too — the opposite of #151's own motivation, which is
   *larger* buffers to absorb GC pauses on zero-copy RX. `DMA_BUFFER_PER_IRQ`
   and the MSI-only `writer_hw_count` update are untouched by it as well, so
   even with small buffers the host still waits `DMA_BUFFER_PER_IRQ` of them per
   interrupt. Per-DMA-channel sizing remains the missing piece.
2. **Bounded-schedule filler records** — *implemented* as the epoch strobe
   (`gnss_epoch_period`, issue #7): one marker record every Δ input samples
   regardless of what the channels are doing, so buffer completion depends on
   wall-clock rather than on how many channels are locked. At Δ = 1 ms it adds
   1000 rec/s, which bounds the *worst* case (1 locked channel: 64 ms →
   32 ms; nothing locked: never → 64 ms) but cannot get below the 16 ms floor
   set by 64 records/buffer, so it is a complement to (1), not a substitute.
   Its primary job is the host's epoch clock; the latency bound is a side
   effect (see `gnss_m2sdr/record_format.py`).
3. **Until then: close the loop over CSR, use DMA1 for bulk logging.** The CSR
   dump readback (`gnss_m2sdr/gateware/bank.py`, `software/gnss_tracking.py`)
   has no buffering latency, and is what the current hardware bring-up uses.
   DMA1 stays the lossless bulk path for post-processing and for the eventual
   `CorrelatorDump` push.

Related: JuliaGNSS/GNSSReceiver.jl#107 ("Overflow = ring full", "Transports").
