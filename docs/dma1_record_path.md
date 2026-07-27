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

The record is **64 bytes** (`RECORD_WORDS = 8`), not the 48 bytes the payload
needs:

- `8192 % 48 = 32`, so 48-byte records straddle buffer boundaries. The driver
  drops *whole* buffers on ring overrun, so one drop shifted every subsequent
  record by 32 bytes — permanently, with nothing to resynchronise on.
- `8192 / 64 = 128` exactly: every DMA buffer starts on a record boundary, a
  dropped buffer costs exactly 128 records, and the property survives any
  power-of-two buffer size a future per-DMA length would pick.

The upper half of word 5 carries `RECORD_MAGIC = 0x53534E47` ("GNSS" in wire
order, at byte offset 44 of every record). The stream endpoint's `first`/`last`
are set by the recorder but litepcie's DMA writer ignores them, so the host gets
no framing help from the transport; the magic is the only in-band anchor. Host
side, `find_record_offset()` locks onto the stream (mid-record attach, torn
buffer) and `parse_records()` unpacks a raw byte stream, resynchronising instead
of misparsing.

The extra 16 B/record costs 64 kB/s per channel at 1 kHz dumps — free at these
rates. Words 6 and 7 are reserved (zero) and are where a format extension
(epoch-strobe/idle records, a second antenna's accumulators) can go without
moving any existing field; bump `RECORD_MAGIC` if a layout ever changes
incompatibly.

## Latency (not fixed here — a driver/transport limit)

A buffer completes only when 128 records have been written, and `hw_count` (the
only thing `read()`/`poll()`/`LITEPCIE_IOCTL_DMA_WRITER` expose) is advanced
*only in the MSI handler* — so polling cannot beat the IRQ cadence either.

| channels | record rate | buffer completes every | MSI (every 8 buffers) |
|---|---|---|---|
| 4 | 4000 rec/s (256 kB/s) | 32 ms  | 256 ms |
| 2 | 2000 rec/s (128 kB/s) | 64 ms  | 512 ms |
| 1 | 1000 rec/s ( 64 kB/s) | 128 ms | 1.02 s |

(64-byte records make this *better* than the 48-byte figures in issue #4 —
42.7 ms/4 ch — because a buffer holds fewer, larger records.)

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
   `DMA_BUFFER_PER_IRQ = 1` gives 8 records → 2 ms at 4 channels. This is the
   real fix and it belongs in the (already required) "expose the 2nd DMA
   channel" driver work, not in gateware.
2. **Bounded-schedule filler records.** Emitting an idle/epoch record when no
   channel dumps makes buffer completion depend on wall-clock rather than on
   how many channels are locked. That is the mechanism issue #7 needs anyway,
   and words 6/7 plus a flag bit are reserved for it. It bounds the *worst*
   case (1 locked channel: 128 ms → 32 ms) but cannot get below the 32 ms floor
   set by 128 records/buffer, so it is a complement to (1), not a substitute.
3. **Until then: close the loop over CSR, use DMA1 for bulk logging.** The CSR
   dump readback (`gnss_m2sdr/gateware/bank.py`, `software/gnss_tracking.py`)
   has no buffering latency, and is what the current hardware bring-up uses.
   DMA1 stays the lossless bulk path for post-processing and for the eventual
   `CorrelatorDump` push.

Related: JuliaGNSS/GNSSReceiver.jl#107 ("Overflow = ring full", "Transports").
