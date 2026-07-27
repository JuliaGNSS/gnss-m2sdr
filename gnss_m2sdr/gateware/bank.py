#
# This file is part of gnss-m2sdr.
#
# GNSS tracking channel bank with CSR control + record DMA stream.
# SPDX-License-Identifier: BSD-2-Clause

"""A bank of GPS L1 C/A tracking channels driven by the RX sample stream.

Each channel is CSR-controlled (carrier/code frequency words, carrier phase,
E/L spacing, PRN tag, runtime code loading, integration restart). All channels
observe the same RX sample strobe and the same free-running sample counter --
one 64-bit counter per bank, ungated and never reset, which timestamps every
dump so channels handed over at different times stay comparable (and which the
host can read over CSR to place the raw DMA0 stream on the same axis).
Correlator dumps are serialized by a CorrelatorRecorder into a 64-bit record
stream for DMA1.
"""

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import *

from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.record  import CorrelatorRecorder, ChannelDumpPort
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH


class ChannelWithCSR(LiteXModule):
    """One TrackingChannel + its control/status CSRs and runtime code loader."""
    def __init__(self, prn=1, code_frac_bits=24, accum_bits=32,
                 carrier_phase_bits=32, code_length=CA_CODE_LENGTH):
        self.sample_i     = Signal((16, True))
        self.sample_q     = Signal((16, True))
        self.sample_stb   = Signal()
        self.sample_count = Signal(64)   # global counter, from GNSSTracking

        self.channel = ch = TrackingChannel(
            prn=prn, code_frac_bits=code_frac_bits, accum_bits=accum_bits,
            carrier_phase_bits=carrier_phase_bits, code_length=code_length)

        # CSRs. restart/carrier_set are edge-triggered: host writes 1 then 0;
        # a one-cycle pulse is generated on the 0->1 transition.
        self._control = CSRStorage(fields=[
            CSRField("restart",     size=1, description="0->1: reset code phase + integration."),
            CSRField("carrier_set", size=1, description="0->1: load carrier_phase."),
        ])
        self._carrier_freq  = CSRStorage(carrier_phase_bits, description="Carrier phase increment / sample.")
        self._carrier_phase = CSRStorage(carrier_phase_bits, description="Carrier phase to load on carrier_set.")
        self._code_freq     = CSRStorage(code_frac_bits, description="Code phase increment / sample.")
        self._spacing       = CSRStorage(code_frac_bits, reset=(1 << (code_frac_bits - 1)),
                                          description="E/L half spacing in chips (fixed-point). Default 0.5.")
        self._prn           = CSRStorage(8, reset=prn, description="PRN tag emitted in records.")
        self._code_load     = CSRStorage(fields=[
            CSRField("dat",        size=1, description="Chip value to write."),
            CSRField("we",         size=1, description="Write dat at the current load address, then increment."),
            CSRField("reset_addr", size=1, description="Reset the load address to 0."),
        ])

        # Correlator-dump readback (latched on each dump). Driver-free way to
        # run/validate the tracking loop over RemoteClient. For a coherent read,
        # sample dump_count, read the fields, then re-read dump_count.
        self._dump_count = CSRStatus(32, description="Increments on each correlator dump.")
        self._ip = CSRStatus(32); self._qp = CSRStatus(32)
        self._ie = CSRStatus(32); self._qe = CSRStatus(32)
        self._il = CSRStatus(32); self._ql = CSRStatus(32)
        self._integrated_samples = CSRStatus(32)
        self._sample_index       = CSRStatus(64)
        self._dump_code_phase    = CSRStatus(code_frac_bits)

        # # #

        # Edge-detect restart / carrier_set (0->1 -> one-cycle pulse).
        # Use storage bit-slices directly (bit0=restart, bit1=carrier_set).
        ctl_restart     = self._control.storage[0]
        ctl_carrier_set = self._control.storage[1]
        restart_d, carrier_set_d = Signal(), Signal()
        self.sync += [
            restart_d.eq(ctl_restart),
            carrier_set_d.eq(ctl_carrier_set),
        ]
        self.comb += [
            ch.sample_i.eq(self.sample_i),
            ch.sample_q.eq(self.sample_q),
            ch.sample_stb.eq(self.sample_stb),
            ch.sample_count.eq(self.sample_count),
            ch.carrier_fw.eq(self._carrier_freq.storage),
            ch.carrier_phase_in.eq(self._carrier_phase.storage),
            ch.code_step.eq(self._code_freq.storage),
            ch.spacing.eq(self._spacing.storage),
            ch.restart.eq(ctl_restart & ~restart_d),
            ch.carrier_set.eq(ctl_carrier_set & ~carrier_set_d),
        ]

        # Runtime code loader: auto-incrementing write address.
        load_addr = Signal(max=code_length)
        self.sync += [
            If(self._code_load.re,
                If(self._code_load.storage[2],      # reset_addr
                    load_addr.eq(0),
                ).Elif(self._code_load.storage[1],   # we
                    load_addr.eq(load_addr + 1),
                ),
            ),
        ]
        # code_load storage: bit0=dat, bit1=we, bit2=reset_addr.
        self.comb += [
            ch.code.load_adr.eq(load_addr),
            ch.code.load_dat.eq(self._code_load.storage[0]),
            ch.code.load_we.eq(self._code_load.re & self._code_load.storage[1]),
        ]

        # Latch dump fields into readback status registers.
        self.sync += If(ch.dump_stb,
            self._dump_count.status.eq(self._dump_count.status + 1),
            self._ip.status.eq(ch.ip), self._qp.status.eq(ch.qp),
            self._ie.status.eq(ch.ie), self._qe.status.eq(ch.qe),
            self._il.status.eq(ch.il), self._ql.status.eq(ch.ql),
            self._integrated_samples.status.eq(ch.integrated_samples),
            self._sample_index.status.eq(ch.sample_index),
            self._dump_code_phase.status.eq(ch.dump_code_phase),
        )

    def connect_dump(self, port):
        ch = self.channel
        return [
            port.stb.eq(ch.dump_stb),
            port.ie.eq(ch.ie), port.qe.eq(ch.qe),
            port.ip.eq(ch.ip), port.qp.eq(ch.qp),
            port.il.eq(ch.il), port.ql.eq(ch.ql),
            port.integrated_samples.eq(ch.integrated_samples),
            port.sample_index.eq(ch.sample_index),
            port.code_phase.eq(ch.dump_code_phase),
            port.prn.eq(self._prn.storage),
        ]


class GNSSTracking(LiteXModule):
    """Bank of tracking channels + recorder. Observes the RX sample stream."""
    def __init__(self, n_channels=4, prns=None, code_frac_bits=24, accum_bits=32):
        if prns is None:
            prns = [i + 1 for i in range(n_channels)]
        assert len(prns) == n_channels

        self.sample_i   = Signal((16, True))
        self.sample_q   = Signal((16, True))
        self.sample_stb = Signal()

        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, description="Enable sample processing in all channels."),
        ])
        # Overflow status a host can actually poll: the bit stays set until the
        # host writes 1 to the matching bit of overflow_clear. Self-clearing on
        # the next captured dump would leave it observable for under a
        # millisecond at ~1 kHz dumps, i.e. invisible to any realistic poll
        # rate. droppedN counts the lost dumps so "one missed epoch" and "the
        # loop stalled for 200 ms" are distinguishable; the per-record
        # FLAG_OVERFLOW remains the transient, per-dump marker.
        self._overflow = CSRStatus(n_channels,
            description="Sticky per-channel record overflow; cleared only via overflow_clear.")
        self._overflow_clear = CSRStorage(n_channels,
            description="Write 1 to a bit to clear that channel's overflow bit + drop counter.")
        # The one time axis: a free-running count of observed sample strobes,
        # ungated by `enable` and never reset (not by a channel restart either),
        # so every channel's dumps -- and the raw DMA0 stream, which the host
        # relates to it through this CSR -- share one origin. Reads as the
        # 0-based index of the next sample; during a strobe cycle it reads the
        # 0-based index of the sample being presented, which is what channels
        # latch. Doubles as the RX-observer liveness diagnostic.
        # A 64-bit CSR read is not atomic: read the high word, the low word,
        # then the high word again and retry if it changed.
        self.sample_count = Signal(64)
        self._sample_count = CSRStatus(64,
            description="Global free-running input-sample counter (also RX-observer liveness).")
        self.sync += If(self.sample_stb, self.sample_count.eq(self.sample_count + 1))
        self.comb += self._sample_count.status.eq(self.sample_count)

        # # #

        self.recorder = recorder = CorrelatorRecorder(n_channels, accum_bits, code_frac_bits)
        self.source = recorder.source

        gated_stb = Signal()
        self.comb += gated_stb.eq(self.sample_stb & self._control.storage[0])  # enable

        self.channels = []
        for i in range(n_channels):
            chan = ChannelWithCSR(prn=prns[i], code_frac_bits=code_frac_bits, accum_bits=accum_bits)
            setattr(self.submodules, f"ch{i}", chan)
            self.channels.append(chan)
            self.comb += [
                chan.sample_i.eq(self.sample_i),
                chan.sample_q.eq(self.sample_q),
                chan.sample_stb.eq(gated_stb),
                chan.sample_count.eq(self.sample_count),
            ]
            self.comb += chan.connect_dump(recorder.ports[i])
            # One drop counter per channel (name must be explicit: the tracer
            # cannot derive a CSR name from a loop variable).
            dropped = CSRStatus(len(recorder.dropped[i]), name=f"dropped{i}",
                description=f"Saturating count of dumps dropped on channel {i}.")
            setattr(self, f"_dropped{i}", dropped)
            self.comb += dropped.status.eq(recorder.dropped[i])

        # Write-1-to-clear: `re` pulses for one cycle with `storage` already
        # holding the written mask (same pattern as the code_load strobe above).
        self.comb += [
            self._overflow.status.eq(recorder.overflow),
            recorder.overflow_clear.eq(self._overflow_clear.storage
                                       & Replicate(self._overflow_clear.re, n_channels)),
        ]
