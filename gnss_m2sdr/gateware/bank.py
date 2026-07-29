#
# This file is part of gnss-m2sdr.
#
# GNSS tracking channel bank with CSR control + record DMA stream.
# SPDX-License-Identifier: BSD-2-Clause

"""A bank of GPS L1 C/A tracking channels driven by the RX sample stream.

Each channel is CSR-controlled (carrier/code frequency words, carrier/code
phase, E/L spacing, PRN tag, runtime code loading, integration restart), either
immediately or -- for NCO updates and acquisition handover, where "whenever the
PCIe write landed" is not good enough -- atomically on a sample index the host
picks (``apply_at``; see ChannelWithCSR). All channels
observe the same RX sample strobe and the same free-running sample counter --
one 64-bit counter per bank, ungated and never reset, which timestamps every
dump so channels handed over at different times stay comparable (and which the
host can read over CSR to place the raw DMA0 stream on the same axis).
Correlator dumps are serialized by a CorrelatorRecorder into a 64-bit record
stream for DMA1, alongside an optional epoch strobe (`epoch_period` CSR): a
marker record every N samples of that same counter, so the host's epoch clock
does not depend on a channel being locked.

Two sticky per-channel health bits are exposed: `overflow` (a dump was dropped
because the recorder had not drained the previous one) and `saturation` (an
accumulator hit the `accum_bits` rail). Both say "these records do not describe
the RF"; saturation is cleared by that channel's restart.

With `num_ants` > 1 every channel correlates all antennas against one shared
replica set and reports one E/P/L block per antenna (see channel.py); the
control CSRs, the sample strobe, the counter, the health bits and the record's
timestamp / code_phase stay one-per-channel, since the antennas track the same
signal.
"""

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import *

from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.record  import CorrelatorRecorder
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH
from gnss_m2sdr.record_format import ACC_SIGNALS, N_ANTS_MAX


class ChannelWithCSR(LiteXModule):
    """One TrackingChannel + its control/status CSRs and runtime code loader.

    Two ways to get a parameter into the channel:

    * **Immediately** -- write ``carrier_freq`` / ``code_freq`` / ``spacing``,
      pulse ``control.restart`` / ``control.carrier_set``. Each write takes
      effect on whatever sample happens to be in flight. Fine for bring-up,
      acquisition sweeps and static configuration.
    * **At a known sample** -- stage ``carrier_freq_next`` / ``code_freq_next``
      (and ``carrier_phase`` / ``code_phase`` for the phase loads), write the
      target sample index to ``apply_at``, then arm ``apply`` with the selects
      for what to commit. Everything commits in *one* cycle, on the strobe that
      carries sample ``apply_at - 1``, so ``apply_at`` is the first input sample
      processed with the new parameters -- on the bank's global counter, the same
      axis the records are timestamped on. ``apply_status.late`` /
      ``applied_at`` report what actually happened.

    The second path is what makes an NCO update's round-trip delay a known
    number of epochs (GNSSReceiver.jl's ``NCOUpdate.apply_at_epoch``) instead of
    PCIe jitter, and what makes acquisition handover atomic: carrier frequency,
    carrier phase, code frequency and code phase all change on the same sample.
    A scheduled ``restart`` clears the accumulators on the commit strobe, one
    cycle before sample ``apply_at - 1`` reaches the accumulate stage, so that
    sample joins the new integration period; ``integrated_samples`` counts it,
    which keeps the record's ``first_sample = sample_index -
    integrated_samples + 1`` invariant intact.
    """
    def __init__(self, prn=1, code_frac_bits=24, accum_bits=32,
                 carrier_phase_bits=32, code_length=CA_CODE_LENGTH, num_ants=1):
        # One I/Q pair per antenna, all on the same strobe. Antenna 0 keeps the
        # scalar names, so single-antenna wiring is unchanged.
        self.sample_i_ants = [Signal((16, True)) for _ in range(num_ants)]
        self.sample_q_ants = [Signal((16, True)) for _ in range(num_ants)]
        self.sample_i     = self.sample_i_ants[0]
        self.sample_q     = self.sample_q_ants[0]
        self.sample_stb   = Signal()
        self.sample_count = Signal(64)   # global counter, from GNSSTracking

        self.channel = ch = TrackingChannel(
            prn=prn, code_frac_bits=code_frac_bits, accum_bits=accum_bits,
            carrier_phase_bits=carrier_phase_bits, code_length=code_length,
            num_ants=num_ants)

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
        # atomic_write: chip+frac spans two bus words and a restart may fire
        # between them, which would load a half-written phase.
        self._code_phase    = CSRStorage(atomic_write=True, fields=[
            CSRField("frac", size=code_frac_bits,            description="Fractional chip phase loaded on restart."),
            CSRField("chip", size=bits_for(code_length - 1), description="Chip index loaded on restart."),
        ], description="Code phase to load on restart (0 = start of the code).")
        self._code_load     = CSRStorage(fields=[
            CSRField("dat",        size=1, description="Chip value to write."),
            CSRField("we",         size=1, description="Write dat at the current load address, then increment."),
            CSRField("reset_addr", size=1, description="Reset the load address to 0."),
        ])

        # Deterministic apply point. The staged frequency words sit in
        # write-only shadow registers until the commit, so the host can prepare
        # an update without perturbing the loop; the phase registers
        # (carrier_phase, code_phase) need no shadow because they are only
        # sampled by the load event itself, which is what gets scheduled.
        self._carrier_freq_next = CSRStorage(carrier_phase_bits,
            description="Staged carrier phase increment, committed at apply_at.")
        self._code_freq_next    = CSRStorage(code_frac_bits,
            description="Staged code phase increment, committed at apply_at.")
        # atomic_write: a 64-bit CSR takes two bus writes, and a half-updated
        # target would be compared against the counter in between.
        self._apply_at = CSRStorage(64, atomic_write=True,
            description="Global sample index of the first sample to be processed with the staged values.")
        self._apply    = CSRStorage(fields=[
            CSRField("arm",          size=1, description="0->1: arm the commit (cleared again by the commit)."),
            CSRField("restart",      size=1, description="Commit also rebases code phase + integration."),
            CSRField("carrier_set",  size=1, description="Commit also loads carrier_phase."),
            CSRField("carrier_freq", size=1, description="Commit carrier_freq_next."),
            CSRField("code_freq",    size=1, description="Commit code_freq_next."),
        ], description="Scheduled-commit control: what the commit does. The selects say which "
                       "staged values it takes, so a carrier-only update cannot drag a stale "
                       "code word in with it. Keep all bits stable while armed.")
        self._apply_status = CSRStatus(2,
            description="bit0: a commit is pending. bit1: the last commit fired after its target "
                        "sample (host too late, so the feedback delay was longer than planned); "
                        "cleared when the next commit is armed.")
        self._applied_at = CSRStatus(64,
            description="Sample index actually governed by the last commit (== apply_at unless late).")

        # Correlator-dump readback (latched on each dump). Driver-free way to
        # run/validate the tracking loop over RemoteClient. For a coherent read,
        # sample dump_count, read the fields, then re-read dump_count.
        # Antenna 0 keeps the bare names (ip, qp, ...) the host tooling already
        # reads; further antennas are suffixed (ip_ant1, ...). The CSR names are
        # passed explicitly because the frame-inspecting default cannot see
        # through the loop.
        self._dump_count = CSRStatus(32, description="Increments on each correlator dump.")
        self.acc_csr = []
        for n in range(num_ants):
            suffix = "" if n == 0 else f"_ant{n}"
            regs = {}
            for k in ACC_SIGNALS:
                regs[k] = CSRStatus(32, name=k + suffix)
                setattr(self, "_" + k + suffix, regs[k])
            self.acc_csr.append(regs)
        self._integrated_samples = CSRStatus(32)
        self._sample_index       = CSRStatus(64)
        self._dump_code_phase    = CSRStatus(code_frac_bits)
        self._dump_saturated     = CSRStatus(1,
            description="Set if the integration behind the latched dump clamped.")

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

        # Scheduled commit. `apply_at` names the first sample to be processed
        # with the staged values, so the commit has to fire one sample earlier:
        # the NCOs advance *into* that sample on the strobe that carries
        # apply_at-1, and a phase load done on that strobe is what the next
        # sample sees. Compare with >= (not ==) so an already-passed target
        # commits on the next strobe with `late` set, instead of the channel
        # waiting 2**64 samples for a compare that can never match. The compare
        # is against the channel's (enable-gated) strobe: with the bank disabled
        # no sample is being processed, so the commit waits for the next one.
        armed     = Signal()
        late      = Signal()
        apply_stb = Signal()
        arm_bit   = self._apply.storage[0]
        arm_d     = Signal()
        first_governed = Signal(64)     # sample index the commit takes effect for
        # `sample_count + 1 >= apply_at` is the same test as
        # `sample_count >= apply_at - 1`, and the second form keeps the 64-bit
        # incrementer out of the path that decides `apply_stb`. That matters:
        # `apply_stb` gates `restart` / `carrier_set` / the frequency-word muxes,
        # so it lands on the code NCO's chip index and the carrier phase in the
        # same cycle. Adder-then-comparator put ~23 carry stages in series ahead
        # of all of that and it was the worst setup path in the whole SoC (the
        # 4-channel build missed by 167 ps, with the code chip index among the
        # failing endpoints). Pre-decrementing turns it into a single 64-bit
        # compare against a value that only changes when the host writes
        # `apply_at`, i.e. never while it matters.
        # Re-registered every cycle rather than on `_apply_at.re`, so nothing
        # depends on whether `storage` updates with `re` or a cycle after it. The
        # one-cycle lag behind `apply_at` cannot be observed: arming takes a
        # second, later CSR write (`apply.arm`), by which time this has settled.
        apply_at_m1 = Signal(64)
        self.sync += If(self._apply_at.storage == 0,
            # apply_at == 0 has no predecessor; keep the old semantics for it (a
            # target already in the past commits on the next strobe with `late`
            # set) rather than wrapping to 2**64-1 and never firing at all.
            apply_at_m1.eq(0),
        ).Else(
            apply_at_m1.eq(self._apply_at.storage - 1),
        )
        # On top of the pre-decrement, the compare itself is REGISTERED: at 20
        # channels even the bare 64-bit `>=` fans combinationally into every
        # channel's restart/carrier_set gates and NCO muxes and drags the whole
        # sys_clk floorplan under (measured -0.5 ns class with thousands of
        # endpoints; registering it closed the same build to -0.07 ns).
        # Registering must stay exact under back-to-back strobes -- the raw
        # stream drains out of the DMA0 FIFO in sys-rate bursts, so
        # `sample_count` can advance every cycle. The precomputation therefore
        # folds in this cycle's increment: next cycle the strobe (if any) sees
        # count' = count + stb and must fire iff count' >= apply_at_m1.
        reach_r = Signal()
        self.sync += reach_r.eq((self.sample_count + self.sample_stb) >= apply_at_m1)
        self.comb += [
            # Still exposed for `late` / `applied_at`, but no longer feeding
            # `apply_stb`, so its carry chain is off the critical path.
            first_governed.eq(self.sample_count + 1),
            apply_stb.eq(armed & self.sample_stb & reach_r),
            self._apply_status.status.eq(Cat(armed, late)),
        ]
        self.sync += [
            arm_d.eq(arm_bit),
            If(apply_stb,
                armed.eq(0),
                late.eq(first_governed != self._apply_at.storage),
                self._applied_at.status.eq(first_governed),
            ).Elif(arm_bit & ~arm_d,
                armed.eq(1),
                late.eq(0),
            ),
        ]

        # Committed frequency words. Each stays in force until the host writes
        # its immediate CSR again (`re`), so staging the *next* update cannot
        # leak in ahead of its own apply point.
        # apply storage: bit0=arm, 1=restart, 2=carrier_set, 3=carrier_freq, 4=code_freq.
        apply_carrier_fw = Signal()
        apply_code_fw    = Signal()
        carrier_fw_act   = Signal(carrier_phase_bits)
        code_step_act    = Signal(code_frac_bits)
        sched_carrier    = Signal()
        sched_code       = Signal()
        self.comb += [
            apply_carrier_fw.eq(apply_stb & self._apply.storage[3]),
            apply_code_fw.eq(apply_stb & self._apply.storage[4]),
        ]
        self.sync += [
            If(apply_carrier_fw, carrier_fw_act.eq(self._carrier_freq_next.storage)),
            If(apply_code_fw,    code_step_act.eq(self._code_freq_next.storage)),
            If(apply_carrier_fw, sched_carrier.eq(1)).Elif(self._carrier_freq.re, sched_carrier.eq(0)),
            If(apply_code_fw,    sched_code.eq(1)).Elif(self._code_freq.re,       sched_code.eq(0)),
        ]

        self.comb += [
            *[ch.sample_i_ants[n].eq(self.sample_i_ants[n]) for n in range(num_ants)],
            *[ch.sample_q_ants[n].eq(self.sample_q_ants[n]) for n in range(num_ants)],
            ch.sample_stb.eq(self.sample_stb),
            ch.sample_count.eq(self.sample_count),
            # On the apply cycle the staged word must reach the NCO
            # combinationally -- the phase advance into apply_at happens on that
            # very strobe -- so bypass the register that latches it.
            ch.carrier_fw.eq(Mux(apply_carrier_fw, self._carrier_freq_next.storage,
                             Mux(sched_carrier, carrier_fw_act, self._carrier_freq.storage))),
            ch.code_step.eq(Mux(apply_code_fw, self._code_freq_next.storage,
                            Mux(sched_code, code_step_act, self._code_freq.storage))),
            ch.carrier_phase_in.eq(self._carrier_phase.storage),
            # code_phase storage: [code_frac_bits-1:0]=frac, above it=chip.
            ch.code_phase_frac.eq(self._code_phase.storage[:code_frac_bits]),
            ch.code_phase_chip.eq(self._code_phase.storage[code_frac_bits:]),
            ch.spacing.eq(self._spacing.storage),
            ch.restart.eq((ctl_restart & ~restart_d) | (apply_stb & self._apply.storage[1])),
            ch.carrier_set.eq((ctl_carrier_set & ~carrier_set_d) | (apply_stb & self._apply.storage[2])),
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
            *[regs[k].status.eq(ch.acc[n][k])
              for n, regs in enumerate(self.acc_csr) for k in ACC_SIGNALS],
            self._integrated_samples.status.eq(ch.integrated_samples),
            self._sample_index.status.eq(ch.sample_index),
            self._dump_code_phase.status.eq(ch.dump_code_phase),
            self._dump_saturated.status.eq(ch.dump_saturated),
        )

    def connect_dump(self, port):
        ch = self.channel
        return [
            port.stb.eq(ch.dump_stb),
            *[port.acc[n][k].eq(ch.acc[n][k])
              for n in range(len(ch.acc)) for k in ACC_SIGNALS],
            port.integrated_samples.eq(ch.integrated_samples),
            port.sample_index.eq(ch.sample_index),
            port.code_phase.eq(ch.dump_code_phase),
            port.prn.eq(self._prn.storage),
        ]


class GNSSTracking(LiteXModule):
    """Bank of tracking channels + recorder. Observes the RX sample stream.

    ``attach_channels`` decides who owns the channels in the CSR hierarchy.
    ``True`` (the default, and what every simulation test uses) makes each
    channel a submodule of this bank, so all per-channel CSRs land in the one
    ``gnss`` CSR bank. That bank pages at 0x800 bytes = 512 CSR words, which a
    channel's ~29 words exhaust at 17 channels — and LiteX does **not** fail
    the build when a bank outgrows its page: it emits a csr.csv whose upper
    registers silently overlap the next bank (measured: a 20-channel build put
    ``gnss`` registers on top of ``header``). For real builds the SoC therefore
    passes ``attach_channels=False`` and attaches every channel itself as its
    own SoC-level CSR bank named ``gnss_ch<i>`` — the emitted register names
    (``gnss_ch<i>_<csr>``) are identical either way, so no host software can
    tell the difference.
    """
    def __init__(self, n_channels=4, prns=None, code_frac_bits=24, accum_bits=32,
                 num_ants=1, attach_channels=True):
        if prns is None:
            prns = [i + 1 for i in range(n_channels)]
        assert len(prns) == n_channels
        assert 1 <= num_ants <= N_ANTS_MAX, f"1..{N_ANTS_MAX} antennas"
        # The wire format and the CSR readback both carry 32-bit accumulators
        # (record_format.py word 2..4, ChannelWithCSR's CSRStatus(32)), and
        # record.py's s32() takes the low 32 bits unconditionally -- any other
        # accum_bits would be truncated on the way out without a word of
        # warning. Fail the build instead of shipping mangled correlators.
        assert accum_bits == 32, (
            f"accum_bits must be 32 (record_format.py word layout), got {accum_bits}")

        # One I/Q pair per antenna, all on the same strobe (antenna 0 also under
        # the scalar names).
        self.sample_i_ants = [Signal((16, True)) for _ in range(num_ants)]
        self.sample_q_ants = [Signal((16, True)) for _ in range(num_ants)]
        self.sample_i   = self.sample_i_ants[0]
        self.sample_q   = self.sample_q_ants[0]
        self.sample_stb = Signal()
        # How many antennas the sample stream currently carries: the build-time
        # count unless something upstream knows better (the RX observer lowers
        # it to 1 in the AD9361's 1R1T mode, where the two slots of a word are
        # consecutive samples of one antenna). Left undriven it reads its reset.
        self.ants_valid = Signal(max=num_ants + 1, reset=num_ants)

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
        # Bit i is channel i; bit n_channels is the epoch strobe, which is a
        # recorder slot like any other and so gets the same sticky bit, the same
        # write-1-to-clear and its own drop counter (droppedstrobe).
        n_slots = n_channels + 1
        self._overflow = CSRStatus(n_slots,
            description="Sticky per-slot record overflow (bit n_channels = epoch strobe); "
                        "cleared only via overflow_clear.")
        self._overflow_clear = CSRStorage(n_slots,
            description="Write 1 to a bit to clear that slot's overflow bit + drop counter.")
        # Saturation stays per *channel*: the strobe slot has no accumulators.
        self._saturation = CSRStatus(n_channels,
            description="Sticky per-channel accumulator saturation (cleared by that channel's restart).")
        # Epoch strobe: a timebase marker record every N input samples, so the
        # host can close an epoch without waiting for a satellite to dump (see
        # record_format.py). Ungated by `enable` -- the case it exists for is
        # precisely "no channel is producing anything". 0 = off, so a build the
        # host never configures streams exactly what it did before.
        self._epoch_period = CSRStorage(32, reset=0,
            description="Epoch-strobe period in input samples (0 = no strobe records).")
        self._num_ants = CSRStatus(8, description=
            "Antennas currently reported per dump (1..N_ANTS_MAX); host discovery.")
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

        self.recorder = recorder = CorrelatorRecorder(n_channels, accum_bits, code_frac_bits,
                                                      num_ants=num_ants)
        self.source = recorder.source
        self.comb += [
            recorder.sample_stb.eq(self.sample_stb),      # ungated on purpose
            recorder.sample_count.eq(self.sample_count),
            recorder.epoch_period.eq(self._epoch_period.storage),
            recorder.num_ants.eq(self.ants_valid),
            self._num_ants.status.eq(self.ants_valid),
        ]

        gated_stb = Signal()
        self.comb += gated_stb.eq(self.sample_stb & self._control.storage[0])  # enable

        self.channels = []
        for i in range(n_channels):
            chan = ChannelWithCSR(prn=prns[i], code_frac_bits=code_frac_bits,
                                  accum_bits=accum_bits, num_ants=num_ants)
            if attach_channels:
                setattr(self.submodules, f"ch{i}", chan)
            self.channels.append(chan)
            self.comb += [
                *[chan.sample_i_ants[n].eq(self.sample_i_ants[n]) for n in range(num_ants)],
                *[chan.sample_q_ants[n].eq(self.sample_q_ants[n]) for n in range(num_ants)],
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

        # ... and one for the strobe slot, on the same footing as a channel's.
        self._droppedstrobe = CSRStatus(len(recorder.dropped[n_channels]),
            description="Saturating count of epoch strobes dropped.")
        self.comb += self._droppedstrobe.status.eq(recorder.dropped[n_channels])

        # Write-1-to-clear: `re` pulses for one cycle with `storage` already
        # holding the written mask (same pattern as the code_load strobe above).
        self.comb += [
            self._overflow.status.eq(recorder.overflow),
            recorder.overflow_clear.eq(self._overflow_clear.storage
                                       & Replicate(self._overflow_clear.re, n_slots)),
        ]
        # Saturation is reported per channel next to overflow: both mean "the
        # records you are reading are not what the RF actually correlated to".
        self.comb += self._saturation.status.eq(
            Cat(*[c.channel.saturated for c in self.channels]))
