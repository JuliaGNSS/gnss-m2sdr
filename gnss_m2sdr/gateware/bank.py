#
# This file is part of gnss-m2sdr.
#
# GNSS tracking channel bank with CSR control + record DMA stream.
# SPDX-License-Identifier: BSD-2-Clause

"""A bank of tracking channels driven by the RX sample stream.

Each channel is CSR-controlled (carrier/code frequency words, carrier/code
phase, primary-code length, per-tap offsets, replica shape, PRN tag, runtime
code and subcarrier loading, integration restart), either
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

Three sticky per-channel health bits are exposed: `overflow` (a dump was dropped
because the recorder had not drained the previous one), `saturation` (an
accumulator hit the `accum_bits` rail) and `rate_error` (a code rate outside
what the NCO can represent was programmed). All three say "these records do not
describe the RF"; saturation and rate_error are cleared by that channel's
restart.

Signal configuration
--------------------
A channel is not tied to GPS L1 C/A. What it replicates is set at runtime:

  * `code_load` writes the primary code, chip by chip, into the channel's code
    RAM (up to the build's `max_code_length`);
  * `code_length` says how many of those chips are the code -- 330, 1023, 2046,
    4092, 5115, 10230 or anything else up to `max_code_length`;
  * `code_freq` sets the chipping rate, as `round(f_chip / fs * 2**frac_bits)`;
  * `subcarrier_load` writes the sub-chip subcarrier table and `replica.subchips`
    says how many of its entries are a chip, which is what turns the same
    channel from plain BPSK into BOC(1,1), CBOC or TMBOC (see
    gnss_m2sdr/subcarrier.py);
  * `replica.taps` picks the tap layout this channel's records report, 3 or 5,
    so one bank runs GPS L1 C/A next to Galileo E1;
  * `tap_offset_*` place each tap independently, in whole input samples times
    `code_freq`.

`code_load`, `subcarrier_load`, `code_length` and `replica` are *armed*, not
applied: a `code_load.reset_addr` or a subcarrier write raises the channel's
`loading` bit, which suppresses its dumps, and the next `restart` -- immediate
or scheduled through `apply_at` -- clears it and commits the staged shape. So a
re-assignment is atomic with respect to the record stream: no record can ever
describe a half-written code, a code read at the wrong length, or a replica
whose amplitude changed under the integration, and the first record after the
restart is the new satellite's.

`code_freq` is one bit wider than `code_frac_bits` so an unrepresentable rate
(>= 1 chip per input sample -- the code NCO crosses at most one chip boundary
per sample) arrives as a value the gateware can *detect*. It then sets
`rate_error`, and suppresses that channel's dumps, instead of tracking at the
truncated rate a narrower register would have silently produced.

`gnss_version` and `gnss_capabilities` report the CSR-layout and record-format
revisions and the build's fixed limits, so the host driver discovers what it is
talking to rather than assuming (see docs/signal_configuration.md).

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
from gnss_m2sdr.gateware.code_replica import replica_bits_for
from gnss_m2sdr.gateware.record  import CorrelatorRecorder
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH
from gnss_m2sdr.record_format import (
    CSR_LAYOUT_VERSION, MAX_SECONDARY_CODE_LENGTH, N_ANTS_MAX,
    RECORD_FORMAT_VERSION, TAPS_EPL, TAPS_VEPL, acc_signals, modulations_mask,
    tap_layouts_mask, tap_short_names,
)


class ChannelWithCSR(LiteXModule):
    """One TrackingChannel + its control/status CSRs and runtime code loader.

    Two ways to get a parameter into the channel:

    * **Immediately** -- write ``carrier_freq`` / ``code_freq`` /
      ``tap_offset_*``, pulse ``control.restart`` / ``control.carrier_set``.
      Each write takes effect on whatever sample happens to be in flight. Fine
      for bring-up, acquisition sweeps and static configuration.
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
                 carrier_phase_bits=32, max_code_length=CA_CODE_LENGTH,
                 num_ants=1, num_taps=TAPS_EPL, max_subchips=1,
                 replica_bits=None):
        len_bits  = bits_for(max_code_length)
        chip_bits = bits_for(max_code_length - 1)
        code_len_reset = min(CA_CODE_LENGTH, max_code_length)
        self.num_taps = num_taps
        acc_names     = acc_signals(num_taps)
        tap_names     = tap_short_names(num_taps)
        sub_bits      = bits_for(max_subchips)
        sub_adr_bits  = bits_for(max(1, max_subchips - 1))
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
            carrier_phase_bits=carrier_phase_bits,
            max_code_length=max_code_length, num_ants=num_ants,
            num_taps=num_taps, max_subchips=max_subchips,
            replica_bits=replica_bits)
        replica_bits = ch.code.replica_bits

        # CSRs. restart/carrier_set are edge-triggered: host writes 1 then 0;
        # a one-cycle pulse is generated on the 0->1 transition.
        self._control = CSRStorage(fields=[
            CSRField("restart",     size=1, description="0->1: reset code phase + integration."),
            CSRField("carrier_set", size=1, description="0->1: load carrier_phase."),
        ])
        self._carrier_freq  = CSRStorage(carrier_phase_bits, description="Carrier phase increment / sample.")
        self._carrier_phase = CSRStorage(carrier_phase_bits, description="Carrier phase to load on carrier_set.")
        # One bit wider than code_frac_bits so >= 1 chip/sample is representable
        # and can be rejected; see the module docstring.
        self._code_freq     = CSRStorage(code_frac_bits + 1,
            description="Code phase increment / sample, in 2**-code_frac_bits chips. "
                        "Bit code_frac_bits set means >= 1 chip/sample, which this NCO "
                        "cannot represent: the channel reports rate_error and stops "
                        "dumping instead of tracking a truncated rate.")
        self._code_length   = CSRStorage(len_bits, reset=code_len_reset,
            description="Primary-code chips (1..max_code_length). Staged: committed "
                        "by the next restart, so it changes with the code and the code "
                        "phase in one step.")
        # One signed offset per non-prompt tap, in fixed-point chips: prompt is
        # zero by definition (GNSSReceiver's contract says so, and the register
        # would have no other legal value). They are *independent* -- an
        # asymmetric layout is programmable -- because Tracking's discriminators
        # recover the spacing from the correlator they are handed, and a five-tap
        # correlator's VE/VL distance enters the discriminator separately, so
        # there is no single number to re-derive the array from. Write
        # `sample_shift * code_freq` for each: whole input samples, the grid
        # Tracking quantises onto.
        half = 1 << (code_frac_bits - 1)
        tap_reset = {"ve": 0, "e": half, "l": -half & ((1 << (code_frac_bits + 1)) - 1),
                     "vl": 0}
        self.tap_offset_csr = {}
        for name in tap_names:
            if name == "p":
                continue
            reg = CSRStorage(code_frac_bits + 1, name="tap_offset_" + name,
                             reset=tap_reset[name], description=
                f"Signed offset of the {name} tap from the prompt replica, in "
                f"2**-{code_frac_bits} chips (positive = earlier). |offset| must "
                f"stay below one chip: the taps reach chip index +/-1 only, and "
                f"exactly -1.0 chip is rejected as code_status.replica_unsupported "
                f"rather than silently landing on the wrong chip.")
            setattr(self, "_tap_offset_" + name, reg)
            self.tap_offset_csr[name] = reg
        self._prn           = CSRStorage(8, reset=prn, description="PRN tag emitted in records.")
        # atomic_write: chip+frac spans two bus words and a restart may fire
        # between them, which would load a half-written phase.
        self._code_phase    = CSRStorage(atomic_write=True, fields=[
            CSRField("frac", size=code_frac_bits, description="Fractional chip phase loaded on restart."),
            CSRField("chip", size=chip_bits,      description="Chip index loaded on restart."),
        ], description="Code phase to load on restart (0 = start of the code).")
        self._code_load     = CSRStorage(fields=[
            CSRField("dat",        size=1, description="Chip value to write."),
            CSRField("we",         size=1, description="Write dat at the current load address, then increment."),
            CSRField("reset_addr", size=1, description="Reset the load address to 0."),
            CSRField("sub",        size=1, description=
                "Subcarrier-table select stored with the chip: 0 = table A, 1 = "
                "table B. Only TMBOC sets it; every other modulation leaves it 0. "
                "Keeping it beside the chip is what frees the subcarrier from a "
                "counter that would have to stay in step with the code wrap and "
                "with every acquisition handover."),
        ])
        self._code_status   = CSRStatus(fields=[
            CSRField("loading", size=1, description=
                "A code load is in progress (set by code_load.reset_addr or by a "
                "subcarrier_load write, cleared by restart). Dumps are suppressed "
                "while it is set, so no record can describe a half-written "
                "replica."),
            CSRField("rate_unsupported", size=1, description=
                "The code rate in force is >= 1 chip/sample; dumps are suppressed."),
            CSRField("replica_unsupported", size=1, description=
                "The replica shape in force cannot be evaluated -- subchips is 0 "
                "or past the build's table, or a tap offset is a whole chip. "
                "Dumps are suppressed."),
        ], description="Live per-channel replica status.")
        # Replica shape. Staged like code_length and committed by the same
        # restart, because both change what a record means: committing either
        # mid-integration would produce one record of two different replicas.
        replica_fields = [
            CSRField("subchips", size=sub_bits, reset=1, description=
                f"Sub-chips per chip of the subcarrier table (1..{max_subchips}); "
                f"1 = plain BPSK. Staged: committed by the next restart."),
        ]
        if num_taps >= TAPS_VEPL:
            replica_fields.append(CSRField("taps", size=1, description=
                "0 = report 3 taps (E/P/L), 1 = report 5 (VE/E/P/L/VL). Staged: "
                "committed by the next restart. Present only on a five-tap build "
                "-- read gnss_signal_caps.tap_layouts first."))
        self._replica = CSRStorage(fields=replica_fields, description=
            "Replica shape staged for the next restart.")
        if max_subchips > 1:
            self._subcarrier_load = CSRStorage(fields=[
                CSRField("dat", size=replica_bits, description=
                    "Signed subcarrier amplitude for this sub-chip."),
                CSRField("adr", size=sub_adr_bits, description="Sub-chip index."),
                CSRField("lut", size=1, description="0 = table A, 1 = table B (TMBOC)."),
                CSRField("we",  size=1, description=
                    "Write dat. Also raises code_status.loading, so the next "
                    "restart is what lets records flow again -- a replica whose "
                    "amplitude changed mid-integration is not one record."),
            ], description="Subcarrier table write port.")
        self._code_length_active = CSRStatus(len_bits, reset=code_len_reset,
            description="Primary-code length actually in force (code_length as of the "
                        "last restart). Read it back to confirm a commit landed.")

        # Deterministic apply point. The staged frequency words sit in
        # write-only shadow registers until the commit, so the host can prepare
        # an update without perturbing the loop; the phase registers
        # (carrier_phase, code_phase) need no shadow because they are only
        # sampled by the load event itself, which is what gets scheduled.
        self._carrier_freq_next = CSRStorage(carrier_phase_bits,
            description="Staged carrier phase increment, committed at apply_at.")
        self._code_freq_next    = CSRStorage(code_frac_bits + 1,
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
            for k in acc_names:
                regs[k] = CSRStatus(32, name=k + suffix)
                setattr(self, "_" + k + suffix, regs[k])
            self.acc_csr.append(regs)
        self._integrated_samples = CSRStatus(32)
        self._sample_index       = CSRStatus(64)
        self._dump_code_phase    = CSRStatus(code_frac_bits)
        self._dump_code_chip     = CSRStatus(chip_bits, description=
            "Integer chip index of the replica on the last sample of the latched "
            "dump; with dump_code_phase it is the complete code phase, so the host "
            "never reconstructs the chip from the code length.")
        self._dump_saturated     = CSRStatus(1,
            description="Set if the integration behind the latched dump clamped.")
        self._dump_num_taps      = CSRStatus(8, reset=TAPS_EPL, description=
            "Taps the latched dump reports (3 or 5); the acc CSRs above the "
            "third tap read stale values when it says 3.")

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
        self.comb += [
            first_governed.eq(self.sample_count + 1),
            apply_stb.eq(armed & self.sample_stb & (first_governed >= self._apply_at.storage)),
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
        code_step_act    = Signal(code_frac_bits + 1)
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

        # The one restart pulse, immediate or scheduled. It is the channel's
        # arming point: code phase, integration, the staged code length, the
        # sticky health bits and the code-load gate all move on it together.
        restart_pulse = Signal()
        self.comb += restart_pulse.eq(
            (ctl_restart & ~restart_d) | (apply_stb & self._apply.storage[1]))

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
            ch.restart.eq(restart_pulse),
            ch.carrier_set.eq((ctl_carrier_set & ~carrier_set_d) | (apply_stb & self._apply.storage[2])),
        ]

        # Tap offsets. The CSR is an unsigned register holding a two's-complement
        # value; reinterpreting it is a same-width copy, not an arithmetic
        # conversion, so the negative (late) offsets survive intact.
        for t, name in enumerate(tap_names):
            if name == "p":
                # Prompt is the phase reference; the contract fixes it at zero and
                # there is no register to get it wrong with.
                self.comb += ch.tap_offset[t].eq(0)
            else:
                self.comb += ch.tap_offset[t].eq(
                    self.tap_offset_csr[name].storage[:code_frac_bits + 1])

        # Replica shape (sub-chip count, reported tap layout): staged and
        # committed by the arming restart, exactly like code_length.
        # `replica` storage: [sub_bits-1:0] = subchips, [sub_bits] = taps.
        # Read through `storage` slices rather than `.fields`, like every other
        # CSR here: `fields` are the bus-side registers, so a simulation poking
        # `storage` (which is how these are driven in test) would leave them at
        # their reset and the commit would silently take the wrong shape.
        subchips_act = Signal(sub_bits, reset=1)
        taps_act     = Signal(8, reset=TAPS_EPL)
        staged_taps  = (Mux(self._replica.storage[sub_bits], TAPS_VEPL, TAPS_EPL)
                        if num_taps >= TAPS_VEPL else C(TAPS_EPL, 8))
        self.sync += If(restart_pulse,
            subchips_act.eq(self._replica.storage[:sub_bits]),
            taps_act.eq(staged_taps),
        )
        self.comb += [
            ch.subchips.eq(subchips_act),
            ch.taps_cfg.eq(taps_act),
        ]

        # Subcarrier table write port. A write also opens the load window, so the
        # amplitude a record was integrated with cannot change under it.
        # `subcarrier_load` storage: dat | adr | lut | we, low to high.
        lut_write = Signal()
        if max_subchips > 1:
            st      = self._subcarrier_load.storage
            adr_lsb = replica_bits
            lut_lsb = adr_lsb + sub_adr_bits
            self.comb += [
                lut_write.eq(self._subcarrier_load.re & st[lut_lsb + 1]),
                ch.code.lut_we.eq(lut_write),
                ch.code.lut_sel.eq(st[lut_lsb]),
                ch.code.lut_adr.eq(st[adr_lsb:lut_lsb]),
                ch.code.lut_dat.eq(st[:replica_bits]),
            ]

        # Primary-code length: staged in a CSR, committed by the arming restart.
        # Committing it anywhere else would let the wrap point move under a
        # running integration, i.e. produce one record of two different codes.
        code_length_act = Signal(len_bits, reset=code_len_reset)
        self.sync += If(restart_pulse, code_length_act.eq(self._code_length.storage))
        self.comb += [
            ch.code_length.eq(code_length_act),
            self._code_length_active.status.eq(code_length_act),
        ]

        # Code-load gate. `reset_addr` is the host saying "I am about to write a
        # code", so it opens the window; the arming restart closes it. Dumps are
        # suppressed in between (channel.py), which is what makes a re-assignment
        # atomic as far as the record stream is concerned.
        loading = Signal()
        self.sync += [
            If(restart_pulse,
                loading.eq(0),
            ).Elif((self._code_load.re & self._code_load.storage[2]) | lut_write,
                loading.eq(1),
            ),
        ]
        self.comb += [
            ch.code_loading.eq(loading),
            self._code_status.status.eq(
                Cat(loading, ch.rate_unsupported, ch.replica_unsupported)),
        ]

        # Sticky "an unrepresentable code rate was in force", cleared by the
        # restart that would fix it. Sampled on the channel's strobe, so it
        # describes samples actually processed rather than a transient CSR state.
        self.rate_error = Signal()
        self.sync += [
            If(restart_pulse,
                self.rate_error.eq(0),
            ).Elif(self.sample_stb & ch.rate_unsupported,
                self.rate_error.eq(1),
            ),
        ]

        # Runtime code loader: auto-incrementing write address.
        load_addr = Signal(max=max_code_length)
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
            ch.code.load_sub.eq(self._code_load.storage[3]),
            ch.code.load_we.eq(self._code_load.re & self._code_load.storage[1]),
        ]

        # Latch dump fields into readback status registers.
        self.sync += If(ch.dump_stb,
            self._dump_count.status.eq(self._dump_count.status + 1),
            *[regs[k].status.eq(ch.acc[n][k])
              for n, regs in enumerate(self.acc_csr) for k in acc_names],
            self._integrated_samples.status.eq(ch.integrated_samples),
            self._sample_index.status.eq(ch.sample_index),
            self._dump_code_phase.status.eq(ch.dump_code_phase),
            self._dump_code_chip.status.eq(ch.dump_code_chip),
            self._dump_saturated.status.eq(ch.dump_saturated),
            self._dump_num_taps.status.eq(ch.dump_num_taps),
        )

    def connect_dump(self, port):
        ch = self.channel
        return [
            port.stb.eq(ch.dump_stb),
            *[port.acc[n][k].eq(ch.acc[n][k])
              for n in range(len(ch.acc)) for k in ch.acc_signals],
            port.num_taps.eq(ch.dump_num_taps),
            port.integrated_samples.eq(ch.integrated_samples),
            port.sample_index.eq(ch.sample_index),
            port.code_phase.eq(ch.dump_code_phase),
            port.code_chip.eq(ch.dump_code_chip),
            port.code_length.eq(ch.dump_code_length),
            port.code_step.eq(ch.dump_code_step),
            port.prn.eq(self._prn.storage),
        ]


class GNSSTracking(LiteXModule):
    """Bank of tracking channels + recorder. Observes the RX sample stream."""
    def __init__(self, n_channels=4, prns=None, code_frac_bits=24, accum_bits=32,
                 num_ants=1, max_code_length=CA_CODE_LENGTH,
                 carrier_phase_bits=32, num_taps=TAPS_EPL, max_subchips=1,
                 replica_bits=None):
        if prns is None:
            prns = [i + 1 for i in range(n_channels)]
        assert len(prns) == n_channels
        assert 1 <= num_ants <= N_ANTS_MAX, f"1..{N_ANTS_MAX} antennas"
        # The capability CSR reports these as fixed-width fields, and a host that
        # reads a truncated limit configures a channel the gateware cannot serve.
        assert 1 <= max_code_length <= 0xFFFF, "max_code_length must fit the capability CSR"
        assert code_frac_bits <= 31, (
            "code_frac_bits must leave room for the overflow bit: the code_freq "
            "CSR and the record's code_step field are code_frac_bits + 1 wide")
        # The wire format and the CSR readback both carry 32-bit accumulators
        # (record_format.py word 2..4, ChannelWithCSR's CSRStatus(32)), and
        # record.py's s32() takes the low 32 bits unconditionally -- any other
        # accum_bits would be truncated on the way out without a word of
        # warning. Fail the build instead of shipping mangled correlators.
        assert accum_bits == 32, (
            f"accum_bits must be 32 (record_format.py word layout), got {accum_bits}")
        assert num_taps in (TAPS_EPL, TAPS_VEPL), (
            f"num_taps must be one of {(TAPS_EPL, TAPS_VEPL)}, got {num_taps}")
        assert 1 <= max_subchips <= 0xFF, "max_subchips must fit its capability field"
        self.num_taps     = num_taps
        self.max_subchips = max_subchips
        if replica_bits is None:
            replica_bits = replica_bits_for(max_subchips)
        self.replica_bits = replica_bits

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
        # Sticky per-channel "an unrepresentable code rate was programmed",
        # alongside overflow and saturation. Cleared by that channel's restart.
        self._rate_error = CSRStatus(n_channels,
            description="Sticky per-channel code-rate rejection: the channel was asked "
                        "for >= 1 chip per input sample, which the code NCO cannot "
                        "represent, so its dumps were suppressed. Cleared by that "
                        "channel's restart.")

        # Host discovery. Everything below is a build-time constant; it exists so
        # the driver can declare GNSSReceiver's HardwareCorrelatorCapabilities
        # from what the gateware actually is, instead of from what it was
        # written against (see docs/signal_configuration.md).
        self._version = CSRStatus(fields=[
            CSRField("csr",    size=8, reset=CSR_LAYOUT_VERSION,
                     description="CSR-layout revision of this gateware."),
            CSRField("record", size=8, reset=RECORD_FORMAT_VERSION,
                     description="DMA1 record-format revision (record_format.py)."),
        ], description="Interface revisions. Refuse a revision you do not know.")
        self._capabilities = CSRStatus(fields=[
            CSRField("n_channels",         size=8,  reset=n_channels),
            CSRField("num_ants_max",       size=8,  reset=num_ants,
                     description="Antenna blocks a dump can carry (build-time)."),
            CSRField("num_taps",           size=8,  reset=num_taps,
                     description="Widest correlator layout this build produces "
                                 "(3 = late/prompt/early, 5 adds very late/very "
                                 "early). Narrower layouts are available per "
                                 "channel -- see signal_caps.tap_layouts."),
            CSRField("code_frac_bits",     size=8,  reset=code_frac_bits,
                     description="Fixed-point scale of code_freq / code_phase.frac."),
            CSRField("carrier_phase_bits", size=8,  reset=carrier_phase_bits,
                     description="Fixed-point scale of carrier_freq / carrier_phase."),
            CSRField("accum_bits",         size=8,  reset=accum_bits,
                     description="Accumulator width; sums saturate here, see saturation."),
            CSRField("max_code_length",    size=16, reset=max_code_length,
                     description="Code-RAM depth: the longest primary code this build "
                                 "can hold, i.e. max_primary_code_length."),
        ], description="Fixed limits of this build.")
        self._signal_caps = CSRStatus(fields=[
            CSRField("modulations", size=8, reset=modulations_mask(max_subchips),
                     description="Replica modulations this build can synthesise: "
                                 "bit0 = LOC, bit1 = BOCcos, bit2 = CBOC, "
                                 "bit3 = TMBOC, bit4 = BOCsin. Declared from the "
                                 "sub-chip table depth, so a LOC-only build reads "
                                 "bit0 alone. Check max_subchips too: one bit "
                                 "cannot distinguish BOC(1,1) from BOC(6,1)."),
            CSRField("max_secondary_code_length", size=8, reset=MAX_SECONDARY_CODE_LENGTH,
                     description="Longest overlay the gateware wipes off (1 = primary only)."),
            CSRField("reports_code_phase", size=1, reset=1,
                     description="Records carry a complete code phase (chip + fraction)."),
            CSRField("tap_layouts", size=4, reset=tap_layouts_mask(num_taps),
                     description="Tap layouts a channel can be configured for: "
                                 "bit i means 2*i+3 taps (bit0 = 3, bit1 = 5). A "
                                 "five-tap build declares both, which is what lets "
                                 "one bank mix GPS L1 C/A with Galileo E1. Bit 1 "
                                 "is also what says gnss_chN_replica.taps and the "
                                 "very-early/very-late tap offsets exist."),
            CSRField("max_subchips", size=8, reset=max_subchips,
                     description="Sub-chips per chip the subcarrier table holds. "
                                 "A modulation needs its own factor: 2 for "
                                 "BOCsin(1,1), 4 for BOCcos(1,1), 12 for "
                                 "CBOC(6,1) and TMBOC(6,1). 1 = no subcarrier."),
            CSRField("replica_bits", size=8, reset=replica_bits,
                     description="Signed width of a subcarrier table entry, so "
                                 "the host can check the amplitudes it wants to "
                                 "program fit before it writes them."),
        ], description="Signal-level capabilities.")
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
                                                      num_ants=num_ants,
                                                      max_code_length=max_code_length,
                                                      num_taps=num_taps)
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
                                  accum_bits=accum_bits, num_ants=num_ants,
                                  max_code_length=max_code_length,
                                  carrier_phase_bits=carrier_phase_bits,
                                  num_taps=num_taps, max_subchips=max_subchips,
                                  replica_bits=replica_bits)
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
        self.comb += self._rate_error.status.eq(
            Cat(*[c.rate_error for c in self.channels]))
