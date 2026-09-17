#
# This file is part of gnss-m2sdr.
#
# Single GNSS tracking channel: carrier wipe-off + E/P/L correlation + I&D.
# SPDX-License-Identifier: BSD-2-Clause

"""One tracking channel, over `num_ants` coherent antennas.

Per enabled input sample (I, Q) of every antenna:
  1. Carrier NCO -> cos/sin replica; complex mix (multiply by conjugate) to
     wipe off the residual carrier:  I_bb = I*cos + Q*sin,  Q_bb = Q*cos - I*sin.
  2. Code NCO -> one replica sample per tap (VE/E/P/L/VL), each the primary chip
     times its sub-chip subcarrier value (see code_replica.py). For a plain
     BPSK (:LOC) channel that is still +/-1.
  3. Accumulate replica * baseband into two I&D accumulators per tap (I and Q).

The accumulators saturate at `accum_bits` rather than wrapping: a wrapped sum
still looks like a valid correlator value to the host, so overflow would be
silent. `dump_saturated` marks the dump whose integration clamped, and
`saturated` is the sticky per-channel version (cleared by `restart`) that the
bank exposes over CSR. Both are per channel, not per antenna: a clamp anywhere
in the array invalidates the dump the host would beamform.

Steps 1 and 2 need the carrier NCO, the code NCO and the code replicas --
by far the expensive part -- and all antennas of a coherent array track the
*same* signal (shared LO, only the spatial phase differs), so those are
instantiated once and shared; only the accumulators (two per tap) and their
multipliers are per antenna. `num_ants` therefore scales cheaply, which is why the
accumulators must stay separate: GNSSReceiver.jl#107 beamforms
post-correlation on the CPU from the per-antenna prompt covariance, so summing
antennas here would destroy the spatial information (and one NCOUpdate per
channel, not per antenna, is what #107 specifies). Antenna 0's ports keep their
scalar names, so single-antenna wiring is unchanged.

Taps
----
`num_taps` is 3 (Early/Prompt/Late) or 5 (adding Very Early/Very Late), fixed at
build time -- it sizes the accumulators. Which of them a *record* carries is a
separate, per-channel runtime choice (`taps_cfg`), because Tracking uses three
taps for GPS L1 C/A and five for the BOC-family signals and one bank has to run
both at once. A five-tap build serving a three-tap channel simply does not
report VE/VL; the accumulators are there either way, so nothing about the
integration changes with the setting.

On each code epoch (one primary-code period -- `code_length` chips, runtime
programmable) the accumulators are latched to the
dump registers, the integrated-sample count and the sample-counter value are
captured, and the accumulators reset. The dump maps onto Tracking.jl's
CorrelatorOutput(EarlyPromptLateCorrelator(SVector(late, prompt, early), spacing),
integrated_samples, sample_index) -- note that Tracking.jl orders its
accumulators latest-first, so E and L go in reversed relative to the names used
here; see record_format.py for why getting that backwards inverts the DLL. The
dumped `code_phase` is extra device-side metadata: Tracking.jl's CorrelatorOutput
has no code_phase field or keyword, so the host carries it out of band. It is
reported *completely* -- `dump_code_chip` (integer chip index) next to
`dump_code_phase` (fractional chip) -- so the host never has to reconstruct the
chip from "the dump fires on the wrap, so it must be code_length - 1". That
reconstruction is what hard-codes 1022 into a host, and it stops being true the
moment a dump is shorter than a primary period (GNSSReceiver.jl#133).

Two conditions make a dump meaningless rather than merely bad, and both raise
`inhibit`, which suppresses the dump strobe entirely instead of emitting a
record the host would have to second-guess:

  * the code RAM is mid-load (`code_loading` from the bank) -- the replica is
    then part one satellite and part another;
  * `code_step` asks for >= 1 chip per input sample (`rate_unsupported`), which
    this NCO cannot represent: its accumulator carries at most one chip
    boundary per sample. The step word is one bit wider than the fraction
    precisely so that an unrepresentable rate arrives as a detectable value
    rather than as a silently truncated one;
  * the replica shape cannot be evaluated (`replica_unsupported` from the code
    replica) -- a sub-chip count outside the table, or a tap offset of a whole
    chip, both of which would otherwise yield a replica that looks like a
    replica and is not.

The timestamp is NOT generated here: `sample_count` is an input, driven by the
one free-running counter shared by every channel (and by the raw stream) in
GNSSTracking, so dumps from channels restarted at different times stay on a
single time axis. `restart` therefore rebases only the code phase -- onto the
`code_phase_chip`/`code_phase_frac` inputs, so an acquisition handover can start
on the code phase the CPU measured -- and the integration accumulators, never
the timestamp. `sample_index` is the 0-based
global index of the last sample included in the integration; the host maps it
to Tracking.jl's 1-based per-chunk convention with
`sample_index_julia = sample_index - chunk_origin + 1` (see record_format.py).
"""

from migen import *

from litex.gen import *

from gnss_m2sdr.gateware.carrier_nco import CarrierNCO
from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH
from gnss_m2sdr.record_format import (
    N_ANTS_MAX, TAPS_EPL, acc_signals, tap_short_names,
)


class TrackingChannel(LiteXModule):
    def __init__(self, prn=1, sample_bits=16, carrier_phase_bits=32,
                 carrier_lut_addr_bits=8, carrier_amp_bits=8,
                 code_frac_bits=24, accum_bits=32,
                 max_code_length=CA_CODE_LENGTH, num_ants=1, code_init=None,
                 num_taps=TAPS_EPL, max_subchips=1, replica_bits=None,
                 staged_length=False):
        assert 1 <= num_ants <= N_ANTS_MAX, f"1..{N_ANTS_MAX} antennas"
        self.max_code_length = max_code_length
        self.num_taps        = num_taps
        acc_names            = acc_signals(num_taps)
        self.acc_signals     = acc_names

        # Sample inputs, one I/Q pair per antenna. All antennas are presented on
        # the same sample_stb: they sample simultaneously off the shared LO, so
        # one strobe carries one time instant across the array.
        # sample_count is the global free-running input-sample counter (0-based
        # index of the sample presented this strobe), owned by GNSSTracking and
        # shared by all channels.
        self.sample_i_ants = [Signal((sample_bits, True)) for _ in range(num_ants)]
        self.sample_q_ants = [Signal((sample_bits, True)) for _ in range(num_ants)]
        self.sample_i     = self.sample_i_ants[0]   # same Signal, not a copy
        self.sample_q     = self.sample_q_ants[0]
        self.sample_stb   = Signal()
        self.sample_count = Signal(64)

        # Control (host / acquisition feedback).
        self.carrier_fw    = Signal(carrier_phase_bits)  # carrier phase increment / sample
        self.carrier_set   = Signal()                    # load carrier_phase_in
        self.carrier_phase_in = Signal(carrier_phase_bits)
        # Code phase increment per sample, in code_frac_bits fixed-point chips.
        # One bit wider than the fraction: >= 1.0 chips/sample is outside what
        # the NCO can represent, and it has to be *representable on the wire* to
        # be rejected rather than truncated into a plausible-looking rate.
        self.code_step     = Signal(code_frac_bits + 1)
        self.code_length   = Signal(bits_for(max_code_length),
                                    reset=min(CA_CODE_LENGTH, max_code_length))
        # The length the next `restart` wraps with. The bank drives it from the
        # staged CSR; left alone it follows `code_length` (see code_replica.py).
        self.restart_length = Signal(bits_for(max_code_length),
                                     reset=min(CA_CODE_LENGTH, max_code_length))
        # Sub-chips per chip of the replica in force (1 = plain BPSK).
        self.subchips      = Signal(bits_for(max_subchips), reset=1)
        # One signed tap offset per tap, earliest first, in fixed-point chips.
        self.tap_offset    = [Signal((code_frac_bits + 1, True))
                              for _ in range(num_taps)]
        # Correlator taps this channel's records report (3 or 5). It selects what
        # the record says and which accumulators it carries, not what is
        # integrated -- see the Taps section above.
        self.taps_cfg      = Signal(8, reset=TAPS_EPL)
        self.restart       = Signal()                    # rebase code phase + integration
        # Suppress dumps: the code RAM is being rewritten, or the programmed
        # code rate is unrepresentable. Driven by the bank.
        self.code_loading  = Signal()
        # Code phase loaded by `restart` (0/0 = start of the code).
        self.code_phase_chip = Signal(max=max_code_length)
        self.code_phase_frac = Signal(code_frac_bits)

        # High while `code_step` asks for >= 1 chip per input sample.
        self.rate_unsupported = Signal()
        # High while the programmed replica shape cannot be evaluated.
        self.replica_unsupported = Signal()

        # Dump outputs (valid for one cycle when dump_stb high, then held).
        # acc[n] holds antenna n's accumulators, two per tap; antenna 0's are
        # also exposed under the original scalar names (ip/qp/ie/qe/il/ql, plus
        # ive/qve/ivl/qvl on a five-tap build).
        self.dump_stb           = Signal()
        self.dump_saturated     = Signal()  # the dumped integration clamped
        self.acc = [{k: Signal((accum_bits, True)) for k in acc_names}
                    for _ in range(num_ants)]
        for k, sig in self.acc[0].items():
            setattr(self, k, sig)
        self.integrated_samples = Signal(32)
        self.sample_index       = Signal(64)
        self.dump_code_phase    = Signal(code_frac_bits)
        # Complete code phase + the configuration that produced the dump, so a
        # record describes itself (see record_format.py).
        self.dump_code_chip     = Signal(max=max_code_length)
        self.dump_code_length   = Signal(bits_for(max_code_length))
        self.dump_code_step     = Signal(code_frac_bits + 1)
        self.dump_num_taps      = Signal(8)

        # Sticky "an accumulator hit the rail since the last restart" status,
        # for the bank's saturation CSR. Unlike dump_saturated this survives
        # across dumps, so a host polling at its own pace cannot miss it.
        self.saturated = Signal()

        # # #

        # Sub-modules.
        self.carrier = carrier = CarrierNCO(carrier_phase_bits, carrier_lut_addr_bits, carrier_amp_bits)
        self.code    = code    = CodeReplica(prn=prn, frac_bits=code_frac_bits,
                                             max_code_length=max_code_length,
                                             code_init=code_init,
                                             num_taps=num_taps,
                                             max_subchips=max_subchips,
                                             replica_bits=replica_bits,
                                             staged_length=staged_length)
        replica_bits = code.replica_bits
        # An out-of-range step still drives the NCO with its low bits (the
        # replica keeps running at *some* rate rather than freezing), but every
        # dump it would produce is inhibited, so nothing truncated reaches the
        # host.
        self.comb += self.rate_unsupported.eq(self.code_step[code_frac_bits])
        self.comb += self.replica_unsupported.eq(code.replica_unsupported)
        inhibit = Signal()
        self.comb += inhibit.eq(self.code_loading | self.rate_unsupported
                                | self.replica_unsupported)
        self.comb += [
            carrier.freq_word.eq(self.carrier_fw),
            carrier.stb.eq(self.sample_stb),
            carrier.set_phase.eq(self.carrier_set),
            carrier.phase_in.eq(self.carrier_phase_in),
            code.code_step.eq(self.code_step[:code_frac_bits]),
            code.code_length.eq(self.code_length),
            code.subchips.eq(self.subchips),
            *[code.tap_offset[t].eq(self.tap_offset[t]) for t in range(num_taps)],
            code.stb.eq(self.sample_stb),
            code.restart.eq(self.restart),
            code.restart_chip.eq(self.code_phase_chip),
            code.restart_frac.eq(self.code_phase_frac),
        ]
        if staged_length:
            self.comb += code.restart_length.eq(self.restart_length)

        # Two-stage pipeline (RFIC samples are many sys cycles apart, so extra
        # latency is free and it keeps the multiply and the accumulate off the
        # same critical path):
        #   Stage 1 (on sample_stb): carrier wipe-off (multiply by conjugate)
        #     + register the per-tap replica / epoch / code phase.
        #   Stage 2 (next cycle):    replica multiply + integrate-and-dump.
        # The replica state (taps, epoch, code phase) is registered once and
        # reused by every antenna; only the wipe-off multiply is per antenna.
        prod_bits = sample_bits + carrier_amp_bits + 1

        s1_valid  = Signal()
        i_bb = [Signal((prod_bits, True)) for _ in range(num_ants)]
        q_bb = [Signal((prod_bits, True)) for _ in range(num_ants)]
        rep_r    = [Signal((replica_bits, True)) for _ in range(num_taps)]
        epoch_r  = Signal()
        cphase_r = Signal(code_frac_bits)
        cchip_r  = Signal(max=max_code_length)
        sidx_r   = Signal(64)
        self.sync += [
            s1_valid.eq(self.sample_stb),
            If(self.sample_stb,
                *[i_bb[n].eq(self.sample_i_ants[n] * carrier.cos
                             + self.sample_q_ants[n] * carrier.sin)
                  for n in range(num_ants)],
                *[q_bb[n].eq(self.sample_q_ants[n] * carrier.cos
                             - self.sample_i_ants[n] * carrier.sin)
                  for n in range(num_ants)],
                *[rep_r[t].eq(code.replica[t]) for t in range(num_taps)],
                epoch_r.eq(code.epoch), cphase_r.eq(code.code_frac),
                cchip_r.eq(code.chip_index),
                # Global index of *this* sample, carried alongside it into
                # stage 2 so the dump timestamps the sample it integrated.
                sidx_r.eq(self.sample_count),
            ),
        ]

        # Running accumulators + integrated-sample bookkeeping, one set of six
        # per antenna. No sample-index state here: restart must not rebase the
        # (global) timestamp.
        acc = [{k: Signal((accum_bits, True)) for k in acc_names}
               for _ in range(num_ants)]
        nsamp = Signal(32)

        # Saturating multiply-accumulate. Nominal GNSS operation stays far from
        # the rail (the sum is noise-dominated and grows as sqrt(N)), but a
        # strong in-band interferer, a badly set AD9361 gain or a high fs x long
        # integration can overrun accum_bits -- and a wrapped accumulator is
        # indistinguishable from a plausible correlator value once it reaches
        # the host. Clamp instead, and say so, so a bad dump is recognisable.
        acc_max =  (1 << (accum_bits - 1)) - 1
        acc_min = -(1 << (accum_bits - 1))
        # rep_r is replica_bits signed, so the product is prod_bits+replica_bits
        # wide; +1 more for the accumulate, which must not wrap before it is
        # clamped. An amplitude-bearing replica moves the rail in by its peak
        # value -- see docs/signal_configuration.md for the numbers.
        sum_bits = max(accum_bits, prod_bits + replica_bits) + 1

        def sat_mac(acc_sig, sign, bb):
            """acc + sign*bb clamped to accum_bits; returns (value, clamped).

            The range test is deliberately *not* written as `raw > acc_max` /
            `raw < acc_min`. Migen renders a negative bound as an **unsigned**
            Verilog literal (`-32'h80000000`), and Verilog evaluates a
            relational expression as unsigned whenever either operand is -- so
            `raw < -32'h80000000` reinterprets `raw` as unsigned and is *true
            for every positive partial sum*, clamping it to the negative rail.
            Migen's own simulator evaluates in Python ints and never sees it,
            which is how it reached silicon; docs/gateware_builds.md 5.6e has
            the measurement and the xsim proof.

            Instead: a two's-complement value fits in `accum_bits` exactly when
            every bit at or above the sign position equals the sign bit. That is
            an equality test on plain unsigned slices, so there is no signed
            literal for the lowering to get wrong, and it is the cheaper circuit
            besides -- a few LUTs against two 34-bit comparators.
            """
            raw = Signal((sum_bits, True))
            val = Signal((accum_bits, True))
            sat = Signal()
            top  = raw[accum_bits - 1:]      # sign bit and everything above it
            fits = Signal()
            self.comb += [
                raw.eq(acc_sig + sign * bb),
                fits.eq((top == 0) | (top == (1 << len(top)) - 1)),
                If(fits,
                    val.eq(raw),
                ).Elif(raw[sum_bits - 1],    # the sum's real sign bit
                    val.eq(acc_min), sat.eq(1),
                ).Else(
                    val.eq(acc_max), sat.eq(1),
                ),
            ]
            return val, sat

        # nxt[n][k] is antenna n's accumulator k after this sample. The clamp
        # bits are OR-ed across the whole array: saturation is reported per
        # channel, because a clamped antenna spoils the dump the host beamforms.
        short = tap_short_names(num_taps)
        nxt, sat_bits = [], []
        for n in range(num_ants):
            vals = {}
            for t, name in enumerate(short):
                for iq, bb in (("i", i_bb[n]), ("q", q_bb[n])):
                    k = iq + name
                    vals[k], sat = sat_mac(acc[n][k], rep_r[t], bb)
                    sat_bits.append(sat)
            nxt.append(vals)
        any_sat = Signal()
        self.comb += any_sat.eq(Cat(*sat_bits) != 0)

        # Per-integration saturation, latched into dump_saturated alongside the
        # accumulators it describes and cleared with them.
        sat_r = Signal()

        def store(dst):
            """Latch every antenna's post-MAC value into `dst` (the running
            accumulators, or the dump registers on the epoch sample)."""
            return [dst[n][k].eq(nxt[n][k])
                    for n in range(num_ants) for k in acc_names]

        def clear():
            return [acc[n][k].eq(0) for n in range(num_ants) for k in acc_names]

        self.sync += [
            self.dump_stb.eq(0),
            If(self.restart,
                *clear(),
                nsamp.eq(0), sat_r.eq(0), self.saturated.eq(0),
            ).Elif(s1_valid,
                *store(acc),
                nsamp.eq(nsamp + 1),
                If(any_sat, sat_r.eq(1), self.saturated.eq(1)),
                # Dump on the sample that completes a code period.
                If(epoch_r,
                    # Inhibited: the integration still restarts on the epoch, it
                    # just does not become a record.
                    self.dump_stb.eq(~inhibit),
                    *store(self.acc),
                    self.integrated_samples.eq(nsamp + 1),
                    self.sample_index.eq(sidx_r),
                    self.dump_code_phase.eq(cphase_r),
                    self.dump_code_chip.eq(cchip_r),
                    self.dump_code_length.eq(self.code_length),
                    self.dump_code_step.eq(self.code_step),
                    self.dump_num_taps.eq(self.taps_cfg),
                    self.dump_saturated.eq(sat_r | any_sat),
                    # Reset accumulators for the next integration.
                    *clear(),
                    nsamp.eq(0), sat_r.eq(0),
                ),
            ),
        ]
