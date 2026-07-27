#
# This file is part of gnss-m2sdr.
#
# Single GNSS tracking channel: carrier wipe-off + E/P/L correlation + I&D.
# SPDX-License-Identifier: BSD-2-Clause

"""One GPS L1 C/A tracking channel, over `num_ants` coherent antennas.

Per enabled input sample (I, Q) of every antenna:
  1. Carrier NCO -> cos/sin replica; complex mix (multiply by conjugate) to
     wipe off the residual carrier:  I_bb = I*cos + Q*sin,  Q_bb = Q*cos - I*sin.
  2. Code NCO -> Early/Prompt/Late code chips (+/-1).
  3. Accumulate code * baseband into six I&D accumulators (I/Q x E/P/L).

The accumulators saturate at `accum_bits` rather than wrapping: a wrapped sum
still looks like a valid correlator value to the host, so overflow would be
silent. `dump_saturated` marks the dump whose integration clamped, and
`saturated` is the sticky per-channel version (cleared by `restart`) that the
bank exposes over CSR. Both are per channel, not per antenna: a clamp anywhere
in the array invalidates the dump the host would beamform.

Steps 1 and 2 need the carrier NCO, the code NCO and the three code replicas --
by far the expensive part -- and all antennas of a coherent array track the
*same* signal (shared LO, only the spatial phase differs), so those are
instantiated once and shared; only the six accumulators and their multipliers
are per antenna. `num_ants` therefore scales cheaply, which is why the
accumulators must stay separate: GNSSReceiver.jl#107 beamforms
post-correlation on the CPU from the per-antenna prompt covariance, so summing
antennas here would destroy the spatial information (and one NCOUpdate per
channel, not per antenna, is what #107 specifies). Antenna 0's ports keep their
scalar names, so single-antenna wiring is unchanged.

On each code epoch (one 1023-chip period) the accumulators are latched to the
dump registers, the integrated-sample count and the sample-counter value are
captured, and the accumulators reset. The dump maps onto Tracking.jl's
CorrelatorOutput(EarlyPromptLateCorrelator(SVector(late, prompt, early), spacing),
integrated_samples, sample_index) -- note that Tracking.jl orders its
accumulators latest-first, so E and L go in reversed relative to the names used
here; see record_format.py for why getting that backwards inverts the DLL. The
dumped `code_phase` is extra device-side metadata: Tracking.jl's CorrelatorOutput
has no code_phase field or keyword, so the host carries it out of band.

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
from gnss_m2sdr.record_format import ACC_SIGNALS, N_ANTS_MAX


class TrackingChannel(LiteXModule):
    def __init__(self, prn=1, sample_bits=16, carrier_phase_bits=32,
                 carrier_lut_addr_bits=8, carrier_amp_bits=8,
                 code_frac_bits=24, accum_bits=32, code_length=CA_CODE_LENGTH,
                 num_ants=1):
        assert 1 <= num_ants <= N_ANTS_MAX, f"1..{N_ANTS_MAX} antennas"

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
        self.code_step     = Signal(code_frac_bits)      # code phase increment / sample
        self.spacing       = Signal(code_frac_bits)      # E/L half spacing (chips)
        self.restart       = Signal()                    # rebase code phase + integration
        # Code phase loaded by `restart` (0/0 = start of the code).
        self.code_phase_chip = Signal(max=code_length)
        self.code_phase_frac = Signal(code_frac_bits)

        # Dump outputs (valid for one cycle when dump_stb high, then held).
        # acc[n] holds antenna n's six accumulators; antenna 0's are also
        # exposed under the original scalar names.
        self.dump_stb           = Signal()
        self.dump_saturated     = Signal()  # the dumped integration clamped
        self.acc = [{k: Signal((accum_bits, True)) for k in ACC_SIGNALS}
                    for _ in range(num_ants)]
        for k, sig in self.acc[0].items():
            setattr(self, k, sig)
        self.integrated_samples = Signal(32)
        self.sample_index       = Signal(64)
        self.dump_code_phase    = Signal(code_frac_bits)

        # Sticky "an accumulator hit the rail since the last restart" status,
        # for the bank's saturation CSR. Unlike dump_saturated this survives
        # across dumps, so a host polling at its own pace cannot miss it.
        self.saturated = Signal()

        # # #

        # Sub-modules.
        self.carrier = carrier = CarrierNCO(carrier_phase_bits, carrier_lut_addr_bits, carrier_amp_bits)
        self.code    = code    = CodeReplica(prn=prn, frac_bits=code_frac_bits, code_length=code_length)
        self.comb += [
            carrier.freq_word.eq(self.carrier_fw),
            carrier.stb.eq(self.sample_stb),
            carrier.set_phase.eq(self.carrier_set),
            carrier.phase_in.eq(self.carrier_phase_in),
            code.code_step.eq(self.code_step),
            code.spacing.eq(self.spacing),
            code.stb.eq(self.sample_stb),
            code.restart.eq(self.restart),
            code.restart_chip.eq(self.code_phase_chip),
            code.restart_frac.eq(self.code_phase_frac),
        ]

        # Two-stage pipeline (RFIC samples are many sys cycles apart, so extra
        # latency is free and it keeps the multiply and the accumulate off the
        # same critical path):
        #   Stage 1 (on sample_stb): carrier wipe-off (multiply by conjugate)
        #     + register the code chips / epoch / code phase.
        #   Stage 2 (next cycle):    code sign multiply + integrate-and-dump.
        # The replica state (chips, epoch, code phase) is registered once and
        # reused by every antenna; only the wipe-off multiply is per antenna.
        prod_bits = sample_bits + carrier_amp_bits + 1

        s1_valid  = Signal()
        i_bb = [Signal((prod_bits, True)) for _ in range(num_ants)]
        q_bb = [Signal((prod_bits, True)) for _ in range(num_ants)]
        early_r  = Signal((2, True)); prompt_r = Signal((2, True)); late_r = Signal((2, True))
        epoch_r  = Signal()
        cphase_r = Signal(code_frac_bits)
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
                early_r.eq(code.early), prompt_r.eq(code.prompt), late_r.eq(code.late),
                epoch_r.eq(code.epoch), cphase_r.eq(code.code_frac),
                # Global index of *this* sample, carried alongside it into
                # stage 2 so the dump timestamps the sample it integrated.
                sidx_r.eq(self.sample_count),
            ),
        ]

        # Running accumulators + integrated-sample bookkeeping, one set of six
        # per antenna. No sample-index state here: restart must not rebase the
        # (global) timestamp.
        acc = [{k: Signal((accum_bits, True)) for k in ACC_SIGNALS}
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
        # code.*_r are +/-1 (2 bits signed), so the product is prod_bits+2 wide;
        # +1 more for the accumulate, which must not wrap before it is clamped.
        sum_bits = max(accum_bits, prod_bits + 2) + 1

        def sat_mac(acc_sig, sign, bb):
            """acc + sign*bb clamped to accum_bits; returns (value, clamped)."""
            raw = Signal((sum_bits, True))
            val = Signal((accum_bits, True))
            sat = Signal()
            self.comb += [
                raw.eq(acc_sig + sign * bb),
                If(raw > acc_max,
                    val.eq(acc_max), sat.eq(1),
                ).Elif(raw < acc_min,
                    val.eq(acc_min), sat.eq(1),
                ).Else(
                    val.eq(raw),
                ),
            ]
            return val, sat

        # nxt[n][k] is antenna n's accumulator k after this sample. The clamp
        # bits are OR-ed across the whole array: saturation is reported per
        # channel, because a clamped antenna spoils the dump the host beamforms.
        nxt, sat_bits = [], []
        for n in range(num_ants):
            vals = {}
            for k, sign, bb in (("ie", early_r,  i_bb[n]), ("qe", early_r,  q_bb[n]),
                                ("ip", prompt_r, i_bb[n]), ("qp", prompt_r, q_bb[n]),
                                ("il", late_r,   i_bb[n]), ("ql", late_r,   q_bb[n])):
                vals[k], s = sat_mac(acc[n][k], sign, bb)
                sat_bits.append(s)
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
                    for n in range(num_ants) for k in ACC_SIGNALS]

        def clear():
            return [acc[n][k].eq(0) for n in range(num_ants) for k in ACC_SIGNALS]

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
                    self.dump_stb.eq(1),
                    *store(self.acc),
                    self.integrated_samples.eq(nsamp + 1),
                    self.sample_index.eq(sidx_r),
                    self.dump_code_phase.eq(cphase_r),
                    self.dump_saturated.eq(sat_r | any_sat),
                    # Reset accumulators for the next integration.
                    *clear(),
                    nsamp.eq(0), sat_r.eq(0),
                ),
            ),
        ]
