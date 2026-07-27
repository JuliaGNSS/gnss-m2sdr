#
# This file is part of gnss-m2sdr.
#
# Single GNSS tracking channel: carrier wipe-off + E/P/L correlation + I&D.
# SPDX-License-Identifier: BSD-2-Clause

"""One GPS L1 C/A tracking channel.

Per enabled input sample (I, Q):
  1. Carrier NCO -> cos/sin replica; complex mix (multiply by conjugate) to
     wipe off the residual carrier:  I_bb = I*cos + Q*sin,  Q_bb = Q*cos - I*sin.
  2. Code NCO -> Early/Prompt/Late code chips (+/-1).
  3. Accumulate code * baseband into six I&D accumulators (I/Q x E/P/L).

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
single time axis. `restart` therefore rebases only the code phase and the
integration accumulators, never the timestamp. `sample_index` is the 0-based
global index of the last sample included in the integration; the host maps it
to Tracking.jl's 1-based per-chunk convention with
`sample_index_julia = sample_index - chunk_origin + 1` (see record_format.py).
"""

from migen import *

from litex.gen import *

from gnss_m2sdr.gateware.carrier_nco import CarrierNCO
from gnss_m2sdr.gateware.code_replica import CodeReplica
from gnss_m2sdr.gateware.ca_code import CA_CODE_LENGTH


class TrackingChannel(LiteXModule):
    def __init__(self, prn=1, sample_bits=16, carrier_phase_bits=32,
                 carrier_lut_addr_bits=8, carrier_amp_bits=8,
                 code_frac_bits=24, accum_bits=32, code_length=CA_CODE_LENGTH):
        # Sample input. sample_count is the global free-running input-sample
        # counter (0-based index of the sample presented this strobe), owned by
        # GNSSTracking and shared by all channels.
        self.sample_i     = Signal((sample_bits, True))
        self.sample_q     = Signal((sample_bits, True))
        self.sample_stb   = Signal()
        self.sample_count = Signal(64)

        # Control (host / acquisition feedback).
        self.carrier_fw    = Signal(carrier_phase_bits)  # carrier phase increment / sample
        self.carrier_set   = Signal()                    # load carrier_phase_in
        self.carrier_phase_in = Signal(carrier_phase_bits)
        self.code_step     = Signal(code_frac_bits)      # code phase increment / sample
        self.spacing       = Signal(code_frac_bits)      # E/L half spacing (chips)
        self.restart       = Signal()                    # reset code phase + integration

        # Dump outputs (valid for one cycle when dump_stb high, then held).
        self.dump_stb           = Signal()
        self.ie = Signal((accum_bits, True))
        self.qe = Signal((accum_bits, True))
        self.ip = Signal((accum_bits, True))
        self.qp = Signal((accum_bits, True))
        self.il = Signal((accum_bits, True))
        self.ql = Signal((accum_bits, True))
        self.integrated_samples = Signal(32)
        self.sample_index       = Signal(64)
        self.dump_code_phase    = Signal(code_frac_bits)

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
        ]

        # Two-stage pipeline (RFIC samples are many sys cycles apart, so extra
        # latency is free and it keeps the multiply and the accumulate off the
        # same critical path):
        #   Stage 1 (on sample_stb): carrier wipe-off (multiply by conjugate)
        #     + register the code chips / epoch / code phase.
        #   Stage 2 (next cycle):    code sign multiply + integrate-and-dump.
        prod_bits = sample_bits + carrier_amp_bits + 1

        s1_valid  = Signal()
        i_bb = Signal((prod_bits, True)); q_bb = Signal((prod_bits, True))
        early_r  = Signal((2, True)); prompt_r = Signal((2, True)); late_r = Signal((2, True))
        epoch_r  = Signal()
        cphase_r = Signal(code_frac_bits)
        sidx_r   = Signal(64)
        self.sync += [
            s1_valid.eq(self.sample_stb),
            If(self.sample_stb,
                i_bb.eq(self.sample_i * carrier.cos + self.sample_q * carrier.sin),
                q_bb.eq(self.sample_q * carrier.cos - self.sample_i * carrier.sin),
                early_r.eq(code.early), prompt_r.eq(code.prompt), late_r.eq(code.late),
                epoch_r.eq(code.epoch), cphase_r.eq(code.code_frac),
                # Global index of *this* sample, carried alongside it into
                # stage 2 so the dump timestamps the sample it integrated.
                sidx_r.eq(self.sample_count),
            ),
        ]

        # Running accumulators + integrated-sample bookkeeping. No sample-index
        # state here: restart must not rebase the (global) timestamp.
        ie = Signal((accum_bits, True)); qe = Signal((accum_bits, True))
        ip = Signal((accum_bits, True)); qp = Signal((accum_bits, True))
        il = Signal((accum_bits, True)); ql = Signal((accum_bits, True))
        nsamp = Signal(32)

        self.sync += [
            self.dump_stb.eq(0),
            If(self.restart,
                ie.eq(0), qe.eq(0), ip.eq(0), qp.eq(0), il.eq(0), ql.eq(0),
                nsamp.eq(0),
            ).Elif(s1_valid,
                # code.*_r are +/-1; multiply-accumulate the registered baseband.
                ie.eq(ie + early_r  * i_bb),
                qe.eq(qe + early_r  * q_bb),
                ip.eq(ip + prompt_r * i_bb),
                qp.eq(qp + prompt_r * q_bb),
                il.eq(il + late_r   * i_bb),
                ql.eq(ql + late_r   * q_bb),
                nsamp.eq(nsamp + 1),
                # Dump on the sample that completes a code period.
                If(epoch_r,
                    self.dump_stb.eq(1),
                    self.ie.eq(ie + early_r  * i_bb),
                    self.qe.eq(qe + early_r  * q_bb),
                    self.ip.eq(ip + prompt_r * i_bb),
                    self.qp.eq(qp + prompt_r * q_bb),
                    self.il.eq(il + late_r   * i_bb),
                    self.ql.eq(ql + late_r   * q_bb),
                    self.integrated_samples.eq(nsamp + 1),
                    self.sample_index.eq(sidx_r),
                    self.dump_code_phase.eq(cphase_r),
                    # Reset accumulators for the next integration.
                    ie.eq(0), qe.eq(0), ip.eq(0), qp.eq(0), il.eq(0), ql.eq(0),
                    nsamp.eq(0),
                ),
            ),
        ]
