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
dump registers, the integrated-sample count and a free-running sample_index are
captured, and the accumulators reset. The dump maps onto Tracking.jl's
CorrelatorOutput(correlator=[E,P,L], integrated_samples, sample_index; code_phase).
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
        # Sample input.
        self.sample_i   = Signal((sample_bits, True))
        self.sample_q   = Signal((sample_bits, True))
        self.sample_stb = Signal()

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

        # Carrier wipe-off (multiply by conjugate of the replica).
        prod_bits = sample_bits + carrier_amp_bits + 1
        i_bb = Signal((prod_bits, True))
        q_bb = Signal((prod_bits, True))
        self.comb += [
            i_bb.eq(self.sample_i * carrier.cos + self.sample_q * carrier.sin),
            q_bb.eq(self.sample_q * carrier.cos - self.sample_i * carrier.sin),
        ]

        # Running accumulators + sample/index bookkeeping.
        ie = Signal((accum_bits, True)); qe = Signal((accum_bits, True))
        ip = Signal((accum_bits, True)); qp = Signal((accum_bits, True))
        il = Signal((accum_bits, True)); ql = Signal((accum_bits, True))
        nsamp = Signal(32)
        sidx  = Signal(64)

        self.sync += [
            self.dump_stb.eq(0),
            If(self.restart,
                ie.eq(0), qe.eq(0), ip.eq(0), qp.eq(0), il.eq(0), ql.eq(0),
                nsamp.eq(0), sidx.eq(0),
            ).Elif(self.sample_stb,
                # code.early/prompt/late are +/-1; multiply-accumulate.
                ie.eq(ie + code.early  * i_bb),
                qe.eq(qe + code.early  * q_bb),
                ip.eq(ip + code.prompt * i_bb),
                qp.eq(qp + code.prompt * q_bb),
                il.eq(il + code.late   * i_bb),
                ql.eq(ql + code.late   * q_bb),
                nsamp.eq(nsamp + 1),
                sidx.eq(sidx + 1),
                # Dump on the sample that completes a code period.
                If(code.epoch,
                    self.dump_stb.eq(1),
                    self.ie.eq(ie + code.early  * i_bb),
                    self.qe.eq(qe + code.early  * q_bb),
                    self.ip.eq(ip + code.prompt * i_bb),
                    self.qp.eq(qp + code.prompt * q_bb),
                    self.il.eq(il + code.late   * i_bb),
                    self.ql.eq(ql + code.late   * q_bb),
                    self.integrated_samples.eq(nsamp + 1),
                    self.sample_index.eq(sidx),
                    self.dump_code_phase.eq(code.code_frac),
                    # Reset accumulators for the next integration.
                    ie.eq(0), qe.eq(0), ip.eq(0), qp.eq(0), il.eq(0), ql.eq(0),
                    nsamp.eq(0),
                ),
            ),
        ]
