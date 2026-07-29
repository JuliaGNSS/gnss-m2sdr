#!/usr/bin/env python3
"""Emit TrackingChannel with LiteX's converter and stable port names.

LiteX's printer is the one `Builder` ships to Vivado, and it differs from
migen's exactly where the accumulator-clamp bug lived, so the injection test has
to run *this* rendering.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from migen import *
from litex.gen.fhdl.verilog import convert as litex_convert

from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.record_format import ACC_SIGNALS

FRAC = 24
ACC = list(ACC_SIGNALS)


class Wrapper(Module):
    def __init__(self, prn=1):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.i_sample_i = Signal((16, True))
        self.i_sample_q = Signal((16, True))
        self.i_sample_stb = Signal()
        self.i_sample_count = Signal(64)
        self.i_carrier_fw = Signal(32)
        self.i_carrier_set = Signal()
        self.i_carrier_phase_in = Signal(32)
        self.i_code_step = Signal(FRAC)
        self.i_spacing = Signal(FRAC)
        self.i_restart = Signal()
        self.i_code_phase_chip = Signal(10)
        self.i_code_phase_frac = Signal(FRAC)
        self.i_load_we = Signal()
        self.i_load_adr = Signal(10)
        self.i_load_dat = Signal()

        self.o_dump_stb = Signal()
        self.o_dump_saturated = Signal()
        self.o_saturated = Signal()
        self.o_integrated_samples = Signal(32)
        self.o_sample_index = Signal(64)
        self.o_dump_code_phase = Signal(FRAC)
        self.o_acc = {k: Signal((32, True), name="o_acc_" + k) for k in ACC}

        self.submodules.ch = ch = TrackingChannel(prn=prn, code_frac_bits=FRAC,
                                                  num_ants=1)
        self.comb += [
            ch.sample_i.eq(self.i_sample_i),
            ch.sample_q.eq(self.i_sample_q),
            ch.sample_stb.eq(self.i_sample_stb),
            ch.sample_count.eq(self.i_sample_count),
            ch.carrier_fw.eq(self.i_carrier_fw),
            ch.carrier_set.eq(self.i_carrier_set),
            ch.carrier_phase_in.eq(self.i_carrier_phase_in),
            ch.code_step.eq(self.i_code_step),
            ch.spacing.eq(self.i_spacing),
            ch.restart.eq(self.i_restart),
            ch.code_phase_chip.eq(self.i_code_phase_chip),
            ch.code_phase_frac.eq(self.i_code_phase_frac),
            # Runtime code loader, exactly as ChannelWithCSR drives it.
            ch.code.load_we.eq(self.i_load_we),
            ch.code.load_adr.eq(self.i_load_adr),
            ch.code.load_dat.eq(self.i_load_dat),
            self.o_dump_stb.eq(ch.dump_stb),
            self.o_dump_saturated.eq(ch.dump_saturated),
            self.o_saturated.eq(ch.saturated),
            self.o_integrated_samples.eq(ch.integrated_samples),
            self.o_sample_index.eq(ch.sample_index),
            self.o_dump_code_phase.eq(ch.dump_code_phase),
            *[self.o_acc[k].eq(ch.acc[0][k]) for k in ACC],
        ]


dut = Wrapper()
ios = {getattr(dut, n) for n in dir(dut)
       if n.startswith(("i_", "o_")) and isinstance(getattr(dut, n), Signal)}
ios |= set(dut.o_acc.values())
ios |= {dut.cd_sys.clk, dut.cd_sys.rst}
out = litex_convert(dut, ios=ios, name="ch_wrap")
out.write(sys.argv[1])
print("ACC order:", ACC)
