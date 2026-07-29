#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Structural checks on the *generated* RTL, for bugs no simulation can see.

Every other test in this suite runs migen's Python simulator, which evaluates
the design in exact Python integer arithmetic and never touches the Verilog
emitter. Two of the bugs that stopped the on-FPGA correlators from correlating
lived exactly in that blind spot -- green in simulation, wrong on the board:

  * **Comparison against a negative constant.** The saturating accumulator used
    ``raw < -(2**31)``. LiteX's printer renders that as
    ``raw < -32'h80000000``; ``32'h80000000`` is an *unsigned* literal and
    Verilog's unary minus keeps it unsigned, so the whole comparison is
    evaluated unsigned and comes out true for every non-negative ``raw``. Each
    integration clamped to the negative rail the first time its running sum was
    non-negative, so every dump came back saturated and the correlators read a
    random walk reflected off -2**31.

  * **Negative ``Memory`` init values.** The carrier sin/cos ROMs were built
    from signed Python ints. A ``Memory`` init list is emitted as a
    ``$readmemh`` data file with plain ``"{:x}"`` formatting, so -127 lands in
    the file as ``-7F``. xsim rejects the token ("Illegal hex digit '-'") and
    Vivado synthesis silently drops the minus and stores +0x7F -- either way the
    ROM's negative half is wrong and the carrier replica comes out full-wave
    rectified, which is not a complex exponential, so carrier wipe-off fails.

Both are properties of the emitted text, so that is what these tests read. They
use LiteX's emitter (``litex.gen.fhdl.verilog``), not migen's, because that is
the one ``Builder`` ships to Vivado -- and the two differ precisely where it
mattered: migen's printer renders the same clamp as ``32'sd2147483648``, which
is correct, so converting with migen would have hidden the bug.
"""

import re
import unittest

from migen import *

from litex.gen.fhdl.verilog import convert as litex_convert

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.carrier_nco import CarrierNCO
from gnss_m2sdr.gateware.channel import TrackingChannel

# A sized literal with a leading minus on the right of a comparison, e.g.
# `x < -32'h80000000` or `x >= -8'd5`. Any match is an unsigned-comparison trap.
NEGATIVE_COMPARISON = re.compile(r"[<>]=?\s*-\s*\d+\s*'\s*[a-zA-Z]?[0-9a-fA-F_]+")

# Tokens $readmemh accepts in a data file: hex digits, x/z, and _ separators.
HEX_TOKEN = re.compile(r"\A[0-9a-fA-FxXzZ_]+\Z")


def emit(dut, ios=()):
    """LiteX-emit `dut` in a `sys` clock domain; return (verilog, data_files)."""
    class Top(Module):
        def __init__(self):
            self.clock_domains.cd_sys = ClockDomain("sys")
            self.submodules.dut = dut

    top = Top()
    out = litex_convert(top, ios={top.cd_sys.clk, top.cd_sys.rst, *ios})
    return str(out), dict(out.data_files)


class TestNoNegativeLiteralComparisons(unittest.TestCase):
    """No comparison in the shipped RTL may name a negative literal.

    Verilog makes a comparison unsigned as soon as *either* operand is
    unsigned, and `-<size>'h<value>` is unsigned. Range-check against the
    redundant sign bits instead (see `sat_mac` in channel.py).
    """

    def assert_clean(self, verilog, what):
        found = sorted(set(m.group(0).strip() for m in
                           NEGATIVE_COMPARISON.finditer(verilog)))
        self.assertEqual(found, [], msg=(
            f"{what}: comparison against a negative literal in the generated "
            f"Verilog: {found}. Verilog evaluates a mixed signed/unsigned "
            f"comparison unsigned, so this is true for the wrong operands on "
            f"hardware while migen's simulator still agrees with Python."))

    def test_channel(self):
        dut = TrackingChannel(prn=1, code_frac_bits=24, num_ants=1)
        verilog, _ = emit(dut, ios={dut.sample_stb})
        self.assert_clean(verilog, "TrackingChannel")

    def test_channel_two_antennas(self):
        dut = TrackingChannel(prn=1, code_frac_bits=24, num_ants=2)
        verilog, _ = emit(dut, ios={dut.sample_stb})
        self.assert_clean(verilog, "TrackingChannel(num_ants=2)")

    def test_bank(self):
        dut = GNSSTracking(n_channels=2, prns=[1, 2], code_frac_bits=24)
        verilog, _ = emit(dut, ios={dut.sample_stb})
        self.assert_clean(verilog, "GNSSTracking")


class TestClampBoundsSurviveEmission(unittest.TestCase):
    """The clamp must still be pinned to the accumulator rails after emission.

    A range test written on sign bits is easy to get subtly wrong (off by one
    bit gives a clamp at +/-2**30), so check the emitted constants are the rails
    the record format promises, and that both appear.
    """

    def test_rail_constants_present(self):
        dut = TrackingChannel(prn=1, code_frac_bits=24, num_ants=1)
        verilog, _ = emit(dut, ios={dut.sample_stb})
        # 32-bit accumulators: -2**31 as a bit pattern, +2**31-1 as itself.
        self.assertIn("32'h80000000", verilog,
                      "negative accumulator rail missing from the emitted clamp")
        self.assertIn("31'h7fffffff", verilog,
                      "positive accumulator rail missing from the emitted clamp")


class TestMemoryInitFilesAreValidHex(unittest.TestCase):
    """Every emitted `$readmemh` data file must be readable by `$readmemh`.

    A negative entry is not: xsim refuses it and Vivado stores its absolute
    value, so the ROM silently ends up holding something else.
    """

    def assert_files_ok(self, data_files, what):
        self.assertTrue(data_files, f"{what}: no memory init files were emitted")
        for name, content in data_files.items():
            for line_no, line in enumerate(content.splitlines(), start=1):
                token = line.strip()
                if not token:
                    continue
                self.assertRegex(token, HEX_TOKEN, msg=(
                    f"{what}: {name}:{line_no} is {token!r}, which $readmemh "
                    f"cannot parse. Mask Memory init values to unsigned "
                    f"(value & (2**width - 1)) and let the signed read side "
                    f"reinterpret the bits."))

    def test_carrier_nco_roms(self):
        dut = CarrierNCO(32, 8, 8)
        _, data_files = emit(dut, ios={dut.stb, dut.cos, dut.sin})
        self.assert_files_ok(data_files, "CarrierNCO")

    def test_bank_memories(self):
        dut = GNSSTracking(n_channels=2, prns=[1, 2], code_frac_bits=24)
        _, data_files = emit(dut, ios={dut.sample_stb})
        self.assert_files_ok(data_files, "GNSSTracking")


class TestCarrierRomHoldsTwosComplement(unittest.TestCase):
    """The masked ROM must still be the signed table the wipe-off needs.

    Masking is only correct if the read side reinterprets the pattern, so check
    the words are the two's-complement encodings of the intended amplitudes --
    the guard against "fixed the emitter, broke the maths".
    """

    def test_tables_are_twos_complement(self):
        import math

        from gnss_m2sdr.gateware.carrier_nco import _sincos_tables

        amp_bits, addr_bits = 8, 8
        sin_t, cos_t = _sincos_tables(addr_bits, amp_bits)
        peak = (1 << (amp_bits - 1)) - 1
        for i, (s, c) in enumerate(zip(sin_t, cos_t)):
            for word, fn, name in ((s, math.sin, "sin"), (c, math.cos, "cos")):
                self.assertIsInstance(word, int)
                self.assertTrue(0 <= word < (1 << amp_bits),
                                f"{name}[{i}] = {word} is not an unsigned "
                                f"{amp_bits}-bit word")
                signed = word - (1 << amp_bits) if word >> (amp_bits - 1) else word
                want = int(round(peak * fn(2 * math.pi * i / (1 << addr_bits))))
                self.assertEqual(signed, want,
                                 f"{name}[{i}] decodes to {signed}, want {want}")


if __name__ == "__main__":
    unittest.main()
