#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Faults that Migen's simulator cannot see, checked on the emitted Verilog.

Every other test in this suite runs the Migen simulator, which evaluates
expressions with Python integers of unbounded range and exact signedness. That
is the right model of what the design *means*, and it is blind by construction
to what the Verilog backend *writes* -- so a lowering that changes the meaning
passes the whole suite and fails only on silicon.

One did. A five-tap build reached the board with every correlator accumulator
railed at -2^31 and the sums independent of the code, because the saturating
accumulator's lower bound was emitted as

    if ((raw < -32'h80000000))

`32'h80000000` is an **unsigned** literal and unary minus keeps it unsigned, and
Verilog evaluates a relational expression as unsigned whenever either operand
is. So `raw` -- a signed sum -- was reinterpreted as unsigned, every positive
partial sum compared "less than" the negative rail, and the accumulator was
clamped to -2^31 on the first sample of every integration. Proven in Vivado's
xsim: for `raw = 40000000`, the emitted expression is true and a signed
comparison is false. Two weeks of simulation could not have found it; reading
the generated Verilog found it in minutes.

See docs/gateware_builds.md 5.6e.
"""

import re
import unittest

from migen import *
from migen.fhdl.verilog import convert

from gnss_m2sdr.gateware.bank import GNSSTracking
from gnss_m2sdr.gateware.channel import TrackingChannel
from gnss_m2sdr.gateware.code_replica import CodeReplica

# How Migen renders a constant bound is version-dependent, and that is the
# whole trap: the Migen that built the flashed bitstream emitted
# `-32'h80000000` (unsigned, wrong), while the Migen pinned for this suite
# emits `32'sd2147483648` (signed, right). A test that greps for one spelling
# passes on the other and proves nothing.
#
# So do not test the spelling. Test that the saturating accumulator contains no
# magnitude comparison at all: its range check is an equality on slices, which
# has the same meaning under every Migen and in Verilog. Any `<` or `>` applied
# to the accumulate sum is the shape that can be mis-lowered, whatever it
# renders as today.
RELATIONAL = re.compile(r"[<>]", re.ASCII)

# A relational operator against a bare negative literal -- `< -32'h80000000`.
# This is what LiteX's converter emits and what Verilog reads as unsigned.
NEG_LITERAL_IN_COMPARE = re.compile(r"[<>]=?\s*-\s*\d+'", re.ASCII)


def offending_lines(verilog):
    """Lines where the saturating accumulator's sum meets a relational operator.

    `raw` is the pre-clamp sum inside `sat_mac`; Migen names it `*_raw<n>`.
    Bit-slices (`raw0[33:31]`) and equality are fine, magnitude tests are not.
    """
    out = []
    for n, line in enumerate(verilog.splitlines(), 1):
        if "_raw" not in line and not re.search(r"\braw\d", line):
            continue
        # Strip bit-slices so `[33:31]` cannot look like anything, and drop
        # `<=` used as a non-blocking assignment.
        probe = re.sub(r"\[[^\]]*\]", "", line).replace("<=", " ")
        if RELATIONAL.search(probe):
            out.append((n, line.strip()))
    return out


class TestTheClampUsesNoMagnitudeComparison(unittest.TestCase):
    """The accumulate sum must never meet a `<` or `>`.

    Such a comparison is unsigned in Verilog and signed in Migen's simulator,
    so it means two different things in the two places the design has to be
    correct. Express the bound as a signal, or -- better, and what the
    saturating accumulator does -- test the sign and range bits instead.
    """

    def _check(self, dut, label, **kwargs):
        verilog = str(convert(dut, **kwargs))
        bad = offending_lines(verilog)
        self.assertEqual(
            bad, [],
            f"{label}: the saturating accumulator's sum is used in "
            f"{len(bad)} magnitude comparison(s). A relational operator against "
            f"a constant bound is exactly what Migen lowered to an UNSIGNED "
            f"Verilog compare on the flashed build; use an equality test on the "
            f"sign and range bits instead:\n"
            + "\n".join(f"  line {n}: {l}" for n, l in bad[:8]))

    def test_the_tracking_channel_lowers_without_one(self):
        # The five-tap, sub-chip configuration the board was flashed with.
        self._check(TrackingChannel(max_code_length=64, num_taps=5,
                                    max_subchips=12),
                    "TrackingChannel(taps=5, subchips=12)")

    def test_the_three_tap_channel_lowers_without_one(self):
        self._check(TrackingChannel(max_code_length=64), "TrackingChannel()")

    def test_the_two_antenna_channel_lowers_without_one(self):
        self._check(TrackingChannel(max_code_length=64, num_ants=2, num_taps=5,
                                    max_subchips=12),
                    "TrackingChannel(num_ants=2)")

    def test_the_code_replica_lowers_without_one(self):
        self._check(CodeReplica(max_code_length=64, num_taps=5, max_subchips=12),
                    "CodeReplica")

    def test_the_whole_bank_lowers_without_one(self):
        self._check(GNSSTracking(n_channels=2, prns=[1, 2], max_code_length=64,
                                 num_taps=5, max_subchips=12),
                    "GNSSTracking")


class TestTheLiteXConverterEmitsNoUnsignedNegativeLiteral(unittest.TestCase):
    """The build's own converter, checked directly.

    This is the exact path that produced the flashed bitstream, and the exact
    reason the rest of the suite was blind: **the tests convert with
    `migen.fhdl.verilog`, the build converts with `litex.gen.fhdl.verilog`, and
    the two render a negative constant differently.**

        migen  ->  32'sd2147483648     signed, two's complement, correct
        litex  ->  -32'h80000000       unsigned: no `'s` marker at all

    `litex/gen/fhdl/expression.py::_generate_constant` formats a negative value
    as `"-" + nbits + "'" + hex(abs(value))` and never writes the signedness
    marker, so Verilog reads the literal as unsigned and evaluates the whole
    relational expression as unsigned. Running the old code through this
    converter reproduces the flashed build's line byte for byte:

        if ((trackingchannel_raw0 < -32'h80000000)) begin

    Ten of them, one per tap per I/Q.
    """

    def _litex_verilog(self, dut):
        from litex.gen.fhdl.verilog import convert as litex_convert

        class Wrap(Module):
            def __init__(self, inner):
                self.clock_domains.cd_sys = ClockDomain("sys")
                self.submodules.inner = inner

        w = Wrap(dut)
        return str(litex_convert(w, ios={w.cd_sys.clk, w.cd_sys.rst}))

    def _check(self, dut, label):
        try:
            verilog = self._litex_verilog(dut)
        except ImportError:                      # pragma: no cover
            self.skipTest("litex is not installed")
        bad = [(n, l.strip())
               for n, l in enumerate(verilog.splitlines(), 1)
               if NEG_LITERAL_IN_COMPARE.search(l)]
        self.assertEqual(
            bad, [],
            f"{label}: {len(bad)} comparison(s) against a bare negative literal "
            f"in the Verilog the BUILD generates. Verilog evaluates these as "
            f"unsigned, so a signed operand is reinterpreted and the test means "
            f"something different on silicon than in simulation:\n"
            + "\n".join(f"  line {n}: {l}" for n, l in bad[:8]))

    def test_the_tracking_channel(self):
        self._check(TrackingChannel(max_code_length=64, num_taps=5,
                                    max_subchips=12),
                    "TrackingChannel(taps=5, subchips=12)")

    def test_the_two_antenna_channel(self):
        self._check(TrackingChannel(max_code_length=64, num_ants=2,
                                    num_taps=5, max_subchips=12),
                    "TrackingChannel(num_ants=2)")

    def test_the_code_replica(self):
        self._check(CodeReplica(max_code_length=64, num_taps=5,
                                max_subchips=12), "CodeReplica")


class TestTheSaturatingAccumulatorClamp(unittest.TestCase):
    """The clamp itself, in Migen's simulator.

    The lowering test above is what would have caught the bug; this is what
    keeps the *logic* right if someone rewrites the range test back into a pair
    of magnitude comparisons. The cases are the ones that matter: a small
    positive sum must pass through untouched (that is the one the broken
    lowering clamped to the negative rail), and both overflows must clamp to
    their own end.
    """

    ACCUM = 32

    class _Mac(Module):
        def __init__(self, accum_bits=32, prod_bits=25, replica_bits=8):
            self.acc = Signal((accum_bits, True))
            self.rep = Signal((replica_bits, True))
            self.bb  = Signal((prod_bits, True))
            self.val = Signal((accum_bits, True))
            self.sat = Signal()
            acc_max, acc_min = (1 << (accum_bits - 1)) - 1, -(1 << (accum_bits - 1))
            sum_bits = max(accum_bits, prod_bits + replica_bits) + 1
            raw = Signal((sum_bits, True))
            top, fits = raw[accum_bits - 1:], Signal()
            self.comb += [
                raw.eq(self.acc + self.rep * self.bb),
                fits.eq((top == 0) | (top == (1 << len(top)) - 1)),
                If(fits, self.val.eq(raw))
                .Elif(raw[sum_bits - 1], self.val.eq(acc_min), self.sat.eq(1))
                .Else(self.val.eq(acc_max), self.sat.eq(1)),
            ]

    def _mac(self, acc, rep, bb):
        dut = self._Mac()
        got = {}

        def bench():
            yield dut.acc.eq(acc); yield dut.rep.eq(rep); yield dut.bb.eq(bb)
            yield
            got["val"] = (yield dut.val); got["sat"] = (yield dut.sat)

        from migen.sim import run_simulation
        run_simulation(dut, bench())
        v = got["val"]
        return (v - (1 << self.ACCUM) if v >= (1 << (self.ACCUM - 1)) else v,
                got["sat"])

    def test_a_small_positive_sum_passes_through(self):
        # The broken lowering clamped exactly this to -2**31.
        self.assertEqual(self._mac(0, 1, 1_000_000), (1_000_000, 0))

    def test_a_small_negative_sum_passes_through(self):
        self.assertEqual(self._mac(0, -1, 1_000_000), (-1_000_000, 0))

    def test_positive_overflow_clamps_to_the_positive_rail(self):
        self.assertEqual(self._mac((1 << 31) - 1, 1, 1000),
                         ((1 << 31) - 1, 1))

    def test_negative_overflow_clamps_to_the_negative_rail(self):
        self.assertEqual(self._mac(-(1 << 31), -1, 1000), (-(1 << 31), 1))

    def test_the_largest_in_range_product_does_not_clamp(self):
        self.assertEqual(self._mac(0, 127, (1 << 24) - 1),
                         (127 * ((1 << 24) - 1), 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
