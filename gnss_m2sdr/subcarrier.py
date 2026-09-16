#
# This file is part of gnss-m2sdr.
#
# Sub-chip replica shapes (BOC / CBOC / TMBOC) as the integer LUTs the gateware
# holds, plus the amplitude bookkeeping that goes with them.
# SPDX-License-Identifier: BSD-2-Clause

"""Sub-chip modulation: what the gateware's subcarrier LUT has to contain.

The code RAM yields one chip sign per chip. Everything a BOC-family signal adds
happens *inside* a chip, and it is periodic in the chip: GNSSSignals models it
as ``get_subcarrier_code(modulation, phase)``, a function of the code phase
alone (for TMBOC, of the phase and the chip's position in a 33-chip pattern).
So the replica factorises exactly:

    replica(phase) = primary_chip(floor(phase)) * subcarrier(phase)

and the gateware evaluates the second factor from a small table indexed by
``floor(frac * P)``, where ``frac`` is the fractional chip phase and ``P`` is
the modulation's *sub-chip factor*. That index is exact -- not a quantisation
of the transition points -- for every modulation below, which is the whole
reason the table is indexed this way instead of by the top bits of ``frac``:

===========================  ===  ===========================================
Modulation                     P  sub-chip k carries
===========================  ===  ===========================================
``LOC()``                      1  +1
``BOCsin(m, 1)``             2*m  ``+1`` if ``k`` even else ``-1``
``BOCcos(m, 1)``             4*m  ``+1`` if ``(k + m) // 2`` even else ``-1``
``CBOC(b1, b2, p, s)``  2*lcm(m)  ``a1*b1(k) + s*a2*b2(k)`` -- multi-level
``TMBOC(b1, b2, pattern)``  2*m2  ``b1(k)`` or ``b2(k)``, per chip position
===========================  ===  ===========================================

Why these are exact, taking BOCcos as the hard case: GNSSSignals evaluates it as
``BOCsin(m,1)`` at ``phase + 1/4``. For a phase in sub-chip ``k`` of ``P = 4m``,
``(f + 1/4) * 2m`` spans ``[(k+m)/2, (k+m+1)/2)``, an interval of length 1/2
whose floor is ``(k + m) // 2`` throughout. So the sign is constant across the
sub-chip and depends only on ``k`` -- there is no phase inside a sub-chip at
which the table and GNSSSignals disagree. The same argument at ``P = 12`` makes
``BOCsin(1,1)`` and ``BOCsin(6,1)`` (hence CBOC(1,6) and TMBOC(1,6)) constant
per sub-chip, which is why one 12-entry table serves the whole L1 BOC family.

Amplitude
---------
``LOC`` and the BOC/TMBOC subcarriers are +/-1, so their tables are +/-1 and the
device reproduces GNSSSignals' modelled code exactly.

CBOC is **not** +/-1: it is ``sqrt(p)*b1 +/- sqrt(1-p)*b2``, four levels. The
amplitude is the signal, not a detail -- GNSSReceiver's contract divides the
device's code amplitude out of every accumulator before the C/N0 estimator sees
it, so a sign-only substitute reads about 26 dB away from the same satellite
tracked in software. GNSSSignals' own resampled table carries the same
irrational amplitudes as the *integer* pair ``(a1, a2) = (19, 6)`` (levels
+/-25 and +/-13), whose RMS is ``sqrt((25**2 + 13**2)/2) = sqrt(397) =
19.9249`` -- exactly what ``get_code_amplitude(GalileoE1B)`` reports. So
programming that pair makes the gateware's replica *the modelled code*, and the
host's default ``replica_code_amplitude`` (which says "the device reproduces the
modelled code") is already right. Program another scale if you like -- the LUT
is yours -- but then report its RMS (``lut_rms``) to the host, or every C/N0 on
that channel is out by the ratio.

A sign-only BOC(1,1) stand-in for E1B/E1C is a *different signal*, not a cheaper
CBOC, and GNSSSignals says so by giving it its own type
(``GalileoE1B_BOC11`` / ``GalileoE1C_BOC11``, ~0.45 dB of correlation loss).
This module exposes it the same way: as its own entry in `SIGNAL_REPLICAS`,
never as a substitution applied behind the caller's back.

What is implemented, and what is not
------------------------------------
Everything here is the replica GNSSSignals models for the L1 signals in scope:
``GPSL1CA`` (LOC), ``GalileoE1B``/``GalileoE1C`` (CBOC(6,1,1/11), in phase and
anti-phase), their ``_BOC11`` approximations, ``GPSL1C_D``, ``BeiDouB1C_D`` and
``BeiDouB1C_P`` (BOCsin(1,1)) and ``GPSL1C_P`` (TMBOC(6,1,4/33)).

BeiDou B1C's pilot is **QMBOC(6,1,4/33)** in the ICD; GNSSSignals models it as a
pure BOCsin(1,1) because the pilot's minor BOC(6,1) arm is in phase quadrature
and no single real replica can capture it. This module replicates what
GNSSSignals models -- BOC(1,1) -- and does not claim QMBOC. Likewise, nothing
here generates the secondary/overlay code: the host removes it from the dumps
(GNSSReceiver.jl#132), so the gateware replicates the primary code only.
"""

from math import gcd, sqrt


def _lcm(a, b):
    return a * b // gcd(a, b)


# Sub-chip factor P of each modulation, i.e. how many equal slices of a chip the
# subcarrier is constant over. This is what `max_subchips` in the build has to
# cover, and what the host checks the capability CSR against.
def subchip_factor(kind, m=1, m2=None):
    """Sub-chips per chip for `kind`, matching GNSSSignals' own factors."""
    if kind == "LOC":
        return 1
    if kind == "BOCsin":
        return 2 * m
    if kind == "BOCcos":
        return 4 * m
    if kind == "CBOC":
        return 2 * _lcm(m, m2)
    if kind == "TMBOC":
        return 2 * m2
    raise ValueError(f"unknown modulation {kind!r}")


def _boc_sin(m, k, subchips):
    """BOCsin(m,1) at sub-chip k of `subchips`: iseven(floor(phase*2m))."""
    return 1 if ((k * 2 * m) // subchips) % 2 == 0 else -1


def _boc_cos(m, k, subchips):
    """BOCcos(m,1) at sub-chip k of `subchips` = 4m: iseven((k + m) // 2)."""
    assert subchips == 4 * m, "BOCcos needs the quarter-sub-chip grid P = 4m"
    return 1 if ((k + m) // 2) % 2 == 0 else -1


def loc_lut():
    """LOC (plain BPSK): a one-entry unit table."""
    return [1]


def boc_sin_lut(m=1, subchips=None):
    """Sine-phased BOC(m,1) over `subchips` (default 2m) sub-chips."""
    subchips = subchips or subchip_factor("BOCsin", m)
    return [_boc_sin(m, k, subchips) for k in range(subchips)]


def boc_cos_lut(m=1, subchips=None):
    """Cosine-phased BOC(m,1) over `subchips` (default 4m) sub-chips."""
    subchips = subchips or subchip_factor("BOCcos", m)
    return [_boc_cos(m, k, subchips) for k in range(subchips)]


# GNSSSignals' integer approximation of CBOC(6,1,1/11)'s sqrt-power amplitudes:
# a1/a2 = 19/6 ~ sqrt(10) and RMS sqrt(397), which is what get_code_amplitude
# reports for Galileo E1B/E1C. Keep them unless you also tell the host the RMS.
CBOC_A1 = 19
CBOC_A2 = 6


def cboc_lut(m1=1, m2=6, a1=CBOC_A1, a2=CBOC_A2, boc2_sign=1, subchips=None):
    """CBOC(m1,m2) with integer amplitudes: ``a1*BOC(m1,1) + s*a2*BOC(m2,1)``.

    `boc2_sign` is the ICD's CBOC(+)/CBOC(-) selector: +1 for Galileo E1B, -1
    for E1C (Galileo OS SIS ICD 2.3.3), exactly as GNSSSignals' `CBOC` carries
    it. The four levels are +/-(a1+a2) and +/-(a1-a2).
    """
    subchips = subchips or subchip_factor("CBOC", m1, m2)
    return [a1 * _boc_sin(m1, k, subchips) + boc2_sign * a2 * _boc_sin(m2, k, subchips)
            for k in range(subchips)]


def tmboc_luts(m1=1, m2=6, subchips=None):
    """The two tables TMBOC switches between, on one common sub-chip grid.

    Returns ``(majority, minority)``: BOC(m1,1) for the chip positions the
    pattern leaves clear and BOC(m2,1) for the ones it sets. Both are evaluated
    on the *minority* component's grid (P = 2*m2), which is the finer of the two,
    so one index serves both.
    """
    subchips = subchips or subchip_factor("TMBOC", m1, m2)
    return (boc_sin_lut(m1, subchips), boc_sin_lut(m2, subchips))


# GPS L1C-P's TMBOC(6,1,4/33): BOC(6,1) at chip positions 0, 4, 6 and 29 of
# every 33 (IS-GPS-800 3.3). GNSSSignals carries the same tuple.
TMBOC_L1CP_PATTERN = tuple(i in (0, 4, 6, 29) for i in range(33))


def tmboc_select_bits(code_length, pattern=TMBOC_L1CP_PATTERN, phase=0):
    """Per-chip "use the minority subcarrier" bits, one per chip of the code.

    The gateware keeps this bit *next to the chip in the code RAM* rather than
    deriving it from a counter, so nothing has to stay in step with the code
    wrap: any pattern, any length and any alignment works, and an acquisition
    handover to an arbitrary chip cannot leave the subcarrier one position out.
    `phase` is the pattern position of chip 0 (0 for a code whose length is a
    whole number of pattern periods, which GPS L1C's 10230 = 33 x 310 is).
    """
    n = len(pattern)
    return [1 if pattern[(phase + c) % n] else 0 for c in range(code_length)]


def lut_rms(lut):
    """RMS amplitude of a subcarrier table -- the device's code amplitude.

    This is the number GNSSReceiver's `replica_code_amplitude` wants, on the
    scale `GNSSSignals.get_code_amplitude` reports: the primary chips are +/-1,
    so the replica's RMS is the table's.
    """
    return sqrt(sum(v * v for v in lut) / len(lut))


class ReplicaShape:
    """One channel's sub-chip replica: the LUT(s), the grid and the amplitude.

    `select` is None for every modulation but TMBOC; where it is set, it is the
    per-chip bit `tmboc_select_bits` produces and `lut_b` is the table those
    chips use.
    """
    def __init__(self, name, kind, subchips, lut_a, lut_b=None,
                 select=None, code_amplitude=None):
        assert len(lut_a) == subchips, "lut_a must cover every sub-chip"
        assert lut_b is None or len(lut_b) == subchips
        self.name       = name
        self.kind       = kind
        self.subchips   = subchips
        self.lut_a      = list(lut_a)
        self.lut_b      = list(lut_b) if lut_b is not None else None
        self.select     = select
        # Amplitude of the table the device correlates with. For TMBOC the two
        # tables are both +/-1, so either one's RMS is the replica's.
        self.code_amplitude = (lut_rms(lut_a) if code_amplitude is None
                               else code_amplitude)

    @property
    def peak(self):
        """Largest |LUT entry|: what the accumulator's headroom scales with."""
        vals = self.lut_a + (self.lut_b or [])
        return max(abs(v) for v in vals)

    def value(self, chip_sign, frac, select=0):
        """Replica value for a chip sign (+/-1) at fractional chip phase `frac`.

        `frac` is in [0, 1). This is the arithmetic the gateware performs, in
        Python, so a test can predict a dump without a second model of it.
        """
        k = int(frac * self.subchips)
        assert 0 <= k < self.subchips
        lut = self.lut_b if (select and self.lut_b is not None) else self.lut_a
        return chip_sign * lut[k]

    def __repr__(self):
        return (f"ReplicaShape({self.name!r}, kind={self.kind!r}, "
                f"subchips={self.subchips}, amplitude={self.code_amplitude:.6g})")


def replica_shape(kind, m=1, m2=6, a1=CBOC_A1, a2=CBOC_A2, boc2_sign=1,
                  name=None, code_length=None, pattern=TMBOC_L1CP_PATTERN):
    """Build the `ReplicaShape` for one GNSSSignals modulation."""
    name = name or kind
    if kind == "LOC":
        return ReplicaShape(name, kind, 1, loc_lut())
    if kind == "BOCsin":
        p = subchip_factor(kind, m)
        return ReplicaShape(name, kind, p, boc_sin_lut(m, p))
    if kind == "BOCcos":
        p = subchip_factor(kind, m)
        return ReplicaShape(name, kind, p, boc_cos_lut(m, p))
    if kind == "CBOC":
        p = subchip_factor(kind, m, m2)
        return ReplicaShape(name, kind, p,
                            cboc_lut(m, m2, a1, a2, boc2_sign, p))
    if kind == "TMBOC":
        p = subchip_factor(kind, m, m2)
        lut_a, lut_b = tmboc_luts(m, m2, p)
        select = (tmboc_select_bits(code_length, pattern)
                  if code_length is not None else None)
        return ReplicaShape(name, kind, p, lut_a, lut_b, select)
    raise ValueError(f"unknown modulation {kind!r}")


# The L1 signals GNSSSignals exposes, by the name it gives the type, with the
# modulation it reports for that type. Anything not here is not claimed.
SIGNAL_MODULATIONS = {
    "GPSL1CA":          dict(kind="LOC"),
    "GalileoE1B":       dict(kind="CBOC", m=1, m2=6, boc2_sign=+1),
    "GalileoE1C":       dict(kind="CBOC", m=1, m2=6, boc2_sign=-1),
    "GalileoE1B_BOC11": dict(kind="BOCsin", m=1),
    "GalileoE1C_BOC11": dict(kind="BOCsin", m=1),
    "GPSL1C_D":         dict(kind="BOCsin", m=1),
    "GPSL1C_P":         dict(kind="TMBOC", m=1, m2=6),
    "BeiDouB1C_D":      dict(kind="BOCsin", m=1),
    "BeiDouB1C_P":      dict(kind="BOCsin", m=1),
}

# Primary-code length of each, so `tmboc_select_bits` can be sized without the
# caller repeating GNSSSignals' table.
SIGNAL_CODE_LENGTHS = {
    "GPSL1CA": 1023,
    "GalileoE1B": 4092, "GalileoE1C": 4092,
    "GalileoE1B_BOC11": 4092, "GalileoE1C_BOC11": 4092,
    "GPSL1C_D": 10230, "GPSL1C_P": 10230,
    "BeiDouB1C_D": 10230, "BeiDouB1C_P": 10230,
}


def signal_replica_shape(signal, code_length=None):
    """`ReplicaShape` for a GNSSSignals signal name (see SIGNAL_MODULATIONS)."""
    try:
        spec = dict(SIGNAL_MODULATIONS[signal])
    except KeyError:
        raise ValueError(
            f"{signal!r} is not one of the L1 signals this gateware claims a "
            f"replica for: {', '.join(sorted(SIGNAL_MODULATIONS))}") from None
    if code_length is None:
        code_length = SIGNAL_CODE_LENGTHS[signal]
    return replica_shape(name=signal, code_length=code_length, **spec)
