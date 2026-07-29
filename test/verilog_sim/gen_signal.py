#!/usr/bin/env python3
"""Synthesise a GPS L1 C/A signal of exactly known PRN, code phase and Doppler,
plus the NCO words the host would program for it, for the injection testbench.

Reference: this reproduces what GNSSSignals.jl's
`gen_code(n, GPSL1CA(), prn, fs, code_frequency, start_code_phase)` produces, and
the C/A chips of `gnss_m2sdr.gps_ca.ca_code_reference` were checked bit-exact
against GNSSSignals.jl's `get_code` for PRN 1 (and all 32 PRNs by
test_ca_code_vs_gnsssignals.py), so the signal and the gateware's replica come
from independently written generators.

Length is chosen so the sample stream loops seamlessly: one code period is
exactly 4000 samples at 4 MHz (1023 chips / 1.023 MHz = 1 ms), so 32 periods is a
whole number of periods -- which also means the injected signal presents the same
code phase at every restart of the sweep, and the peak stays at one replica phase.

Usage: gen_signal.py OUTDIR PRN DOPPLER_HZ CODE_PHASE_CHIPS [AMPLITUDE]
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from gnss_m2sdr.gps_ca import (
    ca_code_reference, CA_CODE_LENGTH, GPS_L1_HZ, GPS_CA_CHIP_RATE,
)

FS = 4_000_000.0
PERIODS = 32
N = 4000 * PERIODS
CARRIER_PHASE = math.pi / 4          # so both I and Q carry signal
FRAC_BITS = 24
PHASE_BITS = 32


def main():
    outdir = sys.argv[1]
    prn = int(sys.argv[2])
    doppler = float(sys.argv[3])
    code_phase = float(sys.argv[4])
    amplitude = float(sys.argv[5]) if len(sys.argv) > 5 else 1500.0

    os.makedirs(outdir, exist_ok=True)
    chips = ca_code_reference(prn)

    code_frequency = GPS_CA_CHIP_RATE * (1.0 + doppler / GPS_L1_HZ)
    step = code_frequency / FS                      # chips per sample

    with open(os.path.join(outdir, "samples.txt"), "w") as f:
        for n in range(N):
            phase = (code_phase + n * step) % CA_CODE_LENGTH
            chip = 1 if chips[int(phase)] else -1
            ang = 2 * math.pi * doppler * n / FS + CARRIER_PHASE
            i = int(round(amplitude * chip * math.cos(ang)))
            q = int(round(amplitude * chip * math.sin(ang)))
            f.write(f"{i & 0xFFFF:04x} {q & 0xFFFF:04x}\n")

    with open(os.path.join(outdir, "chips.txt"), "w") as f:
        for c in chips:
            f.write(f"{c & 1}\n")

    # Exactly what GNSSChannel.code_word() / carrier_word() compute.
    code_step = round(code_frequency / FS * (1 << FRAC_BITS))
    carrier_fw = round(doppler / FS * (1 << PHASE_BITS))
    expect = int(round(code_phase)) % CA_CODE_LENGTH
    with open(os.path.join(outdir, "params.txt"), "w") as f:
        f.write(f"{code_step} {carrier_fw} {expect}\n")

    print(f"{outdir}: PRN {prn}, doppler {doppler} Hz, code phase {code_phase} chips")
    print(f"  {N} samples ({PERIODS} code periods), amplitude {amplitude}")
    print(f"  code_step={code_step} carrier_fw={carrier_fw} expected peak={expect}")
    print(f"  aligned prompt should be ~127 * {round(amplitude * math.cos(CARRIER_PHASE))}"
          f" * 4000 = {127 * round(amplitude * math.cos(CARRIER_PHASE)) * 4000}")


if __name__ == "__main__":
    main()
