#!/usr/bin/env python3
"""Drift-compensated on-sky code-phase sweep, with a refine pass in the same run.

Why a static sweep cannot find a satellite. There is no *rate* mismatch --
code_step carries the Doppler, so replica and satellite advance together -- but
each swept phase is restarted at a different sample, and the satellite's own phase
has moved on by then. Restarting at a fixed chip number therefore measures a
different relative offset every time: at -6800 Hz the code phase runs at
~4.4 chips/s, so a 16 s sweep smears the peak over tens of bins.

This sweeps in the satellite's frame instead. `schedule()` commits the restart on
an exact sample index, and the phase the satellite will have at that sample
follows from the counter alone:

    phi_j = phi_base + (S_j - S_0) * code_step / 2**24   (mod 1023)

so the peak sits at one phi_base for the whole sweep. No raw capture and no CPU
acquisition needed -- only the PRN and Doppler.

Two things that are easy to get wrong, both of which cost a session:

* **fs must be measured, not assumed.** A one-second count of the bank's counter
  is only good to ~20 ppm, and being 67 ppm off is ~68 chips/s of code drift --
  enough that averaging dumps at one phase washes out the peak it is averaging
  for. This measures over several seconds.
* **`phi_base` is only defined relative to `S_0`.** A refine pass in a *separate*
  run has a different phase origin, and code_step's quantisation makes the offset
  untransferable (2e8 samples of accumulated rounding is several chips). So the
  refine pass runs here, in the same process, against the same S_0 and fs.

RESOLVED -- this now finds a live satellite. Two host-side errors were in the way,
neither in the gateware:

* **The Doppler must come from a search wider than +-7 kHz.** The reference drives
  both the RX LO and the sample clock, so its offset shows up as an apparent
  Doppler on top of the satellite's own +-5 kHz; observed offsets past 9 kHz are
  normal here. Acquisition.jl's default `min_doppler_coverage = 7000Hz` clips
  those to the edge of its grid, and the resulting ~1 kHz carrier error is exactly
  the first null of a 1 ms coherent integration -- it annihilates the correlation
  rather than merely degrading it.
* **Refer every NCO word to the NOMINAL sampling frequency**, the one the
  acquisition was given -- not a measured one. Because the LO error appears as a
  Doppler on the code rate too, the acquisition's observed offset is then directly
  the right number for both carrier and code. Using a measured fs (or "correcting"
  for the LO) puts back the very error it looks like it removes, and a few chips/s
  is tens of chips across a sweep.

With both fixed, PRN 17 gave a clean correlation triangle: 9.1x the floor at the
peak, monotonic over ~1.5 chips, background ~1.0x.

Usage:
  sweep_compensated.py PRN DOPPLER_HZ [STEP] [DUMPS] [MARGIN] [REFINE_STEP] [REFINE_DUMPS]
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from m2sdr_csr import LiteXCSR
except ImportError:
    from software.m2sdr_csr import LiteXCSR
from software.gnss_tracking import GNSSChannel, GNSSBank, prompt_power
from gnss_m2sdr.gps_ca import CA_CODE_LENGTH, GPS_CA_CHIP_RATE, GPS_L1_HZ

CSV = os.environ.get("GNSS_CSR_CSV", "build/gnss_m2sdr_m2_x1_ch2_ant1/csr.csv")
FRAC = 24
FS_NOMINAL = 4e6            # must match what the acquisition was given
FS_MEASURE_SECONDS = 4.0    # diagnostic only
SKIP_AFTER_RESTART = 2      # the first dump after a restart is a partial period


def measure(bank, ch, carrier_hz, code_doppler_hz, code_step, S0, margin,
            phase_list, dumps_each):
    """Mean prompt power at each phase in `phase_list`, in the satellite's frame."""
    out, late_n = [], 0
    for phi_base in phase_list:
        target = bank.sample_count() + margin
        advance = (target - S0) * code_step / (1 << FRAC)
        phi = (phi_base + advance) % CA_CODE_LENGTH
        ch.schedule(target, carrier_hz=carrier_hz, code_doppler_hz=code_doppler_hz,
                    code_phase_chips=phi)
        _, late = ch.apply_status()
        if late:
            late_n += 1
        while True:                       # wait for the commit to land
            armed, _ = ch.apply_status()
            if not armed:
                break
        powers, seen = [], 0
        while len(powers) < dumps_each:
            dump = ch.wait_dump(timeout=0.05)
            if dump is None:
                break
            seen += 1
            if seen > SKIP_AFTER_RESTART:
                powers.append(prompt_power(dump))
        if powers:
            out.append((phi_base, sum(powers) / len(powers), len(powers)))
    return out, late_n


def report(rows, label):
    means = sorted(r[1] for r in rows)
    floor = means[len(means) // 2]
    ranked = sorted(rows, key=lambda r: -r[1])
    print(f"\n{label}: {len(rows)} phases, floor {floor:.0f}")
    for phi_base, mean, nn in ranked[:8]:
        print(f"  phi_base {phi_base:8.2f}  {mean / floor:6.2f}x floor  (n={nn})")
    hi = min(len(means) - 1, int(0.99 * len(means)))
    print(f"  best {ranked[0][1] / floor:.2f}x, 95th pct "
          f"{means[int(0.95 * len(means))] / floor:.2f}x, 99th pct "
          f"{means[hi] / floor:.2f}x")
    return ranked[0][0], floor


def main():
    prn = int(sys.argv[1])
    doppler = float(sys.argv[2])
    step = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    per_phase = int(sys.argv[4]) if len(sys.argv) > 4 else 8
    margin = int(sys.argv[5]) if len(sys.argv) > 5 else 24000
    refine_step = float(sys.argv[6]) if len(sys.argv) > 6 else 0.25
    refine_dumps = int(sys.argv[7]) if len(sys.argv) > 7 else 40

    csr = LiteXCSR(CSV)
    bank = GNSSBank(csr)

    c0 = bank.sample_count()
    t0 = time.perf_counter()
    time.sleep(FS_MEASURE_SECONDS)
    fs = (bank.sample_count() - c0) / (time.perf_counter() - t0)
    # Use the NOMINAL sampling frequency, i.e. the one the acquisition was told
    # about -- not the measured one. The reference drives both the RX LO and the
    # sample clock, so referring carrier and code to the same nominal rate makes
    # the LO error come out as an apparent Doppler on both, and the acquisition's
    # observed offset is then directly the right number for each. No LO correction
    # is needed or wanted; "correcting" it (or using the measured fs) reintroduces
    # exactly the error it was meant to remove.
    ch = GNSSChannel(csr, FS_NOMINAL, 0)
    code_step = ch.code_word(doppler)
    drift = doppler * GPS_CA_CHIP_RATE / GPS_L1_HZ
    print(f"measured fs = {fs:,.1f} Hz ({(fs / FS_NOMINAL - 1) * 1e6:+.2f} ppm) "
          f"-- diagnostic only; the NCO words use the nominal {FS_NOMINAL:,.0f} Hz")
    print(f"code_step = {code_step}; satellite code phase drifts {drift:+.2f} chips/s")

    bank.enable(True)
    ch.load_code(prn)
    ch.set_spacing_chips(0.5, doppler)
    ch.set_carrier_hz(doppler)
    ch.set_code_doppler(doppler)

    S0 = bank.sample_count()
    phases = [i * step for i in range(int(CA_CODE_LENGTH / step))]
    print(f"\nPRN {prn} at {doppler:+.0f} Hz: coarse {len(phases)} x {step} chips, "
          f"{per_phase} dumps each, S0={S0}")
    t = time.time()
    coarse, late = measure(bank, ch, doppler, doppler, code_step, S0, margin,
                           phases, per_phase)
    print(f"  coarse sweep took {time.time() - t:.0f} s ({late} late commits)")
    best, _ = report(coarse, "COARSE")

    halfwidth = 6.0
    span = [(best - halfwidth + refine_step * i) % CA_CODE_LENGTH
            for i in range(int(round(2 * halfwidth / refine_step)) + 1)]
    print(f"\nrefining +-{halfwidth:.0f} chips around {best:.2f} at {refine_step} chips, "
          f"{refine_dumps} dumps each (same S0, same fs)")
    t = time.time()
    fine, late = measure(bank, ch, doppler, doppler, code_step, S0, margin,
                         span, refine_dumps)
    print(f"  refine took {time.time() - t:.0f} s ({late} late commits)")
    if fine:
        _, ffloor = report(fine, "REFINED")
        print("\nprofile:")
        for phi_base, mean, nn in fine:
            print(f"  {phi_base:8.2f}  {mean / ffloor:6.2f}x  "
                  + "#" * int(round(20 * mean / ffloor)))
    csr.close()


if __name__ == "__main__":
    main()
