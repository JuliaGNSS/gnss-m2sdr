#!/usr/bin/env python3
"""Drift-compensated on-sky code-phase sweep.

Why the earlier static sweep could not work. The replica's phase advances at
code_step per sample, and so does the satellite's (code_step carries the Doppler),
so there is no *rate* mismatch -- but each swept phase is restarted at a
different sample, and the satellite's own phase has moved on by then. Restarting
at a fixed chip number therefore measures a different relative offset every time:
at -3600 Hz the code phase runs at ~2.3 chips/s, so a 22 s sweep smears the peak
over ~50 bins and a re-sweep minutes later looks nowhere near it.

The fix needs no raw capture and no CPU acquisition. `schedule()` commits the
restart on an exact sample index, and the phase the satellite will have at that
sample is known from the counter alone:

    phi_j = phi_base + (S_j - S_0) * code_step / 2**24   (mod 1023)

so every measurement is taken in the satellite's own frame and the peak sits at
one phi_base for the whole sweep.

Usage: sweep_compensated.py PRN DOPPLER_HZ [STEP_CHIPS] [DUMPS] [MARGIN_SAMPLES]
"""
import os
import sys
import time

sys.path.insert(0, "/home/orin/gnss-m2sdr")
from software.m2sdr_csr import LiteXCSR
from software.gnss_tracking import GNSSChannel, GNSSBank, prompt_power
from gnss_m2sdr.gps_ca import CA_CODE_LENGTH

CSV = os.environ.get("GNSS_CSR_CSV",
                    "build/gnss_m2sdr_m2_x1_ch2_ant1/csr.csv")
FRAC = 24

prn = int(sys.argv[1])
doppler = float(sys.argv[2])
step = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
per_phase = int(sys.argv[4]) if len(sys.argv) > 4 else 8
margin = int(sys.argv[5]) if len(sys.argv) > 5 else 24000     # 6 ms of CSR slack

csr = LiteXCSR(CSV)
bank = GNSSBank(csr)

c0 = bank.sample_count(); t0 = time.perf_counter()
time.sleep(6.0)
fs = (bank.sample_count() - c0) / (time.perf_counter() - t0)
ch = GNSSChannel(csr, fs, 0)
p = ch.p
code_step = ch.code_word(doppler)
print(f"measured fs = {fs:,.1f} Hz, code_step = {code_step} "
      f"({code_step / (1 << FRAC):.6f} chips/sample)")

bank.enable(True)
ch.load_code(prn)
ch.set_spacing_chips(0.5, doppler)
ch.set_carrier_hz(doppler)
ch.set_code_doppler(doppler)

S0 = bank.sample_count()
phases = [i * step for i in range(int(CA_CODE_LENGTH / step))]
print(f"PRN {prn} at {doppler:+.0f} Hz, {len(phases)} phases x {step} chips, "
      f"{per_phase} dumps each, drift-compensated from S0={S0}")

results = []
late_count = 0
t_start = time.time()
for phi_base in phases:
    target = bank.sample_count() + margin
    advance = (target - S0) * code_step / (1 << FRAC)
    phi = (phi_base + advance) % CA_CODE_LENGTH
    ch.schedule(target, carrier_hz=doppler, code_doppler_hz=doppler,
                code_phase_chips=phi)
    armed, late = ch.apply_status()
    if late:
        late_count += 1
    # Wait for the commit to land, then measure whole periods after it.
    while True:
        armed, _ = ch.apply_status()
        if not armed:
            break
    powers, seen = [], 0
    while len(powers) < per_phase:
        d = ch.wait_dump(timeout=0.05)
        if d is None:
            break
        seen += 1
        if seen > 2:               # first dump after a restart is partial
            powers.append(prompt_power(d))
    if powers:
        results.append((phi_base, sum(powers) / len(powers), max(powers), len(powers)))

elapsed = time.time() - t_start
print(f"swept in {elapsed:.0f} s ({late_count} late commits), "
      f"{len(results)} phases measured")

by_mean = sorted(results, key=lambda r: -r[1])
means = sorted(r[1] for r in results)
floor = means[len(means) // 2]
print("\ntop 10 phases by mean prompt power:")
for phi_base, mean, mx, nn in by_mean[:10]:
    print(f"  phi_base {phi_base:8.2f}  mean {mean:14.0f}  {mean / floor:6.2f}x floor  n={nn}")
print(f"\n  floor            = {floor:.0f}")
print(f"  best / floor     = {by_mean[0][1] / floor:.2f}x")
print(f"  95th pct / floor = {means[int(0.95 * len(means))] / floor:.2f}x")
print(f"  99th pct / floor = {means[int(0.99 * len(means))] / floor:.2f}x")

# A real peak is ~2 chips wide, so its immediate neighbours must be elevated too.
best = by_mean[0][0]
lookup = {r[0]: r[1] for r in results}
print("\nprofile around the best phase:")
for k in range(-4, 5):
    ph = (best + k * step) % CA_CODE_LENGTH
    if ph in lookup:
        print(f"  {ph:8.2f}  {lookup[ph] / floor:6.2f}x")
csr.close()
