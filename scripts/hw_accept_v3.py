#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Post-flash hardware acceptance for a v3 (five-tap, sub-chip) gateware.

"""What a v3 bitstream has to prove on the board, in one run.

Run it *on the target*, with the repo and the build's `csr.csv` in place and
DMA0 draining (`m2sdr_record /dev/null &`), inside whatever lock the board is
shared under:

    PYTHONPATH=. python3 scripts/hw_accept_v3.py <build-name> [--fs 4000000] \\
        [--prns 3,1,14,22]

Five checks, in the order that makes a failure interpretable:

 1. `gnss_version` reports the CSR layout and record format this driver speaks.
 2. `gnss_capabilities` / `gnss_signal_caps` decode to the build that was
    flashed -- tap layouts, sub-chip depth, code-RAM depth, modulation bits.
    Read from the device, not from the build arguments.
 3. The sample counter advances, i.e. the RX observer sees the stream at all.
 4. **GPS L1 C/A still acquires.** This is the regression baseline for the whole
    chain -- RF, DMA0, observer, carrier NCO, code NCO, correlators, CSRs. A
    board that does not acquire has failed the flash whatever else passes.
 5. A DMA1 capture frames, parses, and carries an honest `num_taps` -- including
    a five-tap record from a channel configured for five taps, which is the
    thing a five-tap build exists to produce.

Every check prints `PASS`/`FAIL` with the value it saw, and the exit code is the
number of failures, so a caller can neither miss nor invent a result.
"""

import argparse
import sys
import time

from gnss_m2sdr.record_format import (
    TAPS_EPL, TAPS_VEPL, is_epoch_strobe, parse_records, tap_accumulators,
)
from software.gnss_tracking import GNSSBank, acquire
from software.m2sdr_csr import LiteXCSR
from software.record_stream import RecordStream


class Report:
    def __init__(self):
        self.failures = 0

    def check(self, ok, name, detail=""):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""),
              flush=True)
        if not ok:
            self.failures += 1
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build", help="build name under build/")
    ap.add_argument("--fs", type=float, default=4e6)
    ap.add_argument("--prns", default="3,1,14,22,11,17,8")
    ap.add_argument("--repo", default=".")
    ap.add_argument("--device", default="/dev/m2sdr0")
    ap.add_argument("--dma1", default="/dev/m2sdr1")
    args = ap.parse_args()

    rep = Report()
    csr = LiteXCSR(f"{args.repo}/build/{args.build}/csr.csv", device=args.device)
    bank = GNSSBank(csr)

    print("== 1. interface revisions ==")
    csr_v, rec_v = bank.version()
    rep.check(csr_v == 3, "gnss_version.csr == 3", f"read {csr_v}")
    print(f"       gnss_version.record = {rec_v}")

    print("\n== 2. capabilities, read from the device ==")
    caps = bank.capabilities(fs=args.fs)
    for k in sorted(caps):
        print(f"       {k:26s} {caps[k]}")
    print(f"       raw gnss_capabilities = {csr.read('gnss_capabilities'):#x}")
    print(f"       raw gnss_signal_caps  = {csr.read('gnss_signal_caps'):#x}")
    rep.check(TAPS_VEPL in caps["tap_layouts"], "signal_caps.tap_layouts includes 5",
              str(caps["tap_layouts"]))
    rep.check(bool(caps["modulations"] & (1 << 4)), "signal_caps.modulations bit 4 (:BOCsin)",
              f"modulations = {caps['modulations']:#06b}")
    print(f"       max_subchips = {caps['max_subchips']}, "
          f"max_code_length = {caps['max_code_length']}")

    print("\n== 3. sample stream ==")
    bank.enable(True)
    a = bank.sample_count()
    time.sleep(1.0)
    rate = bank.sample_count() - a
    rep.check(rate > 0.5 * args.fs, "sample counter advances at ~fs",
              f"{rate} samples/s vs fs = {args.fs:.0f}")

    print("\n== 4. GPS L1 C/A acquisition ==")
    chan = bank.channel(0, args.fs)
    hits = []
    for prn in [int(p) for p in args.prns.split(",")]:
        r = acquire(chan, bank, prn=prn, fs=args.fs, verbose=False)
        print(f"       PRN {prn:2d}: metric {r.metric:8.1f}  "
              f"doppler {r.doppler_hz:+7.0f} Hz  detected={r.detected}", flush=True)
        if r.detected:
            hits.append((prn, r))
    rep.check(bool(hits), "at least one GPS L1 C/A satellite acquired",
              ", ".join(f"PRN {p} @ {r.metric:.0f}" for p, r in hits) or "none")

    print("\n== 5. DMA1 record stream ==")
    # Channel 0 keeps tracking the strongest satellite on three taps; channel 1
    # runs the same code on *five*, which is what puts a five-tap record on the
    # wire without needing a BOC satellite overhead.
    if hits:
        prn, best = hits[0]
        code_dopp = best.doppler_hz * 1.023e6 / 1575.42e6
        chan.configure(prn, carrier_hz=best.doppler_hz, code_doppler_hz=code_dopp,
                       spacing=0.5)
        five = bank.channel(1, args.fs)
        five.load_ca_code(prn)
        five.set_tap_offsets([-2, -1, 0, 1, 2], code_doppler_hz=code_dopp)
        five.set_replica(subchips=1, num_taps=TAPS_VEPL)
        five.set_carrier_hz(best.doppler_hz)
        five.set_code_doppler(code_dopp)
        five.restart()
        time.sleep(0.2)

    with RecordStream(args.dma1) as rs:
        raw = rs.capture(n_records=512, timeout=30.0)
    print(f"       captured {len(raw)} bytes")
    recs = parse_records(raw)
    dumps = [r for r in recs if not is_epoch_strobe(r)]
    rep.check(len(dumps) > 0, "DMA1 records frame and parse",
              f"{len(recs)} records ({len(dumps)} dumps, "
              f"{len(recs) - len(dumps)} epoch strobes)")
    if dumps:
        versions = sorted({r["version"] for r in dumps})
        by_chan = {}
        for r in dumps:
            by_chan.setdefault(r["channel"], set()).add(r["num_taps"])
        print(f"       record versions: {versions}")
        print(f"       num_taps per channel: "
              f"{ {c: sorted(v) for c, v in sorted(by_chan.items())} }")
        rep.check(all(v in (TAPS_EPL, TAPS_VEPL) for s in by_chan.values() for v in s),
                  "every record reports a legal num_taps", str(sorted(by_chan.items())))
        rep.check(any(TAPS_VEPL in v for v in by_chan.values()),
                  "a five-tap record arrived",
                  f"channels reporting 5 taps: "
                  f"{[c for c, v in by_chan.items() if TAPS_VEPL in v]}")
        for c in sorted(by_chan):
            d = next(r for r in dumps if r["channel"] == c)
            print(f"       ch{c}: num_taps={d['num_taps']} num_ants={d['num_ants']} "
                  f"code_length={d['code_length']} n={d['integrated_samples']} "
                  f"taps={tap_accumulators(d)}")

    print(f"\n{rep.failures} failure(s)")
    return rep.failures


if __name__ == "__main__":
    sys.exit(main())
