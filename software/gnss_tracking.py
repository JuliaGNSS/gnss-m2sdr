#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Host-side control of the on-FPGA GNSS tracking channels: configure carrier /
# code NCOs, load PRN codes, read correlator dumps, and acquire a satellite by
# a sliding-correlator + Doppler sweep. Pure Python over LiteXCSR (ioctl).

import math
import time

try:
    from m2sdr_csr import LiteXCSR
except ImportError:                      # when imported as software.gnss_tracking
    from software.m2sdr_csr import LiteXCSR
from gnss_m2sdr.gps_ca import (
    ca_code_reference, CA_CODE_LENGTH, GPS_L1_HZ, GPS_CA_CHIP_RATE,
)


class GNSSChannel:
    def __init__(self, csr, fs, index=0, carrier_phase_bits=32, code_frac_bits=24):
        self.csr = csr
        self.fs  = float(fs)
        self.i   = index
        self.pb  = carrier_phase_bits
        self.fb  = code_frac_bits
        self.p   = f"gnss_ch{index}_"

    # ---- configuration -------------------------------------------------------
    def carrier_word(self, hz):
        return round(hz / self.fs * (1 << self.pb)) & ((1 << self.pb) - 1)

    def code_word(self, doppler_hz=0.0):
        # Code rate scales with carrier Doppler: fc = chip_rate*(1 + fd/L1).
        fc = GPS_CA_CHIP_RATE * (1.0 + doppler_hz / GPS_L1_HZ)
        return round(fc / self.fs * (1 << self.fb)) & ((1 << self.fb) - 1)

    def set_carrier_hz(self, hz):
        self.csr.write(self.p + "carrier_freq", self.carrier_word(hz))

    def set_code_doppler(self, doppler_hz):
        self.csr.write(self.p + "code_freq", self.code_word(doppler_hz))

    def set_spacing_chips(self, d):
        self.csr.write(self.p + "spacing", int(round(d * (1 << self.fb))))

    def set_prn(self, prn):
        self.csr.write(self.p + "prn", prn)

    def load_code(self, prn):
        """Load PRN's 1023 chips into the channel code RAM (reset addr, then write)."""
        code = ca_code_reference(prn)
        self.csr.write(self.p + "code_load", 0b100)          # reset_addr
        for chip in code:
            self.csr.write(self.p + "code_load", 0b010 | (chip & 1))  # we | dat
        self.set_prn(prn)

    def restart(self):
        # Edge-triggered: 0 -> 1 pulses restart + carrier_set (both bits).
        self.csr.write(self.p + "control", 0)
        self.csr.write(self.p + "control", 0b11)
        self.csr.write(self.p + "control", 0)

    def configure(self, prn, carrier_hz, code_doppler_hz=0.0, spacing=0.5):
        self.load_code(prn)
        self.set_spacing_chips(spacing)
        self.set_carrier_hz(carrier_hz)
        self.set_code_doppler(code_doppler_hz)
        self.restart()

    # ---- readback ------------------------------------------------------------
    def read_dump(self):
        r = self.csr.read
        rs = lambda n: self.csr.read_signed(self.p + n, 32)
        return dict(
            count = r(self.p + "dump_count"),
            ip = rs("ip"), qp = rs("qp"),
            ie = rs("ie"), qe = rs("qe"),
            il = rs("il"), ql = rs("ql"),
            n  = r(self.p + "integrated_samples"),
            sample_index = r(self.p + "sample_index"),
            code_phase   = r(self.p + "dump_code_phase"),
        )

    def wait_dump(self, timeout=1.0):
        """Block until dump_count changes; return a coherent dump dict."""
        c0 = self.csr.read(self.p + "dump_count")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.csr.read(self.p + "dump_count") != c0:
                d = self.read_dump()
                if self.csr.read(self.p + "dump_count") == d["count"]:
                    return d
        return None


def prompt_power(d):
    return d["ip"] ** 2 + d["qp"] ** 2


class GNSSBank:
    def __init__(self, csr):
        self.csr = csr

    def enable(self, on=True):
        self.csr.write("gnss_control", 1 if on else 0)

    def overflow(self):
        return self.csr.read("gnss_overflow")


def acquire(chan, bank, prn, fs, doppler_range=5000.0, doppler_step=500.0,
            slide_chips=800.0, dwell=1.4, detect_metric=8.0, verbose=True):
    """Sliding-correlator acquisition of `prn` (validated on hardware, PRN 24
    detected live with peak/median >> 100).

    For each trial Doppler, offset the code rate by `slide_chips` chips/s so the
    code phase slides through all 1023 chips within `dwell`; collect prompt
    power over the dumps and score peak/median (noise ~5-10; a live PRN gives
    tens to hundreds). Returns (best_metric, best_doppler, best_peak_power).
    Requires DMA0 to be draining (e.g. `m2sdr_record /dev/null &`) so the RX
    observer sees samples.
    """
    import statistics
    chan.load_code(prn)
    chan.set_spacing_chips(0.5)
    bank.enable(True)
    off = round(slide_chips / fs * (1 << chan.fb))
    best = (0.0, 0, 0.0)
    d = -doppler_range
    while d <= doppler_range:
        chan.set_carrier_hz(d)
        chan.csr.write(chan.p + "code_freq", chan.code_word(d) + off)
        chan.restart()
        t0 = time.time()
        powers = []
        while time.time() - t0 < dwell:
            dd = chan.wait_dump(timeout=0.05)
            if dd is not None:
                powers.append(prompt_power(dd))
        if len(powers) > 20:
            med = statistics.median(powers)
            metric = (max(powers) / med) if med > 0 else 0.0
            if verbose:
                mark = "  <== DETECTED" if metric >= detect_metric else ""
                print(f"  doppler {d:+6.0f} Hz : peak/median {metric:6.1f}{mark}")
            if metric > best[0]:
                best = (metric, d, max(powers))
        d += doppler_step
    return best
