#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Host-side control of the on-FPGA GNSS tracking channels: configure carrier /
# code NCOs, load PRN codes, read correlator dumps, and acquire a satellite by
# a sliding-correlator + Doppler sweep. Pure Python over LiteXCSR (ioctl).

import math
import time

from typing import NamedTuple, Optional

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

    def sample_shift(self, spacing_chips, code_doppler_hz=0.0):
        """Tracking.jl's E/L shift in whole input samples.

        `calc_preferred_code_shift_to_sample_shift` rounds the preferred chip
        shift to an integer number of samples (at least 1), and `dll_disc`
        derives its (2 - d)/2 normalisation from *that* quantised spacing. The
        step word (not the float chip rate) is the divisor so the shift is
        expressed on the same grid as the NCO actually programmed.
        """
        step = self.code_word(code_doppler_hz)
        return max(1, int(round(spacing_chips * (1 << self.fb) / step)))

    def spacing_word(self, spacing_chips, code_doppler_hz=0.0):
        """E/L half-spacing CSR word: `sample_shift` whole NCO samples.

        `sample_shift * code_step` puts the Early tap exactly that many samples
        ahead of the prompt (and Late that many behind) with no rounding drift
        between the two fixed-point words -- programming the raw preferred shift
        instead leaves the accumulators at a spacing `dll_disc` does not assume
        (~2.3 % DLL loop-gain error at fs = 4 MHz, 0.5 chips). The E/L taps only
        reach chip index +/- 1, so the result must stay below one chip.
        """
        step = self.code_word(code_doppler_hz)
        word = self.sample_shift(spacing_chips, code_doppler_hz) * step
        if word >= (1 << self.fb):
            raise ValueError(
                f"E/L spacing {word / (1 << self.fb):.3f} chips >= 1 chip: the "
                f"E/L taps only reach chip index +/-1 (preferred {spacing_chips} "
                f"chips at fs={self.fs:.0f} Hz)")
        return word

    def set_spacing_chips(self, d, code_doppler_hz=0.0):
        self.csr.write(self.p + "spacing", self.spacing_word(d, code_doppler_hz))

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

    # ---- scheduled updates (deterministic apply point) -----------------------
    def carrier_phase_word(self, cycles):
        return round((cycles % 1.0) * (1 << self.pb)) & ((1 << self.pb) - 1)

    def code_phase_word(self, chips):
        """Code phase (chips, fractional) -> the chip|frac word of code_phase."""
        phase = chips % CA_CODE_LENGTH
        chip  = int(phase)
        frac  = round((phase - chip) * (1 << self.fb))
        if frac == (1 << self.fb):                     # rounding carried a chip
            chip, frac = (chip + 1) % CA_CODE_LENGTH, 0
        return (chip << self.fb) | frac

    def schedule(self, sample_index, carrier_hz=None, code_doppler_hz=None,
                 carrier_phase_cycles=None, code_phase_chips=None):
        """Commit the given values on global sample `sample_index`, atomically.

        `sample_index` lives on the bank's free-running counter
        (`GNSSBank.sample_count`, the axis records are timestamped on) and is the
        first input sample processed with the new values -- the hardware meaning
        of GNSSReceiver.jl's `NCOUpdate.apply_at_epoch`, which is what lets the
        loop filter work with a fixed feedback delay instead of PCIe jitter.
        Only the values passed are committed; passing `code_phase_chips` (i.e. an
        acquisition handover) also restarts the integration on that sample.

        Call `apply_status()` afterwards: `late` means the writes did not make it
        to the board in time and the commit slipped to a later sample.
        """
        w, p = self.csr.write, self.p
        flags = 0b1                                    # arm
        if carrier_hz is not None:
            w(p + "carrier_freq_next", self.carrier_word(carrier_hz))
            flags |= 1 << 3
        if code_doppler_hz is not None:
            w(p + "code_freq_next", self.code_word(code_doppler_hz))
            flags |= 1 << 4
        if carrier_phase_cycles is not None:
            w(p + "carrier_phase", self.carrier_phase_word(carrier_phase_cycles))
            flags |= 1 << 2
        if code_phase_chips is not None:
            w(p + "code_phase", self.code_phase_word(code_phase_chips))
            flags |= 1 << 1
        w(p + "apply_at", sample_index)
        w(p + "apply", 0)                              # arm is 0->1 edge-triggered
        w(p + "apply", flags)

    def apply_status(self):
        """(armed, late) of the last scheduled commit."""
        s = self.csr.read(self.p + "apply_status")
        return bool(s & 0b01), bool(s & 0b10)

    def applied_at(self):
        """Global sample index the last scheduled commit actually took effect for."""
        return self.csr.read(self.p + "applied_at")

    def configure(self, prn, carrier_hz, code_doppler_hz=0.0, spacing=0.5):
        self.load_code(prn)
        self.set_spacing_chips(spacing, code_doppler_hz)
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


def peak_code_phase(dump, frac_bits, code_length=CA_CODE_LENGTH):
    """Code phase (chips) of the incoming signal at `dump["sample_index"]`.

    A dump fires on the sample whose advance wraps the last chip, so on that
    sample the replica sits at chip `code_length - 1` plus the fractional phase
    reported in `dump_code_phase` -- and on the peak dump the replica is (to
    within the correlator's resolution) aligned with the signal, so that is the
    signal's code phase too. There is no separate code-phase readback: the phase
    is implicit in *which* dump peaked, and `sample_index` is what pins it to
    the bank's global sample axis.
    """
    return (code_length - 1) + dump["code_phase"] / float(1 << frac_bits)


class AcquisitionResult(NamedTuple):
    """What `acquire()` hands over to a tracking channel.

    The first three fields keep the historical (metric, doppler, power) order,
    so `metric, doppler, power = result[:3]` still works.

    code_phase   : chips [0, code_length), the signal's code phase at
                   `sample_index`; None if no dump was ever read.
    sample_index : global input-sample counter value the phase refers to (the
                   bank's one free-running axis, see record_format.py) -- a code
                   phase without it is meaningless, since the phase advances by
                   the code rate every sample.
    detected     : metric reached the detection threshold.
    """
    metric:       float
    doppler_hz:   float
    power:        float
    code_phase:   Optional[float]
    sample_index: Optional[int]
    detected:     bool

    def code_phase_at(self, sample_index, fs, code_length=CA_CODE_LENGTH):
        """Propagate the acquired code phase to another global sample index.

        The handover arithmetic: the code runs on at the acquired Doppler, so a
        channel started at `sample_index` must begin at this phase.
        """
        fc = GPS_CA_CHIP_RATE * (1.0 + self.doppler_hz / GPS_L1_HZ)
        return (self.code_phase
                + (sample_index - self.sample_index) * fc / fs) % code_length


class GNSSBank:
    def __init__(self, csr):
        self.csr = csr

    def enable(self, on=True):
        self.csr.write("gnss_control", 1 if on else 0)

    def overflow(self):
        """Sticky per-channel overflow mask: a set bit means that channel lost at
        least one correlator dump since the last clear_overflow(). Sticky, so a
        slow poller cannot miss it (the per-record FLAG_OVERFLOW marks *where*).
        """
        return self.csr.read("gnss_overflow")

    def dropped(self, index):
        """Saturating count of dumps lost on channel `index` since the last clear."""
        return self.csr.read(f"gnss_dropped{index}")

    def clear_overflow(self, mask=0xFFFFFFFF):
        """Write-1-to-clear the sticky overflow bits and their drop counters.

        Read overflow()/dropped() first: a drop landing on the clear cycle is
        kept (the counter restarts at 1), so no event is lost to the clear.
        """
        self.csr.write("gnss_overflow_clear", mask)

    def sample_count(self, tries=3):
        """Global free-running input-sample counter (the record timestamp axis).

        Read this next to a raw DMA0 capture to place the capture on the same
        axis as the correlator records. The 64-bit CSR is two 32-bit reads and
        the counter is live, so re-read the high word and retry if the low word
        wrapped in between (every ~2**32 samples, i.e. ~17 min at 4.092 MHz).
        """
        addr, _ = self.csr.regs["gnss_sample_count"]   # MSW at the lowest addr
        for _ in range(tries):
            hi = self.csr._readl(addr)
            lo = self.csr._readl(addr + 4)
            if self.csr._readl(addr) == hi:
                break
        return (hi << 32) | lo

    def sample_index_julia(self, record_sample_index, chunk_origin):
        """Record timestamp -> Tracking.jl's 1-based per-chunk sample_index."""
        return record_sample_index - chunk_origin + 1


def acquire(chan, bank, prn, fs, doppler_range=5000.0, doppler_step=500.0,
            slide_chips=800.0, dwell=1.4, detect_metric=8.0, verbose=True):
    """Sliding-correlator acquisition of `prn` (validated on hardware, PRN 24
    detected live with peak/median >> 100).

    For each trial Doppler, offset the code rate by `slide_chips` chips/s so the
    code phase slides through all 1023 chips within `dwell`; collect prompt
    power over the dumps and score peak/median (noise ~5-10; a live PRN gives
    tens to hundreds).

    Returns an AcquisitionResult: metric, Doppler, peak power, *and* the code
    phase of the peak plus the global sample index it applies to, which is what
    a tracking channel has to be started at. Requires DMA0 to be draining (e.g.
    `m2sdr_record /dev/null &`) so the RX observer sees samples.

    Bring-up limits of this sliding scheme (none of them a sensitivity limit of
    the correlators themselves):
      * `wait_dump()` busy-polls CSRs and readback is lossy -- a dump can be
        replaced before it is read -- so some code-phase bins are skipped and
        the true peak can be missed. The lossless DMA1 record path
        (record_format.py) has neither problem.
      * sliding `slide_chips / 1000` chips *within* each 1 ms integration also
        smears the correlation peak, costing a few dB.
      * the code phase is therefore resolved only to about the per-dump slide
        (+-0.4 chips at the default 800 chips/s); a code-phase-set CSR plus an
        explicit phase sweep would replace both the sliding and this estimate.
    """
    import statistics
    chan.load_code(prn)
    chan.set_spacing_chips(0.5)
    bank.enable(True)
    off = round(slide_chips / fs * (1 << chan.fb))
    best = AcquisitionResult(0.0, 0.0, 0.0, None, None, False)
    d = -doppler_range
    while d <= doppler_range:
        chan.set_carrier_hz(d)
        chan.csr.write(chan.p + "code_freq", chan.code_word(d) + off)
        chan.restart()
        t0 = time.time()
        dumps = []
        while time.time() - t0 < dwell:
            dd = chan.wait_dump(timeout=0.05)
            if dd is not None:
                dumps.append(dd)
        if len(dumps) > 20:
            # Keep the peak *dump*, not just its power: its code phase and
            # sample index are the half of the answer tracking needs.
            powers = [prompt_power(dd) for dd in dumps]
            peak   = max(dumps, key=prompt_power)
            med    = statistics.median(powers)
            metric = (max(powers) / med) if med > 0 else 0.0
            if verbose:
                mark = "  <== DETECTED" if metric >= detect_metric else ""
                print(f"  doppler {d:+6.0f} Hz : peak/median {metric:6.1f}{mark}")
            if metric > best.metric:
                best = AcquisitionResult(metric, d, prompt_power(peak),
                                         peak_code_phase(peak, chan.fb),
                                         peak["sample_index"], False)
        d += doppler_step
    return best._replace(detected=best.metric >= detect_metric)
