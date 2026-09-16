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
from gnss_m2sdr.record_format import (
    CSR_LAYOUT_VERSION, MAX_TAP_OFFSET_CHIPS, RECORD_FORMAT_VERSION,
    TAPS_EPL, TAPS_VEPL, tap_short_names,
)
from gnss_m2sdr.subcarrier import ReplicaShape, signal_replica_shape


class GNSSChannel:
    """Host side of one hardware tracking channel.

    The signal a channel replicates is *configuration*, not a constant of this
    driver: `code_length` chips at `chip_rate` chips/s, whose Doppler scales
    with the carrier at `carrier_freq`. The defaults are GPS L1 C/A, so an
    existing caller sees no change; a Galileo E1B channel is the same class with
    `code_length=4092`, and a GPS L5 channel with `code_length=10230,
    chip_rate=10.23e6, carrier_freq=1176.45e6`.
    """
    def __init__(self, csr, fs, index=0, carrier_phase_bits=32, code_frac_bits=24,
                 code_length=CA_CODE_LENGTH, chip_rate=GPS_CA_CHIP_RATE,
                 carrier_freq=GPS_L1_HZ, num_taps=TAPS_EPL, max_subchips=1,
                 replica_bits=2):
        self.csr = csr
        self.fs  = float(fs)
        self.i   = index
        self.pb  = carrier_phase_bits
        self.fb  = code_frac_bits
        self.p   = f"gnss_ch{index}_"
        self.code_length  = int(code_length)
        self.chip_rate    = float(chip_rate)
        self.carrier_freq = float(carrier_freq)
        # What the *build* can do. GNSSBank fills these from gnss_capabilities /
        # gnss_signal_caps; the defaults are the lean three-tap BPSK build, so a
        # channel constructed by hand against an old bitstream behaves as before.
        self.num_taps     = int(num_taps)
        self.max_subchips = int(max_subchips)
        self._replica_bits = int(replica_bits)
        self._num_ants = None    # discovered from the CSR set on first use

    # ---- build limits --------------------------------------------------------
    def replica_bits(self):
        """Signed width of one subcarrier-table entry in this build."""
        return self._replica_bits

    def _sub_adr_bits(self):
        return max(1, (self.max_subchips - 1).bit_length())

    def _subchips_bits(self):
        return max(1, self.max_subchips.bit_length())

    def _has_five_taps(self):
        return self.num_taps >= TAPS_VEPL

    # ---- configuration -------------------------------------------------------
    def carrier_word(self, hz):
        return round(hz / self.fs * (1 << self.pb)) & ((1 << self.pb) - 1)

    def code_word(self, doppler_hz=0.0, chip_rate=None):
        """Code NCO step word for this channel's chip rate and a carrier Doppler.

        The code NCO crosses at most one chip boundary per input sample, so a
        chip rate of `fs` or more has no representation at all: the step word
        would need bit `code_frac_bits`, and masking it off -- which is what this
        did while the rate was a constant that could never reach it -- turns
        10.23 Mcps at fs = 4.092 MHz into 0.5 chips/sample, a channel that
        correlates a plausible-looking nothing. Raise instead. The gateware
        makes the same refusal on its side (the CSR is one bit wider than the
        fraction, and an out-of-range word sets `rate_error` and stops the
        channel's dumps), so neither end can truncate silently.
        """
        rate = self.chip_rate if chip_rate is None else float(chip_rate)
        # Code rate scales with carrier Doppler: fc = chip_rate*(1 + fd/f_carrier).
        fc   = rate * (1.0 + doppler_hz / self.carrier_freq)
        word = round(fc / self.fs * (1 << self.fb))
        if not 0 < word < (1 << self.fb):
            raise ValueError(
                f"code rate {fc:.6g} chips/s is not representable at "
                f"fs = {self.fs:.6g} Hz: {fc / self.fs:.6g} chips/sample, and the "
                f"code NCO supports 2**-{self.fb} .. just under 1 chip/sample "
                f"(i.e. 0 < f_chip < fs). Sample faster, or track a slower code.")
        return word

    def set_carrier_hz(self, hz):
        self.csr.write(self.p + "carrier_freq", self.carrier_word(hz))

    def set_code_doppler(self, doppler_hz):
        self.csr.write(self.p + "code_freq", self.code_word(doppler_hz))

    def set_code_length(self, chips=None):
        """Stage the primary-code length; the next restart commits it.

        Staged rather than applied, so the length, the code and the code phase
        of a re-assignment all take effect on the same sample. Read
        `code_length_active` afterwards to confirm the commit.
        """
        if chips is not None:
            self.code_length = int(chips)
        if self.code_length < 1:
            raise ValueError(f"code length {self.code_length} must be >= 1")
        self.csr.write(self.p + "code_length", self.code_length)

    def code_length_active(self):
        """Primary-code length actually in force (as of the last restart)."""
        return self.csr.read(self.p + "code_length_active")

    def code_status(self):
        """(loading, rate_unsupported, replica_unsupported) right now."""
        v = self.csr.read(self.p + "code_status")
        return bool(v & 0b001), bool(v & 0b010), bool(v & 0b100)

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
        """E/L half-spacing, in chips: `sample_shift` whole NCO samples.

        `sample_shift * code_step` puts the Early tap exactly that many samples
        ahead of the prompt (and Late that many behind) with no rounding drift
        between the two fixed-point words -- programming the raw preferred shift
        instead leaves the accumulators at a spacing `dll_disc` does not assume
        (~2.3 % DLL loop-gain error at fs = 4 MHz, 0.5 chips). The taps only
        reach chip index +/- 1, so the result must stay below one chip.
        """
        step = self.code_word(code_doppler_hz)
        word = self.sample_shift(spacing_chips, code_doppler_hz) * step
        if word >= (1 << self.fb):
            raise ValueError(
                f"E/L spacing {word / (1 << self.fb):.3f} chips >= 1 chip: the "
                f"taps only reach chip index +/-1 (preferred {spacing_chips} "
                f"chips at fs={self.fs:.0f} Hz)")
        return word

    def tap_offset_word(self, sample_shift, code_doppler_hz=0.0):
        """Tap-offset CSR word for a tap `sample_shift` input samples early.

        The register is `code_frac_bits + 1` bits of two's complement, so a late
        tap (negative shift) is the wrapped value. Whole samples times the code
        step is the grid Tracking quantises its preferred shifts onto, and the
        five-tap discriminators read the VE/VL distance straight off the
        correlator they are handed, so this is the only representation that
        cannot mis-scale the loop.
        """
        step = self.code_word(code_doppler_hz)
        word = int(sample_shift) * step
        if abs(word) >= (1 << self.fb):
            raise ValueError(
                f"tap offset {word / (1 << self.fb):.3f} chips is a whole chip or "
                f"more: the taps reach chip index +/-1 only "
                f"(max_tap_offset_chips = {MAX_TAP_OFFSET_CHIPS})")
        return word & ((1 << (self.fb + 1)) - 1)

    def set_tap_offsets(self, sample_shifts, code_doppler_hz=0.0):
        """Program every tap from GNSSReceiver's `tap_sample_shifts`.

        `sample_shifts` is the contract's array: whole input samples, **latest
        first**, prompt at zero -- `[-s, 0, s]` for three taps, `[-s2, -s1, 0,
        s1, s2]` for five. The gateware's tap registers run earliest first, so
        the list is reversed here; the prompt entry must be 0 and has no
        register (the contract fixes it, and a register would only be a way to
        get it wrong).
        """
        shifts = list(sample_shifts)
        names  = tap_short_names(len(shifts))
        prompt = len(shifts) // 2
        if shifts[prompt] != 0:
            raise ValueError(
                f"tap shifts {shifts} must carry the prompt (0) at index "
                f"{prompt}; they are ordered latest first")
        for name, shift in zip(names, reversed(shifts)):
            if name == "p":
                continue
            self.csr.write(self.p + "tap_offset_" + name,
                           self.tap_offset_word(shift, code_doppler_hz))

    def set_spacing_chips(self, d, code_doppler_hz=0.0):
        """Symmetric E/L convenience: place Early/Late `d` chips either side.

        A shortcut for the common three-tap case, kept on the host where a
        convenience belongs -- the gateware has one register per tap and no
        notion of "the spacing".
        """
        word  = self.spacing_word(d, code_doppler_hz)
        mask  = (1 << (self.fb + 1)) - 1
        self.csr.write(self.p + "tap_offset_e", word)
        self.csr.write(self.p + "tap_offset_l", (-word) & mask)

    def set_prn(self, prn):
        self.csr.write(self.p + "prn", prn)

    def load_code(self, code, prn=None, select=None):
        """Load a primary code into the channel code RAM.

        `code` is the chips as 0/1 (GNSSSignals' -1/+1 maps 1 -> 1, -1 -> 0), or
        a PRN number, which is taken as GPS L1 C/A for backwards compatibility.
        The load also stages `code_length` from the sequence, so the code and its
        length can never disagree; both are committed by the next `restart()`,
        and the channel produces no records in between (`code_status().loading`).
        """
        if isinstance(code, int):
            prn, code = code, ca_code_reference(code)
        code = list(code)
        if select is None:
            select = [0] * len(code)
        elif len(select) != len(code):
            raise ValueError(
                f"{len(select)} subcarrier-select bits for {len(code)} chips: the "
                f"select bit is stored beside its chip, so there is exactly one "
                f"per chip")
        self.csr.write(self.p + "code_load", 0b100)          # reset_addr -> arm
        for chip, sub in zip(code, select):
            # bit0 = dat, bit1 = we, bit3 = subcarrier-table select.
            self.csr.write(self.p + "code_load",
                           0b010 | (chip & 1) | ((sub & 1) << 3))
        self.set_code_length(len(code))
        if prn is not None:
            self.set_prn(prn)

    def load_ca_code(self, prn):
        """Load GPS L1 C/A PRN `prn` (1023 chips)."""
        self.load_code(ca_code_reference(prn), prn=prn)

    # ---- sub-chip replica shape ---------------------------------------------
    def load_replica_shape(self, shape, num_taps=None):
        """Write the subcarrier table and stage the replica shape.

        `shape` is a `gnss_m2sdr.subcarrier.ReplicaShape` (or a GNSSSignals
        signal name, e.g. "GalileoE1B"). The table write raises the channel's
        `loading` bit, so the next `restart()` is what commits the shape *and*
        lets records flow again -- an amplitude that changed under an
        integration would be one record of two different replicas.

        This programs the replica GNSSSignals models for that signal, at the
        amplitudes its own table uses, so the host's default
        `replica_code_amplitude` is already right. It never substitutes a
        sign-only stand-in for CBOC: ask for "GalileoE1B_BOC11" if that is what
        you want, and tell the receiver so, because it is a different signal
        with ~0.45 dB less correlation.
        """
        if isinstance(shape, str):
            shape = signal_replica_shape(shape)
        for sel, lut in ((0, shape.lut_a), (1, shape.lut_b)):
            if lut is None:
                continue
            for adr, val in enumerate(lut):
                self.write_subcarrier(adr, val, lut=sel)
        if num_taps is None:
            num_taps = TAPS_EPL if shape.kind == "LOC" else TAPS_VEPL
        self.set_replica(subchips=shape.subchips, num_taps=num_taps)
        return shape

    def write_subcarrier(self, adr, value, lut=0):
        """One entry of the subcarrier table (signed amplitude)."""
        bits = self.replica_bits()
        if not -(1 << (bits - 1)) <= value < (1 << (bits - 1)):
            raise ValueError(
                f"subcarrier amplitude {value} does not fit the gateware's "
                f"{bits}-bit signed table entry")
        dat = value & ((1 << bits) - 1)
        self.csr.write(self.p + "subcarrier_load",
                       dat | (adr << bits) | (lut << (bits + self._sub_adr_bits()))
                       | (1 << (bits + self._sub_adr_bits() + 1)))

    def set_replica(self, subchips=1, num_taps=TAPS_EPL):
        """Stage the replica shape; the next restart commits it."""
        if num_taps not in (TAPS_EPL, TAPS_VEPL):
            raise ValueError(f"num_taps must be 3 or 5, got {num_taps}")
        if num_taps == TAPS_VEPL and not self._has_five_taps():
            raise ValueError(
                "this gateware build produces three taps; a five-tap layout "
                "cannot be configured on it (rebuild with --taps 5)")
        word = int(subchips)
        if self._has_five_taps() and num_taps == TAPS_VEPL:
            word |= 1 << self._subchips_bits()
        self.csr.write(self.p + "replica", word)

    def restart(self):
        # Edge-triggered: 0 -> 1 pulses restart + carrier_set (both bits).
        self.csr.write(self.p + "control", 0)
        self.csr.write(self.p + "control", 0b11)
        self.csr.write(self.p + "control", 0)

    # ---- scheduled updates (deterministic apply point) -----------------------
    def carrier_phase_word(self, cycles):
        return round((cycles % 1.0) * (1 << self.pb)) & ((1 << self.pb) - 1)

    def code_phase_word(self, chips, code_length=None):
        """Code phase (chips, fractional) -> the chip|frac word of code_phase."""
        n     = self.code_length if code_length is None else int(code_length)
        phase = chips % n
        chip  = int(phase)
        frac  = round((phase - chip) * (1 << self.fb))
        if frac == (1 << self.fb):                     # rounding carried a chip
            chip, frac = (chip + 1) % n, 0
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

    def configure(self, prn, carrier_hz, code_doppler_hz=0.0, spacing=0.5,
                  code=None, select=None, shape=None, tap_sample_shifts=None):
        """Arm this channel: load the replica, set the NCOs, then restart.

        `restart()` last is not cosmetic -- it is the commit. The code load, the
        subcarrier table, the staged `code_length`, the replica shape and the
        code phase all land on it together, and the channel emits no records
        until it happens.

        `shape` is a `gnss_m2sdr.subcarrier.ReplicaShape` (or a GNSSSignals
        signal name) for a BOC-family channel; leaving it out is plain BPSK, as
        before. `tap_sample_shifts` is GNSSReceiver's array -- whole input
        samples, latest first, prompt at zero -- and takes precedence over the
        symmetric `spacing` shortcut.
        """
        if isinstance(shape, str):
            shape = signal_replica_shape(shape)
        if shape is not None and select is None:
            select = shape.select
        self.load_code(prn if code is None else code, prn=prn, select=select)
        if shape is not None:
            self.load_replica_shape(
                shape,
                num_taps=(len(tap_sample_shifts) if tap_sample_shifts
                          else (TAPS_EPL if shape.kind == "LOC" else TAPS_VEPL)))
        if tap_sample_shifts:
            self.set_tap_offsets(tap_sample_shifts, code_doppler_hz)
        else:
            self.set_spacing_chips(spacing, code_doppler_hz)
        self.set_carrier_hz(carrier_hz)
        self.set_code_doppler(code_doppler_hz)
        self.restart()

    # ---- readback ------------------------------------------------------------
    def num_ants(self):
        """Antennas this channel reports, discovered from the CSR set.

        A 2-antenna build adds a suffixed register set (ip_ant1, ...); antenna 0
        keeps the bare names, so this works against either gateware.
        """
        if self._num_ants is None:
            n = 1
            while f"{self.p}ip_ant{n}" in self.csr.regs:
                n += 1
            self._num_ants = n
        return self._num_ants

    def read_dump(self):
        r = self.csr.read
        rs = lambda n: self.csr.read_signed(self.p + n, 32)
        # One E/P/L set per antenna; the replicas are shared, so there is a
        # single integrated_samples / sample_index / code_phase for all of them.
        # Antenna 0 is also spliced in flat, exactly as in the DMA record.
        # Taps the latched dump actually reports; the VE/VL registers of a
        # three-tap dump hold whatever the accumulators happened to contain, so
        # they are left out rather than reported as correlator values.
        ntaps = (r(self.p + "dump_num_taps")
                 if f"{self.p}dump_num_taps" in self.csr.regs else TAPS_EPL)
        ants = []
        for a in range(self.num_ants()):
            s = "" if a == 0 else f"_ant{a}"
            block = dict(ip=rs("ip" + s), qp=rs("qp" + s),
                         ie=rs("ie" + s), qe=rs("qe" + s),
                         il=rs("il" + s), ql=rs("ql" + s))
            if ntaps >= TAPS_VEPL:
                block.update(ive=rs("ive" + s), qve=rs("qve" + s),
                             ivl=rs("ivl" + s), qvl=rs("qvl" + s))
            ants.append(block)
        return dict(
            count = r(self.p + "dump_count"),
            ants = ants,
            n  = r(self.p + "integrated_samples"),
            sample_index = r(self.p + "sample_index"),
            code_phase   = r(self.p + "dump_code_phase"),
            code_chip    = r(self.p + "dump_code_chip"),
            num_taps     = ntaps,
            **ants[0],
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


def peak_code_phase(dump, frac_bits, code_length=None):
    """Code phase (chips) of the incoming signal at `dump["sample_index"]`.

    The replica's own phase on the last sample of the integration, read straight
    off the dump: `code_chip` (integer chips) plus `code_phase` (the fraction).
    On the peak dump the replica is -- to within the correlator's resolution --
    aligned with the signal, so that is the signal's code phase too, and
    `sample_index` is what pins it to the bank's global sample axis.

    The gateware used to report only the fraction, and the chip was *inferred*:
    a dump fires on the wrap, so the chip "must be" `code_length - 1`, i.e. 1022.
    That is wrong for every non-1023 code and for any dump shorter than a
    primary period, so the chip is now reported. `code_length` is accepted only
    for the fallback below, and is not needed otherwise.
    """
    if "code_chip" in dump:
        chip = dump["code_chip"]
    elif code_length is not None:
        chip = code_length - 1       # pre-v2 gateware: the old inference
    else:
        raise KeyError(
            "dump carries no code_chip and no code_length was given: the integer "
            "chip index cannot be inferred (see record_format.py)")
    return chip + dump["code_phase"] / float(1 << frac_bits)


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
    code_length  : primary-code chips the channel was configured for, and the
                   modulus the propagated phase wraps on.
    chip_rate    : nominal chipping rate (Hz) and
    carrier_freq : nominal carrier (Hz) -- together they turn the acquired
                   Doppler into a code rate. They travel with the result because
                   a code phase whose rate has to be guessed is not a handover.
    """
    metric:       float
    doppler_hz:   float
    power:        float
    code_phase:   Optional[float]
    sample_index: Optional[int]
    detected:     bool
    code_length:  int   = CA_CODE_LENGTH
    chip_rate:    float = GPS_CA_CHIP_RATE
    carrier_freq: float = GPS_L1_HZ

    def code_phase_at(self, sample_index, fs, code_length=None, chip_rate=None,
                      carrier_freq=None):
        """Propagate the acquired code phase to another global sample index.

        The handover arithmetic: the code runs on at the acquired Doppler, so a
        channel started at `sample_index` must begin at this phase. The signal's
        own length and rates are used unless overridden.
        """
        n    = self.code_length  if code_length  is None else code_length
        rate = self.chip_rate    if chip_rate    is None else chip_rate
        f0   = self.carrier_freq if carrier_freq is None else carrier_freq
        fc   = rate * (1.0 + self.doppler_hz / f0)
        return (self.code_phase
                + (sample_index - self.sample_index) * fc / fs) % n


class GNSSBank:
    def __init__(self, csr):
        self.csr = csr

    def channel(self, index, fs, **kwargs):
        """A `GNSSChannel` already told what this build can do.

        The tap count, sub-chip depth and table width come off the capability
        CSRs rather than from defaults, so the channel refuses a five-tap layout
        on a three-tap build (and sizes the subcarrier write port correctly)
        instead of writing a register that is not there.
        """
        caps = self.capabilities()
        kwargs.setdefault("carrier_phase_bits", caps["carrier_phase_bits"])
        kwargs.setdefault("code_frac_bits", caps["code_frac_bits"])
        kwargs.setdefault("num_taps", caps["num_taps"])
        kwargs.setdefault("max_subchips", max(1, caps["max_subchips"]))
        kwargs.setdefault("replica_bits", max(2, caps["replica_bits"]))
        return GNSSChannel(self.csr, fs, index=index, **kwargs)

    def enable(self, on=True):
        self.csr.write("gnss_control", 1 if on else 0)

    def overflow(self):
        """Sticky per-slot overflow mask: a set bit means that slot lost at
        least one record since the last clear_overflow(). Sticky, so a slow
        poller cannot miss it (the per-record FLAG_OVERFLOW marks *where*).

        Bit i is channel i; bit n_channels is the epoch strobe (see
        strobe_overflow()).
        """
        return self.csr.read("gnss_overflow")

    def dropped(self, index):
        """Saturating count of dumps lost on channel `index` since the last clear."""
        return self.csr.read(f"gnss_dropped{index}")

    def set_epoch_period(self, samples):
        """Emit a timebase-marker record every `samples` input samples (0 = off).

        Set this to the host's epoch length so an epoch boundary is delimited
        even when no channel dumps -- with correlator dumps as the only trigger
        a receiver with nothing locked never closes an epoch at all. Markers
        arrive as records with channel == STROBE_CHANNEL (see record_format.py).
        """
        self.csr.write("gnss_epoch_period", samples)

    def strobe_overflow(self, n_channels):
        """True if an epoch strobe was dropped: bit n_channels of overflow()."""
        return bool(self.overflow() & (1 << n_channels))

    def dropped_strobe(self):
        """Saturating count of epoch strobes lost since the last clear."""
        return self.csr.read("gnss_droppedstrobe")

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

    def rate_error(self):
        """Sticky per-channel mask: that channel was programmed a code rate the
        NCO cannot represent (>= 1 chip/input sample), so its dumps were
        suppressed rather than produced at a truncated rate. Cleared by that
        channel's restart."""
        return self.csr.read("gnss_rate_error")

    def version(self):
        """(csr_layout, record_format) revisions this gateware implements."""
        v = self.csr.read("gnss_version")
        return v & 0xFF, (v >> 8) & 0xFF

    def capabilities(self, fs=None):
        """What this gateware can do, read from its own CSRs.

        This is the discovery step an adapter builds GNSSReceiver's
        `HardwareCorrelatorCapabilities` from: nothing here is a constant of the
        driver, so a rebuilt bitstream with deeper code memory or more channels
        is described correctly without touching the host. `fs` fills in the code
        rate limits, which only exist relative to the sample rate.

        Raises if the gateware's CSR layout is newer than this driver: an
        unknown layout read as if it were this one is an over-declared
        capability, i.e. a channel that arms and never locks.
        """
        csr_version, record_version = self.version()
        if csr_version > CSR_LAYOUT_VERSION:
            raise RuntimeError(
                f"gateware CSR layout v{csr_version} is newer than this driver "
                f"(v{CSR_LAYOUT_VERSION}); update the host rather than guessing "
                f"the register set")
        if record_version > RECORD_FORMAT_VERSION:
            raise RuntimeError(
                f"gateware streams record format v{record_version}, newer than "
                f"this driver's v{RECORD_FORMAT_VERSION}")
        caps = self.csr.read("gnss_capabilities")
        sig  = self.csr.read("gnss_signal_caps")

        def field(value, shift, width):
            return (value >> shift) & ((1 << width) - 1)

        code_frac_bits = field(caps, 24, 8)
        out = dict(
            csr_version        = csr_version,
            record_version     = record_version,
            n_channels         = field(caps, 0, 8),
            num_ants_max       = field(caps, 8, 8),
            num_taps           = field(caps, 16, 8),
            code_frac_bits     = code_frac_bits,
            carrier_phase_bits = field(caps, 32, 8),
            accum_bits         = field(caps, 40, 8),
            max_code_length    = field(caps, 48, 16),
            modulations        = field(sig, 0, 8),
            max_secondary_code_length = field(sig, 8, 8),
            reports_code_phase = bool(field(sig, 16, 1)),
            tap_layouts        = [n for i, n in enumerate((TAPS_EPL, TAPS_VEPL))
                                  if field(sig, 17, 4) & (1 << i)],
            max_subchips       = field(sig, 21, 8),
            replica_bits       = field(sig, 29, 8),
            max_tap_offset_chips = MAX_TAP_OFFSET_CHIPS,
        )
        if fs is not None:
            # The code NCO steps 1..2**code_frac_bits - 1 in 2**-code_frac_bits
            # chips per input sample, so the representable chip rates are a
            # property of fs, not of the gateware alone.
            scale = float(fs) / (1 << code_frac_bits)
            out["code_frequency_limits"] = (scale, scale * ((1 << code_frac_bits) - 1))
        return out


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
    signal = dict(code_length=chan.code_length, chip_rate=chan.chip_rate,
                  carrier_freq=chan.carrier_freq)
    bank.enable(True)
    off = round(slide_chips / fs * (1 << chan.fb))
    best = AcquisitionResult(0.0, 0.0, 0.0, None, None, False, **signal)
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
                                         peak["sample_index"], False, **signal)
        d += doppler_step
    return best._replace(detected=best.metric >= detect_metric)
