#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Shared test harness for the TDM correlator.

``TDMHarness`` wraps ``TDMCorrelator`` behind a minimal register-write port, so
one cycle-by-cycle stimulus drives both the Migen simulation and the
LiteX-lowered Verilog in Vivado's xsim. ``Scenario`` builds that stimulus and
runs the bit-exact model over the same samples and commands, giving the
records the gateware must produce.
"""

import math
import random

from migen import *

from litex.gen import *

from gnss_m2sdr.tdm import params as P
from gnss_m2sdr.tdm import model as M
from gnss_m2sdr.tdm.engine import TDMCorrelator


# Register map of the harness port (an index per CSR). -----------------------------------------------

REGS = [
    "control", "strobe_period", "flush_cycles", "buffer_words",
    "s_carrier_freq", "s_code_freq", "s_carrier_phase", "s_code_frac", "s_code_chip",
    "s_code_length", "s_code_base", "s_taps", "s_misc", "s_subcarrier", "s_amplitude",
    "s_start_lo", "s_start_hi",
    "cmd", "code_addr", "code_data",
]
REG = {name: k for k, name in enumerate(REGS)}


class TDMHarness(LiteXModule):
    def __init__(self, n_channels=4, code_words=256, sample_fifo_depth=64, record_fifo_depth=16):
        self.wr_valid = Signal()
        self.wr_addr  = Signal(8)
        self.wr_data  = Signal(32)
        self.sample_valid = Signal()
        self.sample_data  = Signal(64)
        self.sample_index = Signal(P.SAMPLE_INDEX_BITS)
        self.out_valid = Signal()
        self.out_data  = Signal(64)
        self.out_ready = Signal(reset=1)

        self.dut = dut = TDMCorrelator(n_channels, code_words, sample_fifo_depth, record_fifo_depth)
        for name in REGS:
            csr = getattr(dut, "_" + name)
            # In the SoC the CSR bank finalizes each CSR, which takes in the comb
            # statements wiring its fields; without a bank the harness has to.
            self.comb += csr._fragment.comb
            hit = Signal(name=f"hit_{name}")
            self.comb += hit.eq(self.wr_valid & (self.wr_addr == REG[name]))
            # As LiteX's CSR bank does: the storage takes the value on the write,
            # and the write strobe follows one cycle later.
            self.sync += [
                If(hit, csr.storage.eq(self.wr_data)),
                csr.re.eq(hit),
            ]
        self.comb += [
            dut.sample_valid.eq(self.sample_valid),
            dut.sample_data.eq(self.sample_data),
            dut.sample_index.eq(self.sample_index),
            dut.source.ready.eq(self.out_ready),
        ]
        # The words the DMA took at each edge, registered: what a simulator reads
        # after an edge is then exactly what transferred on it.
        self.sync += [
            self.out_valid.eq(dut.source.valid & self.out_ready),
            self.out_data.eq(dut.source.data),
        ]

    def ios(self):
        return {self.wr_valid, self.wr_addr, self.wr_data, self.sample_valid, self.sample_data,
                self.sample_index, self.out_valid, self.out_data, self.out_ready}


# Stimulus ----------------------------------------------------------------------------------------

def _u(v, bits=32):
    return v & ((1 << bits) - 1)


def pack_sample_word(samples):
    """((i0, q0), (i1, q1)) -> the 64-bit RX word (sign-extended 16-bit slots)."""
    (i0, q0), (i1, q1) = samples
    return _u(i0, 16) | _u(q0, 16) << 16 | _u(i1, 16) << 32 | _u(q1, 16) << 48


class Scenario:
    """A cycle-by-cycle stimulus plus the model run over the same inputs.

    Commands go through ``write``/``arm``/``word``/``release``; ``samples`` feeds
    a block of samples ``spacing`` cycles apart. Commands are only issued while
    the engine is idle (the stimulus waits ``drain`` cycles after each sample
    block), so the model applies them between the same two samples.
    """
    def __init__(self, n_channels=4, code_words=256, strobe_period=0, buffer_words=16,
                 flush_cycles=2000, spacing=16, drain=80):
        self.n_channels = n_channels
        self.code_words = code_words
        self.spacing = spacing
        self.drain = drain
        self.cycles = []                     # (wr_valid, addr, data, s_valid, s_word, s_index)
        self.model = M.Engine(n_channels, code_words, strobe_period)
        self.records = []                    # the model's records, in order
        self.next_index = 0
        self.assign = 0
        self.write("buffer_words", buffer_words)
        self.write("flush_cycles", flush_cycles)
        self.write("strobe_period", strobe_period)
        self.write("control", 0b011)         # enable + record_enable

    def idle(self, n=1):
        self.cycles += [(0, 0, 0, 0, 0, 0)] * n

    def write(self, reg, value):
        self.cycles.append((1, REG[reg], _u(value), 0, 0, 0))

    def code(self, address, words):
        self.write("code_addr", address)
        for w in words:
            self.write("code_data", w)
        self.model.write_code(address, words)

    def word(self, ch, carrier_freq, code_freq):
        self.write("s_carrier_freq", carrier_freq)
        self.write("s_code_freq", code_freq)
        self.write("cmd", P.CMD_WORD | ch << P.CMD_CHANNEL_SHIFT)
        self.model.word(ch, carrier_freq, code_freq)

    def arm(self, ch, *, carrier_freq, code_freq, carrier_phase, code_chip, code_frac,
            code_length, code_base, shifts, antenna, start,
            subcarrier=P.SUBCARRIER_BPSK, amplitude=P.AMPLITUDE_UNIT):
        lead, offsets, warmup = P.tap_offsets(shifts)
        offsets5 = list(offsets) + [0] * (P.MAX_TAPS - len(offsets))
        self.assign = (self.assign + 1) & 0xFFFF
        taps = sum(d << (5 * k) for k, d in enumerate(offsets5)) | len(shifts) << 25
        misc = lead | warmup << 5 | antenna << 10 | self.assign << 16
        for reg, v in (("s_carrier_freq", carrier_freq), ("s_code_freq", code_freq),
                       ("s_carrier_phase", carrier_phase), ("s_code_frac", code_frac),
                       ("s_code_chip", code_chip), ("s_code_length", code_length),
                       ("s_code_base", code_base), ("s_taps", taps), ("s_misc", misc),
                       ("s_subcarrier", subcarrier), ("s_amplitude", amplitude),
                       ("s_start_lo", start & 0xFFFFFFFF), ("s_start_hi", start >> 32)):
            self.write(reg, v)
        self.write("cmd", P.CMD_ARM | ch << P.CMD_CHANNEL_SHIFT)
        self.model.arm(ch,
            M.ChannelConfig(carrier_freq=M._signed(carrier_freq, 32), code_freq=code_freq,
                            code_length=code_length, code_base=code_base, lead=lead,
                            offsets=tuple(offsets5), num_taps=len(shifts), warmup=warmup,
                            antenna=antenna, assign_id=self.assign,
                            subcarrier=subcarrier, amplitude=amplitude),
            M.Pending(start=start, carrier_phase=carrier_phase, code_chip=code_chip, code_frac=code_frac))
        return self.assign

    def release(self, ch):
        self.write("cmd", P.CMD_RELEASE | ch << P.CMD_CHANNEL_SHIFT)
        self.model.release(ch)

    def strobe_period(self, period):
        self.write("strobe_period", period)
        self.model.set_strobe_period(period)

    def samples(self, block, spacing=None):
        """Feed a list of ((i0, q0), (i1, q1)) samples."""
        spacing = self.spacing if spacing is None else spacing
        for s in block:
            n = self.next_index
            self.cycles.append((0, 0, 0, 1, pack_sample_word(s), n))
            self.cycles += [(0, 0, 0, 0, 0, 0)] * (spacing - 1)
            self.records += self.model.step(n, s)
            self.next_index += 1
        self.idle(self.drain)

    def skip_samples(self, k):
        """Lose ``k`` sample indices (as a dropped word upstream would)."""
        self.next_index += k


# Signals ----------------------------------------------------------------------------------------

def gps_like_signal(n, *, fs, code_chips, code_rate, doppler, code_phase, amplitude, noise, seed=1,
                    antenna=1, other=None, subcarrier=None):
    """Two-antenna samples with a BPSK satellite on ``antenna``: ±1 ``code_chips``
    at ``code_rate`` chips/s from ``code_phase`` (chips), carrier ``doppler`` Hz.
    ``subcarrier(frac)``, if given, multiplies each chip by the value at the
    chip fraction ``frac`` (a BOC/CBOC satellite). ``other`` adds a second
    satellite (a dict of the same keys) to the same antenna."""
    rng = random.Random(seed)
    out = []
    sats = [dict(code_chips=code_chips, code_rate=code_rate, doppler=doppler,
                 code_phase=code_phase, amplitude=amplitude, subcarrier=subcarrier)]
    if other:
        sats.append(other)
    L = len(code_chips)
    for k in range(n):
        t = k / fs
        re, im = 0.0, 0.0
        for s in sats:
            phase = s["code_phase"] + s["code_rate"] * t
            chip = s["code_chips"][int(math.floor(phase)) % len(s["code_chips"])]
            if s.get("subcarrier") is not None:
                chip *= s["subcarrier"](phase % 1)
            ph = 2 * math.pi * s["doppler"] * t
            re += s["amplitude"] * chip * math.cos(ph)
            im += s["amplitude"] * chip * math.sin(ph)
        sig = (int(round(re + rng.gauss(0, noise))), int(round(im + rng.gauss(0, noise))))
        nse = (int(round(rng.gauss(0, noise))), int(round(rng.gauss(0, noise))))
        clip = lambda v: max(-2048, min(2047, v))
        sig = (clip(sig[0]), clip(sig[1]))
        out.append((nse, sig) if antenna == 1 else (sig, nse))
    return out


def cboc_subcarrier(m1=1, m2=6, p=10 / 11):
    """The CBOC(+) subcarrier as transmitted: sqrt(p) BOC(m1,1) + sqrt(1-p) BOC(m2,1)."""
    boc = lambda m, f: -1.0 if math.floor(f * 2 * m) % 2 else 1.0
    return lambda f: math.sqrt(p) * boc(m1, f) + math.sqrt(1 - p) * boc(m2, f)


def random_code(length, seed):
    rng = random.Random(seed)
    return [1 if rng.random() < 0.5 else -1 for _ in range(length)]


# Output parsing ---------------------------------------------------------------------------------

def parse_stream(words):
    """64-bit words -> records, skipping all-zero pad blocks. Words must be whole records."""
    assert len(words) % P.RECORD_WORDS == 0, f"{len(words)} words is not a whole number of records"
    out = []
    for k in range(0, len(words), P.RECORD_WORDS):
        rec = M.parse_words(words[k:k + P.RECORD_WORDS])
        if rec.kind != P.KIND_PAD:
            out.append(rec)
    return out
