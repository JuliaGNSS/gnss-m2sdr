#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Bit-exact reference model of the TDM correlator engine.

Integer arithmetic only, in the gateware's widths, so a simulation of the
gateware (Migen, or the LiteX-lowered Verilog in xsim) can be compared record by
record. The model is also the specification: the order of the operations below
is the order of the pipeline stages in ``engine.py``.
"""

from dataclasses import dataclass, field

from gnss_m2sdr.tdm import params as P

M32 = (1 << 32) - 1


def _signed(value, bits):
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >> (bits - 1) else value


def _saturate(value, bits):
    hi = (1 << (bits - 1)) - 1
    lo = -(1 << (bits - 1))
    if value > hi:
        return hi, True
    if value < lo:
        return lo, True
    return value, False


@dataclass
class ChannelConfig:
    carrier_freq: int = 0          # signed, cycles/sample x 2^32
    code_freq: int = 0             # unsigned, chips/sample x 2^32
    code_length: int = 1023
    code_base: int = 0             # code-memory word address
    lead: int = 0
    offsets: tuple = (0, 0, 0, 0, 0)
    num_taps: int = 3
    warmup: int = 0
    antenna: int = 0
    assign_id: int = 0
    subcarrier: int = P.SUBCARRIER_BPSK    # s_subcarrier: sub-chips, signs, levels
    amplitude: int = P.AMPLITUDE_UNIT      # s_amplitude: amp_hi, amp_lo

    def subchips(self):
        return (self.subcarrier >> (2 * P.MAX_SUBCHIPS)) & ((1 << P.SUBCHIP_BITS) - 1)

    def amp(self, level):
        return (self.amplitude >> (8 if not level else 0)) & ((1 << P.AMP_BITS) - 1)


@dataclass
class Pending:
    start: int = 0
    carrier_phase: int = 0
    code_chip: int = 0
    code_frac: int = 0


@dataclass
class ChannelState:
    carrier_phase: int = 0
    code_chip: int = 0
    code_frac: int = 0
    hist_rep: int = 0
    hist_wrap: int = 0
    hist_lvl: int = 0
    integ: int = 0
    warm: int = 0
    sat: bool = False
    lost: bool = False
    acc: list = field(default_factory=lambda: [[0, 0] for _ in range(P.MAX_TAPS)])


@dataclass
class Record:
    kind: int
    channel: int = 0
    num_taps: int = 0
    antenna: int = 0
    flags: int = 0
    assign_id: int = 0
    sample_index: int = 0
    integrated: int = 0
    code_chip: int = 0
    code_frac20: int = 0
    taps: tuple = ((0, 0),) * P.MAX_TAPS   # ((I, Q), ...) for every MAX_TAPS slot

    def words(self):
        """The record as the 8 little-endian 64-bit words the gateware emits."""
        if self.kind == P.KIND_PAD:
            return [0] * P.RECORD_WORDS
        w0 = (P.RECORD_MAGIC
              | P.RECORD_VERSION << 16
              | (self.kind & 0xF) << 20
              | (self.channel & 0xFF) << 24
              | (self.num_taps & 0x7) << 32
              | (self.antenna & 0x1) << 35
              | (self.flags & 0x3F) << 36
              | (self.assign_id & 0xFFFF) << 48)
        w1 = self.sample_index & ((1 << 48) - 1)
        w2 = ((self.integrated & ((1 << 24) - 1))
              | (self.code_chip & ((1 << 20) - 1)) << 24
              | (self.code_frac20 & ((1 << 20) - 1)) << 44)
        taps = list(self.taps) + [(0, 0)] * (P.MAX_TAPS - len(self.taps))
        tw = [(i & M32) | (q & M32) << 32 for i, q in taps]
        return [w0, w1, w2] + tw


def parse_words(words):
    """Inverse of :meth:`Record.words` (a KIND_PAD block for an all-zero one)."""
    w0, w1, w2 = words[0], words[1], words[2]
    if w0 & 0xFFFF != P.RECORD_MAGIC:
        return Record(kind=P.KIND_PAD)
    taps = tuple((_signed(w, 32), _signed(w >> 32, 32)) for w in words[3:8])
    return Record(
        kind=(w0 >> 20) & 0xF,
        channel=(w0 >> 24) & 0xFF,
        num_taps=(w0 >> 32) & 0x7,
        antenna=(w0 >> 35) & 0x1,
        flags=(w0 >> 36) & 0x3F,
        assign_id=(w0 >> 48) & 0xFFFF,
        sample_index=w1 & ((1 << 48) - 1),
        integrated=w2 & ((1 << 24) - 1),
        code_chip=(w2 >> 24) & ((1 << 20) - 1),
        code_frac20=(w2 >> 44) & ((1 << 20) - 1),
        taps=taps,
    )


class Engine:
    """The engine's channels, flags and code memory, stepped one sample at a time."""

    def __init__(self, n_channels, code_words, strobe_period=0):
        self.n = n_channels
        self.code = [0] * code_words
        self.cfg = [ChannelConfig() for _ in range(n_channels)]
        self.pending = [Pending() for _ in range(n_channels)]
        self.state = [ChannelState() for _ in range(n_channels)]
        self.enabled = [False] * n_channels
        self.pend = [False] * n_channels
        self.missed = [False] * n_channels
        self.last_n = None                 # the previous sample processed
        self.set_strobe_period(strobe_period)
        self.cos_t, self.sin_t = P.carrier_table()

    # Host commands ---------------------------------------------------------------------------

    def write_code(self, address, words):
        for k, w in enumerate(words):
            self.code[address + k] = w & M32

    def word(self, ch, carrier_freq, code_freq):
        self.cfg[ch].carrier_freq = _signed(carrier_freq, 32)
        self.cfg[ch].code_freq = code_freq & M32

    def arm(self, ch, cfg: ChannelConfig, pending: Pending):
        self.cfg[ch] = cfg
        self.pending[ch] = pending
        self.pend[ch] = True
        self.enabled[ch] = False
        self.missed[ch] = False

    def release(self, ch):
        self.enabled[ch] = False
        self.pend[ch] = False

    def set_strobe_period(self, period):
        """A write of the strobe period (and the state after a reset): the
        strobes re-find the period's phase from the next sample."""
        self.strobe_period = period & M32
        self.strobe_sync = True
        self.strobe_hold = 0
        self.strobe_next = 0

    # One sample ------------------------------------------------------------------------------

    def step(self, n, samples):
        """Process sample ``n``; ``samples`` is ((I0, Q0), (I1, Q1)) for the two
        antennas. Returns the records it produced, in emission order."""
        out = []
        gap = self.last_n is not None and n != self.last_n + 1
        self.last_n = n
        for ch in range(self.n):
            rec = self._channel(ch, n, samples, gap)
            if rec is not None:
                out.append(rec)
        period = self.strobe_period
        if period:
            if self.strobe_sync:
                self.strobe_next = P.strobe_next(n + 1 + P.STROBE_SYNC_SAMPLES, period)
                self.strobe_hold = P.STROBE_SYNC_SAMPLES - 1
                self.strobe_sync = False
            elif self.strobe_hold:
                self.strobe_hold -= 1
            elif n + 1 >= self.strobe_next:
                out.append(Record(kind=P.KIND_STROBE, sample_index=n + 1))
                self.strobe_next += period
                if self.strobe_next <= n + 1:
                    self.strobe_sync = True
        return out

    def _channel(self, ch, n, samples, gap):
        cfg, st = self.cfg[ch], self.state[ch]
        if self.pend[ch]:
            p = self.pending[ch]
            if n == p.start:
                st.carrier_phase = p.carrier_phase & M32
                st.code_chip = p.code_chip
                st.code_frac = p.code_frac & M32
                st.hist_rep = st.hist_wrap = st.hist_lvl = 0
                st.integ = 0
                st.warm = cfg.warmup
                st.sat = False
                st.lost = False
                st.acc = [[0, 0] for _ in range(P.MAX_TAPS)]
                gap = False                # samples lost before the start do not matter
                self.enabled[ch] = True
                self.pend[ch] = False
            elif n > p.start:
                self.missed[ch] = True
                self.pend[ch] = False
        if not self.enabled[ch]:
            return None
        st.lost |= gap

        # Carrier wipe-off: sample x exp(-j phase).
        k = st.carrier_phase >> (P.PHASE_BITS - P.CARRIER_LUT_BITS)
        c, s = self.cos_t[k], self.sin_t[k]
        i, q = samples[cfg.antenna]
        bb_i = i * c + q * s
        bb_q = q * c - i * s
        st.carrier_phase = (st.carrier_phase + cfg.carrier_freq) & M32

        # Code NCO (earliest tap) and the replica history: the chip's sign times
        # the sub-chip's sign, and the sub-chip's level.
        word = self.code[(cfg.code_base + (st.code_chip >> 5)) % len(self.code)]
        sub = P.subchip_index(st.code_frac, cfg.subchips())
        bit = ((word >> (st.code_chip & 31)) ^ (cfg.subcarrier >> sub)) & 1
        level = (cfg.subcarrier >> (P.MAX_SUBCHIPS + sub)) & 1
        total = st.code_frac + cfg.code_freq
        st.code_frac = total & M32
        chip = st.code_chip + (total >> 32)
        wrap = chip == cfg.code_length
        st.code_chip = 0 if wrap else chip
        st.hist_rep = ((st.hist_rep << 1) | bit) & M32
        st.hist_wrap = ((st.hist_wrap << 1) | int(wrap)) & M32
        st.hist_lvl = ((st.hist_lvl << 1) | level) & M32
        prompt_wrap = (st.hist_wrap >> cfg.lead) & 1

        if st.warm > 0:
            st.warm -= 1
            return None
        for t in range(cfg.num_taps):
            negative = (st.hist_rep >> cfg.offsets[t]) & 1
            amp = cfg.amp((st.hist_lvl >> cfg.offsets[t]) & 1)
            for part, bb in ((0, bb_i), (1, bb_q)):
                v = amp * bb
                value, clipped = _saturate(st.acc[t][part] + (-v if negative else v), P.ACC_BITS)
                st.acc[t][part] = value
                st.sat |= clipped
        st.integ += 1
        if not prompt_wrap:
            return None
        rec = Record(
            kind=P.KIND_DUMP,
            channel=ch,
            num_taps=cfg.num_taps,
            antenna=cfg.antenna,
            flags=(P.FLAG_SATURATED if st.sat else 0) | (P.FLAG_SAMPLE_OVERFLOW if st.lost else 0),
            assign_id=cfg.assign_id,
            sample_index=n + 1,
            integrated=st.integ,
            code_chip=st.code_chip,
            code_frac20=st.code_frac >> 12,
            taps=tuple(tuple(a) for a in st.acc),
        )
        st.acc = [[0, 0] for _ in range(P.MAX_TAPS)]
        st.integ = 0
        st.sat = False
        st.lost = False
        return rec


# Host-side conversion helpers -----------------------------------------------------------------------

def carrier_word(freq_hz, fs):
    return int(round(freq_hz / fs * (1 << 32))) & M32


def code_word(rate_chips_per_s, fs):
    w = int(round(rate_chips_per_s / fs * (1 << 32)))
    if not 0 < w < (1 << 32):
        raise ValueError("code rate must be below one chip per sample")
    return w


def pack_code(chips):
    """±1 chips -> 32-bit code-memory words (bit set = -1), chip k at bit k % 32."""
    words = [0] * ((len(chips) + 31) // 32)
    for k, c in enumerate(chips):
        if c < 0:
            words[k >> 5] |= 1 << (k & 31)
    return words
