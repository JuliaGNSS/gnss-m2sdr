#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Pure-Python GPS L1 C/A code reference (no migen), usable on the host.

CA_PHASE_SELECT = {
     1: ( 2,  6),  2: ( 3,  7),  3: ( 4,  8),  4: ( 5,  9),  5: ( 1,  9),
     6: ( 2, 10),  7: ( 1,  8),  8: ( 2,  9),  9: ( 3, 10), 10: ( 2,  3),
    11: ( 3,  4), 12: ( 5,  6), 13: ( 6,  7), 14: ( 7,  8), 15: ( 8,  9),
    16: ( 9, 10), 17: ( 1,  4), 18: ( 2,  5), 19: ( 3,  6), 20: ( 4,  7),
    21: ( 5,  8), 22: ( 6,  9), 23: ( 1,  3), 24: ( 4,  6), 25: ( 5,  7),
    26: ( 6,  8), 27: ( 7,  9), 28: ( 8, 10), 29: ( 1,  6), 30: ( 2,  7),
    31: ( 3,  8), 32: ( 4,  9),
}

CA_CODE_LENGTH = 1023
GPS_L1_HZ      = 1575.42e6
GPS_CA_CHIP_RATE = 1.023e6


def ca_code_reference(prn):
    """Length-1023 C/A code for `prn` as a list of 0/1 chips."""
    s1, s2 = CA_PHASE_SELECT[prn]
    g1 = [1] * 10
    g2 = [1] * 10
    out = []
    for _ in range(CA_CODE_LENGTH):
        g1_out = g1[9]
        g2_out = g2[s1 - 1] ^ g2[s2 - 1]
        out.append(g1_out ^ g2_out)
        fb1 = g1[2] ^ g1[9]
        fb2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = [fb1] + g1[:9]
        g2 = [fb2] + g2[:9]
    return out


def ca_first10_octal(prn):
    chips = ca_code_reference(prn)[:10]
    value = 0
    for c in chips:
        value = (value << 1) | c
    return oct(value)
