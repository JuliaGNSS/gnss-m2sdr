#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Time-multiplexed (TDM) GNSS correlator.

One pipelined datapath serves every channel: per input sample the engine steps
each channel slot once, reading the channel's state from a RAM, advancing its
carrier and code NCOs, correlating the taps and writing the state back. See
``docs/tdm_correlator.md``.
"""
