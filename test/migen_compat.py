#
# This file is part of gnss-m2sdr.
#
# Simulation-only workaround for write-only Memory ports in migen's simulator.
# SPDX-License-Identifier: BSD-2-Clause

"""Make migen's simulator accept write-only Memory ports.

`migen.genlib.fifo.SyncFIFO` -- which LiteX's `stream.SyncFIFO` wraps, and hence
every simulation that instantiates `CorrelatorRecorder`'s output FIFO -- asks its
storage for a **write-only** port::

    storage.get_port(write_capable=True, read_capable=False, mode=READ_FIRST)

`Memory.get_port()` documents that as `dat_r = None`, and both Verilog emitters
handle it, but the simulator's memory lowering (`MemoryToArray` in
`migen.fhdl.simplify`, used only by `migen.sim` and `litex.gen.sim`) emits the
read unconditionally::

    rd_stmt = port.dat_r.eq(storage[port.adr])
    AttributeError: 'NoneType' object has no attribute 'eq'

So the simulation dies before its first cycle -- which is what the "Memory API
mismatch between migen and LiteX" in issue #13 actually was: not a version skew
but an upstream omission, still present in migen e19524c (the pinned version)
with LiteX 2026.4.

Giving those ports an unused `dat_r` right before the lowering runs is enough:
`MemoryToArray` then registers a read that nothing consumes, so the FIFO behaves
exactly as it does in hardware. The patch is scoped to `MemoryToArray`, so
gateware generation (`build.py`) never sees it, and it is idempotent -- on a
migen that skips write-only ports itself, the extra signal is simply unused.

Imported by `test/__init__.py` (and by `test/run_all.py`, which discovers test
modules as top-level names), so it is installed before any `run_simulation`.
Start the suite through `test/run_all.py` or `python -m unittest test.<module>`:
running a test file as a bare script (`python test/test_record.py`) bypasses the
package import, and with it this patch.
"""

from migen.fhdl.simplify import MemoryToArray
from migen.fhdl.specials import Memory
from migen.fhdl.structure import Signal

_MARKER = "_gnss_m2sdr_write_only_port_fix"


def _patch():
    original = MemoryToArray.transform_fragment
    if getattr(original, _MARKER, False):
        return

    def transform_fragment(self, i, f):
        for special in f.specials:
            if isinstance(special, Memory):
                for port in special.ports:
                    if port.dat_r is None:
                        port.dat_r = Signal(special.width, name="unused_dat_r")
        return original(self, i, f)

    setattr(transform_fragment, _MARKER, True)
    MemoryToArray.transform_fragment = transform_fragment


_patch()
