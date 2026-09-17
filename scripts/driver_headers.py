#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

"""Turn a LiteX-generated csr.h into one the M2SDR driver build can compile.

`docs/hardware_bringup.md` used to say "copy the generated {csr,soc,mem}.h into
the driver", and that is wrong in a way that only shows up after `make clean`
has already deleted the working binaries: LiteX's `csr.h` is written for the
SoC's *own* bare-metal firmware, so it opens with

    #include <generated/soc.h>
    #include <system.h>
    #include <hw/common.h>

none of which exist in `litex_m2sdr/software/{kernel,user}`. The kernel module
then fails with `fatal error: generated/soc.h: No such file or directory`, and
the user tools with `fatal error: system.h`.

The driver only wants the `CSR_*` address macros. This strips those three
includes and leaves everything else byte for byte, which is what the header
that shipped with the previous release actually contains.

    python3 scripts/driver_headers.py build/<name>/software/include/generated/csr.h \\
        -o csr.h
"""

import argparse
import re
import sys

# Includes that only exist inside a LiteX bare-metal firmware build.
FIRMWARE_ONLY = ("generated/soc.h", "system.h", "hw/common.h")


def strip_firmware_includes(text):
    """Comment out the firmware-only #includes; leave the rest untouched."""
    out, removed = [], []
    for line in text.splitlines(keepends=True):
        m = re.match(r'\s*#\s*include\s*[<"]([^>"]+)[>"]', line)
        if m and m.group(1) in FIRMWARE_ONLY:
            removed.append(m.group(1))
            out.append("/* %s */ /* stripped: firmware-only, see "
                       "scripts/driver_headers.py */\n" % line.rstrip("\n"))
        else:
            out.append(line)
    return "".join(out), removed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csr_h", help="generated csr.h from a build")
    ap.add_argument("-o", "--output", help="where to write it (default: stdout)")
    args = ap.parse_args()

    text, removed = strip_firmware_includes(open(args.csr_h).read())
    if args.output:
        with open(args.output, "w") as fp:
            fp.write(text)
    else:
        sys.stdout.write(text)
    print("stripped: " + (", ".join(removed) or "nothing"), file=sys.stderr)
    # `#include <stdint.h>` must survive -- the CSR macros are typed.
    if "#include <stdint.h>" not in text:
        print("warning: no <stdint.h> in the header", file=sys.stderr)


if __name__ == "__main__":
    main()
