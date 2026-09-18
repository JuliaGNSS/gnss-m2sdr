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

# LiteX (since mid-2026) also opens csr.h with `#include <stdint.h>` for the
# typed accessor functions. The kernel module has no <stdint.h>: it gets the
# uintN_t typedefs from <linux/types.h>. The user tools have both.
STDINT_REPLACEMENT = (
    "#ifdef __KERNEL__\n"
    "#include <linux/types.h>   /* stdint.h, see scripts/driver_headers.py */\n"
    "#else\n"
    "#include <stdint.h>\n"
    "#endif\n"
)


def strip_firmware_includes(text):
    """Comment out the firmware-only #includes and drop the accessor functions.

    LiteX (since mid-2026) also writes one `static inline` read/write accessor
    per register into csr.h, calling `csr_read_simple` / `csr_write_simple`
    from the stripped hw/common.h. The driver never uses them and the kernel
    build rejects the implicit declarations, so every `static inline ... {`
    block is dropped up to its closing `}`. The `CSR_*` macros are untouched.
    """
    out, removed = [], []
    in_accessor, n_accessors = False, 0
    for line in text.splitlines(keepends=True):
        if in_accessor:
            if line.strip() == "}":
                in_accessor = False
            continue
        if line.startswith("static inline "):
            in_accessor = not line.rstrip().endswith("}")
            n_accessors += 1
            continue
        m = re.match(r'\s*#\s*include\s*[<"]([^>"]+)[>"]', line)
        if m and m.group(1) in FIRMWARE_ONLY:
            removed.append(m.group(1))
            out.append("/* %s */ /* stripped: firmware-only, see "
                       "scripts/driver_headers.py */\n" % line.rstrip("\n"))
        elif m and m.group(1) == "stdint.h":
            removed.append("stdint.h (kernel: linux/types.h)")
            out.append(STDINT_REPLACEMENT)
        else:
            out.append(line)
    if n_accessors:
        removed.append(f"{n_accessors} accessor functions")
    return "".join(out), removed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csr_h", help="generated csr.h (or soc.h / mem.h) from a build")
    ap.add_argument("-o", "--output", help="where to write it (default: stdout)")
    args = ap.parse_args()

    text, removed = strip_firmware_includes(open(args.csr_h).read())
    if args.output:
        with open(args.output, "w") as fp:
            fp.write(text)
    else:
        sys.stdout.write(text)
    print("stripped: " + (", ".join(removed) or "nothing"), file=sys.stderr)


if __name__ == "__main__":
    main()
