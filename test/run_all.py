#!/usr/bin/env python3
#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause
#
# Discover and run all Migen/reference tests. Usage:
#   PYTHONPATH=. python test/run_all.py

import os
import sys
import unittest

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(here))
    # Discovery imports the modules as top-level names, so the package __init__
    # does not run: install the migen simulator workaround explicitly.
    import test.migen_compat  # noqa: F401
    suite = unittest.defaultTestLoader.discover(here, pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
