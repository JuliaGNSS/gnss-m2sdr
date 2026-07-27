#
# This file is part of gnss-m2sdr.
# SPDX-License-Identifier: BSD-2-Clause

# Importing the test package installs the migen simulator workaround, so
# `PYTHONPATH=. python -m unittest test.test_record` works on a stock migen.
from . import migen_compat  # noqa: F401
