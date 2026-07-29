# Verilog-level injection test

Correlates an injected GPS L1 C/A signal of exactly known PRN, code phase and
Doppler against the **emitted** RTL, under Vivado's `xsim`.

The rest of the suite runs migen's Python simulator, which evaluates the design
in exact Python integer arithmetic and never goes through the Verilog emitter or
a `$readmemh` data file — so it cannot see a bug that lives in the generated
text. Two such bugs stopped the on-FPGA correlators from correlating, both green
in simulation:

* the carrier sin/cos ROM was built from signed Python ints, so `-127` reached the
  `$readmemh` file as the token `-7F`; Vivado silently drops the minus and stores
  `+0x7F`, leaving the replica full-wave rectified.
* the saturating accumulator range-tested with `raw < -(2**31)`, which LiteX
  renders as `raw < -32'h80000000` — an unsigned literal, so Verilog evaluates the
  whole comparison unsigned and it is true for every non-negative `raw`.

`test/test_generated_verilog.py` guards both classes in CI. This directory goes
one step further and checks the RTL actually *correlates*, against an answer known
analytically rather than statistically:

    aligned prompt    = 127 (carrier LUT peak) * amplitude*cos(pi/4) * 4000 samples
    misaligned prompt = the Gold-code off-peak autocorrelation, ~2 orders smaller

## Running

Needs `xvlog`/`xelab` (Vivado) plus migen and LiteX on `PYTHONPATH`; it is
therefore not part of the CI suite.

    ./test/verilog_sim/run.sh              # all four cases
    ./test/verilog_sim/run.sh A            # one case
    PYTHON=/path/to/venv/bin/python WORK=/tmp/vsim ./test/verilog_sim/run.sh

## The cases

| case | injected | carrier NCO | isolates |
|---|---|---|---|
| A | 0 Hz, phase 0 | DC | the code correlator alone |
| B | 0 Hz, phase 250 | DC | that the peak follows the injected code phase |
| C | -3600 Hz, phase 250 | programmed to match | the carrier NCO and its sin/cos ROM |
| D_ctl | -3600 Hz, phase 250 | forced to DC | control: the peak must collapse |

Reference results (PRN 1, amplitude 1500, so an aligned prompt is
`127 * 1061 * 4000 = 538_988_000`):

| case | peak phase | prompt at peak | vs ideal | peak/mean |
|---|---|---|---|---|
| A | 0 ✓ | 538_718_506 | 99.95% | 525x |
| B | 250 ✓ | 538_988_000 | exact | 525x |
| C | 250 ✓ | 508_990_956 | 94.4% | 515x |
| D_ctl | 509 ✗ | -17_212_691 | — | 9x |

A's 0.05% and C's 5.6% shortfalls are the expected quantisation of the code edges
and of the carrier/code frequency words over a 1 ms integration.

## If every phase reports `acc_max` with `saturated` set

The Memory `$readmemh` files are missing next to the Verilog, so the carrier ROM
reads `X`; `X` fails every branch of the clamp's range test and falls through to
its last one, which reports the positive rail. LiteX writes those files relative
to the *current* directory, so emit from inside the work directory (`run.sh` does).
