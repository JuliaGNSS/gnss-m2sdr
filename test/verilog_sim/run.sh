#!/usr/bin/env bash
# End-to-end correlation check on the *emitted* RTL, with an injected GPS L1 C/A
# signal of exactly known PRN, code phase and Doppler.
#
# Why this exists. The rest of the suite runs migen's Python simulator, which
# evaluates the design in exact Python integer arithmetic and never goes through
# the Verilog emitter or a $readmemh data file -- so it cannot see a bug that
# lives in the generated text. Two such bugs stopped the on-FPGA correlators
# from correlating (a sign-dropped carrier ROM and an unsigned accumulator-clamp
# comparison), both green in simulation. This runs the RTL LiteX actually ships,
# on a signal whose correct answer is known analytically:
#
#   aligned prompt   = 127 (carrier LUT peak) * amplitude*cos(pi/4) * 4000 samples
#   misaligned prompt = the Gold-code off-peak autocorrelation, ~2 orders smaller
#
# so it is a yes/no result with no statistics.
#
# Needs Vivado's xsim (xvlog/xelab on PATH) and so is not part of the CI suite.
#
#   ./test/verilog_sim/run.sh            # all four cases
#   ./test/verilog_sim/run.sh A          # just case A
set -eu
cd "$(dirname "$0")"
REPO="$(cd ../.. && pwd)"
WORK="${WORK:-$PWD/work}"
ONLY="${1:-}"

mkdir -p "$WORK"
# Emit from inside $WORK: LiteX writes a Memory's $readmemh data files relative to
# the *current* directory, not next to the Verilog. Leave them behind and the
# carrier ROM reads X, which falls through the clamp's last branch and reports
# acc_max with `saturated` set at every phase -- a confusing way to find out.
( SRC="$PWD"; cd "$WORK" && PYTHONPATH="$REPO" ${PYTHON:-python3} "$SRC/emit_channel.py" ch_wrap.v ) >/dev/null
# The emitter does not add a timescale; xsim insists once any module has one.
grep -q '`timescale' "$WORK/ch_wrap.v" || sed -i '1i `timescale 1ns/1ps' "$WORK/ch_wrap.v"
cp tb_inject.sv "$WORK/"

# name  doppler  code_phase   what it isolates
run_case() {
    local name="$1" dopp="$2" cph="$3" note="$4" force_dc="${5:-}"
    [ -n "$ONLY" ] && [ "$ONLY" != "$name" ] && return 0
    echo "═══════════════════════════════════════════════════════════════════"
    echo "CASE $name  doppler=${dopp} Hz  code phase=${cph} chips   -- $note"
    echo "═══════════════════════════════════════════════════════════════════"
    PYTHONPATH="$REPO" ${PYTHON:-python3} gen_signal.py "$WORK" 1 "$dopp" "$cph" | sed 's/^/  /'
    if [ -n "$force_dc" ]; then
        # Control: keep the Doppler-bearing signal but leave the carrier NCO at DC.
        awk '{print $1, 0, $3}' "$WORK/params.txt" > "$WORK/params.tmp"
        mv "$WORK/params.tmp" "$WORK/params.txt"
        echo "  (control: carrier_fw forced to 0)"
    fi
    ( cd "$WORK"
      xvlog -sv ch_wrap.v tb_inject.sv >/dev/null 2>&1
      xelab -R tb_inject 2>&1 | grep -E \
        "params:|best replica|prompt at peak|expected if|peak / mean|expected peak|VERDICT" \
        | sed 's/^/  /' )
    echo
}

# A and B: the carrier is parked at DC, so they measure the code correlator alone
# and show the peak follows the injected code phase.
run_case A       0.0   0.0 "code correlator, replica at chip 0"
run_case B       0.0 250.0 "does the peak follow the injected code phase?"
# C exercises the carrier NCO and its sin/cos ROM. D is the control: the same
# signal with the carrier left at DC must lose the peak, which is what proves the
# wipe-off (and the ROM) is doing the work rather than the code alone.
run_case C   -3600.0 250.0 "carrier NCO programmed to match"
run_case D_ctl -3600.0 250.0 "CONTROL: carrier left at DC, peak must collapse" force
