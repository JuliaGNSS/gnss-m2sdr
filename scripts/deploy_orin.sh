#!/usr/bin/env bash
# Deploy the GNSS gateware + host software to orin2.
# Usage: scripts/deploy_orin.sh [BUILD_NAME]
set -euo pipefail

HOST="${ORIN_HOST:-orin@orin2}"
BUILD="${1:-gnss_m2sdr_m2_x1_ch4}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
GW="$REPO/build/$BUILD/gateware/$BUILD.bin"
CSV="$REPO/build/$BUILD/csr.csv"
HDR="$REPO/build/$BUILD/software/include/generated"

echo "Deploying $BUILD to $HOST ..."
ssh "$HOST" 'mkdir -p ~/gnss-m2sdr/build/'"$BUILD"'/gateware ~/gnss-m2sdr/build/'"$BUILD"'/software/include/generated'

# Host software + repo (Python only; no build needed on orin2).
rsync -az --exclude build --exclude .git --exclude '__pycache__' \
    "$REPO/gnss_m2sdr" "$REPO/software" "$REPO/docs" "$HOST:~/gnss-m2sdr/"

# Gateware image + CSR map.
scp "$GW"  "$HOST:~/gnss-m2sdr/build/$BUILD/gateware/"
scp "$CSV" "$HOST:~/gnss-m2sdr/build/$BUILD/"

# Generated C headers (to rebuild the M2SDR driver/tools for this gateware).
if [ -d "$HDR" ]; then
    scp "$HDR"/csr.h "$HDR"/soc.h "$HDR"/mem.h "$HOST:~/gnss-m2sdr/build/$BUILD/software/include/generated/" || true
fi

echo "Done. Next: follow docs/hardware_bringup.md on $HOST."
