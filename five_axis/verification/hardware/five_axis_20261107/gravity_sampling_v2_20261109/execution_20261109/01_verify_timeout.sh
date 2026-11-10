#!/bin/bash
# Step 1: readback-only J5 TIMEOUT=2000 verification (no parameter write).
# Refuses if TIMEOUT is not already 2000 or any axis is not fresh DISABLED.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
EVIDENCE="$HERE/evidence/j5_timeout_verify_existing_20261109_session2.json"
python3 src/slaihand_bringup/scripts/j5_timeout_setup.py \
  --port /dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00 \
  --apply-timeout 2000 \
  --confirm EXCLUSIVE_J5_SETUP \
  --audit "$EVIDENCE" \
  --verify-existing
CODE=$?
echo "verify_existing_exit=$CODE evidence=$EVIDENCE"
exit $CODE
