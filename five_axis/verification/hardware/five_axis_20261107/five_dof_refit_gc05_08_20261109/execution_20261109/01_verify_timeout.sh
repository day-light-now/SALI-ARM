#!/bin/bash
# GC09+ step 1: readback-only J5 TIMEOUT=2000 verification (zero parameter writes).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
EVIDENCE="$HERE/evidence/j5_timeout_verify_existing_20261109_session3.json"
python3 src/slaihand_bringup/scripts/j5_timeout_setup.py \
  --port /dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00 \
  --apply-timeout 2000 \
  --confirm EXCLUSIVE_J5_SETUP \
  --audit "$EVIDENCE" \
  --verify-existing
CODE=$?
echo "verify_existing_exit=$CODE evidence=$EVIDENCE"
exit $CODE
