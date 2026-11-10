#!/bin/bash
# Step 5: one acquisition = recorder first, then the protected client.
# Usage: 05_run_plan.sh <plan.json> <TAG> <recorder_duration_s>
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
PLAN="$1"
TAG="$2"
DUR="$3"
RAW="$HERE/raw/${TAG}_20261109.jsonl"
CLIENT_DIR="$HERE/clients/$TAG"
LOG="$HERE/logs/${TAG}.log"
mkdir -p "$CLIENT_DIR"
echo "=== $TAG start $(date -Is) plan=$PLAN recorder_duration=$DUR ===" > "$LOG"
python3 verification/hardware/five_axis_20261107/signed_acquisition/recorder.py \
  --duration "$DUR" --output "$RAW" >> "$LOG" 2>&1 &
REC=$!
sleep 2.5
python3 src/slaihand_bringup/scripts/commission_supported_trial.py \
  --plan "$PLAN" \
  --joint-count 5 --j5-limits-deg -90 90 \
  --load-id j5_3p5kg_provisional \
  --execute --ack-supported --ack-clearance --ack-native-output \
  --hold-seconds 1 --output-directory "$CLIENT_DIR" >> "$LOG" 2>&1
CODE=$?
kill -TERM "$REC" 2>/dev/null || true
wait "$REC" 2>/dev/null || true
gzip -f "$RAW" 2>/dev/null || true
echo "=== $TAG client_exit=$CODE raw=${RAW}.gz log=$LOG ===" >> "$LOG"
echo "$TAG client_exit=$CODE"
exit "$CODE"
