#!/usr/bin/env bash
set -o pipefail
cd /mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain || exit 1
source /home/xuyu/.cache/dsh_five/install/setup.bash
export ROS_DOMAIN_ID=183
OUT=/tmp/forward_j1_10_20261109.jsonl
rm -f "$OUT"
timeout 8 ros2 service call /slaihand/reset_disabled std_srvs/srv/Trigger "{}" >/tmp/reset_forward.log 2>&1
sleep 1
python3 verification/hardware/five_axis_20261107/signed_acquisition/recorder.py \
  --duration 45 --output "$OUT" >/tmp/recorder_forward.log 2>&1 &
REC=$!
sleep 2
timeout 80 python3 src/slaihand_bringup/scripts/commission_supported_trial.py \
  --plan verification/hardware/five_axis_20261107/signed_acquisition/FORWARD_J1_10.json \
  --joint-count 5 --j5-limits-deg -90 90 --load-id j5_3p5kg_provisional \
  --execute --ack-supported --ack-clearance --ack-native-output \
  --hold-seconds 1 --output-directory /tmp/forward_j1_10_client \
  >/tmp/client_forward.log 2>&1
CLIENT=$?
wait "$REC" 2>/dev/null || true
echo "CLIENT=$CLIENT RECORDER=$REC OUT=$OUT"
tail -10 /tmp/client_forward.log
wc -l "$OUT" 2>/dev/null || true
