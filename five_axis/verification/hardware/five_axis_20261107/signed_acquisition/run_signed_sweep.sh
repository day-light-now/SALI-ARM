#!/usr/bin/env bash
set -o pipefail
cd /mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain || exit 1
source /home/xuyu/.cache/dsh_five/install/setup.bash
export ROS_DOMAIN_ID=183
OUT=/tmp/signed_sweep_20261109.jsonl
rm -f "$OUT"
timeout 8 ros2 service call /slaihand/reset_disabled std_srvs/srv/Trigger "{}" >/tmp/reset_signed.log 2>&1
sleep 1
python3 verification/hardware/five_axis_20261107/signed_acquisition/recorder.py \
  --duration 95 --output "$OUT" >/tmp/signed_recorder.log 2>&1 &
REC=$!
sleep 2
timeout 150 python3 src/slaihand_bringup/scripts/commission_supported_trial.py \
  --plan verification/hardware/five_axis_20261107/five_axis_2deg_diagnostics/WORKSPACE_0_40_ACTUALSTART.json \
  --joint-count 5 --j5-limits-deg -90 90 --load-id j5_3p5kg_provisional \
  --execute --ack-supported --ack-clearance --ack-native-output \
  --hold-seconds 1 --output-directory /tmp/signed_sweep_client \
  >/tmp/signed_client.log 2>&1
CLIENT=$?
wait "$REC" 2>/dev/null || true
echo "CLIENT=$CLIENT RECORDER=$REC OUT=$OUT"
tail -8 /tmp/signed_client.log
wc -l "$OUT" 2>/dev/null || true
