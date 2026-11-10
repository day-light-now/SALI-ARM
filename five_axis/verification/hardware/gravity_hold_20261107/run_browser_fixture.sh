#!/bin/bash
set -eo pipefail
WS=/mnt/d/CodexData/Projects/机械臂/slaihand_gravity_hold_ws
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1
export PANEL_SCRIPT="$WS/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py"
export PANEL_WORKSPACE="$WS"
export PYTHONPATH="$WS/verification/hardware/canusb_motion_20261104:$WS/verification/hardware:$WS/verification/hardware/control_panel_20261104:${PYTHONPATH:-}"
POLICY="$WS/src/slaihand_bringup/config/canusb_motion_policy.yaml"
BACKUP=/tmp/slaihand_hold_policy_backup.yaml
cp "$POLICY" "$BACKUP"
python3 - "$POLICY" <<'PY'
import sys, yaml
from pathlib import Path
path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
data['gravity_float_enabled'] = True
# Reproduce the deployed normal-position baseline; float mode still forces alpha=1.
data['gravity_ff_alpha'] = 0.1
path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
PY
cleanup() {
  if [ -n "${CHILD:-}" ] && kill -0 "$CHILD" 2>/dev/null; then kill -TERM "$CHILD" 2>/dev/null || true; fi
  cp "$BACKUP" "$POLICY"
}
trap cleanup EXIT TERM INT
rm -rf /tmp/panel-browser-fixture
python3 verification/hardware/control_panel_20261104/browser_fixture.py > /tmp/hold_browser_fixture.log 2>&1 &
CHILD=$!
wait "$CHILD"
