#!/usr/bin/env bash
# 拖动示教：候选构建 + 离线反例 + 真实回调反例 + PTY E2E。
# 只使用独立候选工作空间与隔离域 176；不触碰实机串口、域 0、现有驱动/面板。
set -o pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
CANDIDATE=/home/xuyu/slaihand_control_panel_ws_dragteaching_20261106

echo "=== 候选构建（显式使用本工作树的 src）"
bash "$ROOT/verification/hardware/control_panel_20261104/stage_workspace.sh" \
  "$CANDIDATE" "$ROOT/src" 2>&1 | tail -5
build_status=${PIPESTATUS[0]}

echo "=== 候选自由拖动开关（只改测试候选的 policy）"
python3 "$HERE/prepare_drag_teaching_candidate.py" "$CANDIDATE"
prepare_status=$?

# shellcheck disable=SC1091
source "$HERE/test_env_drag.sh" || exit 1
cd "$HERE" || exit 1
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1

echo "=== 拖动示教离线反例"
python3 test_drag_teaching.py 2>&1 | tee /tmp/drag_teaching_offline.log
offline_status=${PIPESTATUS[0]}

echo "=== 拖动示教面板级回归（身份绑定票根 / 无帧诊断记录 / 源帧去重）"
python3 test_drag_teaching_panel.py 2>&1 | tee /tmp/drag_teaching_panel.log
panel_status=${PIPESTATUS[0]}

echo "=== 既有面板回调反例（确认未破坏基线入口）"
python3 "$ROOT/verification/hardware/control_panel_20261104/test_control_panel_callbacks.py" \
  > /tmp/drag_teaching_callbacks.log 2>&1
callback_status=$?
tail -3 /tmp/drag_teaching_callbacks.log

echo "=== 拖动示教 PTY/ROS/JTC E2E"
python3 verify_drag_teaching_e2e.py 2>&1 | tee /tmp/drag_teaching_e2e.log
e2e_status=${PIPESTATUS[0]}

if [ "$build_status" -ne 0 ] || [ "$prepare_status" -ne 0 ] || [ "$offline_status" -ne 0 ] \
   || [ "$panel_status" -ne 0 ] || [ "$callback_status" -ne 0 ] || [ "$e2e_status" -ne 0 ]; then
  echo "RESULT: FAIL (build=$build_status prepare=$prepare_status offline=$offline_status panel=$panel_status callbacks=$callback_status e2e=$e2e_status)"
  exit 1
fi
echo "RESULT: PASS (build=0 prepare=0 offline=0 panel=0 callbacks=0 e2e=0)"
