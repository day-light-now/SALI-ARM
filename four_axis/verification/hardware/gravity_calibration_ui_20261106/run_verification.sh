#!/usr/bin/env bash
# 本轮最终验证：候选构建 + C++ + 纯逻辑 + 面板门禁 + 只读报告 + PTY/ROS/JTC E2E。
# 浏览器验证在 Windows 侧单独运行 browser_supported_capture.py。
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${SLAIHAND_CANDIDATE_WS:-/home/xuyu/slaihand_control_panel_ws_gravitycal_20261106}
FAIL=0
step() {
  echo ""
  echo "===== $* ====="
}
run() {
  "$@"
  local code=$?
  if [ $code -ne 0 ]; then echo "STEP_FAILED code=$code: $*"; FAIL=1; fi
  return $code
}
step "candidate build from this worktree src"
bash "$HERE/stage_workspace_gravitycal.sh" "$WS" >/tmp/gcal_run_build.log 2>&1 || { echo "BUILD_FAILED"; tail -20 /tmp/gcal_run_build.log; exit 1; }
step "C++ supervisor tests"
(
  cd "$WS" || exit 1
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source install/local_setup.bash
  ctest --test-dir build/slaihand_hardware --output-on-failure
) || FAIL=1
step "pure protocol tests"
run python3 "$HERE/test_supported_capture.py" || FAIL=1
step "panel gating tests"
(
  # shellcheck disable=SC1091
  source "$HERE/test_env_gravitycal.sh"
  python3 "$HERE/test_supported_capture_panel.py"
) || FAIL=1
step "read-only residual/coverage report"
run python3 "$HERE/readonly_residual_report.py" || FAIL=1
step "PTY/ROS/JTC end-to-end"
(
  # shellcheck disable=SC1091
  source "$HERE/test_env_gravitycal.sh"
  python3 "$HERE/verify_gravity_calibration_ui_e2e.py"
) || FAIL=1
echo ""
if [ "$FAIL" -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; fi
exit "$FAIL"
