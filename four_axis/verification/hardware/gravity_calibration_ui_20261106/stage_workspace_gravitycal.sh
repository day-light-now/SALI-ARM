#!/usr/bin/env bash
# 从本工作树 src 构建独立候选；显式传入 SOURCE_REPO，绝不使用旧主工程。
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BASE=$(cd "$HERE/../../.." && pwd)
TARGET="${1:-/home/xuyu/slaihand_control_panel_ws_gravitycal_20261106}"
mkdir -p "$TARGET/src"
bash "$BASE/verification/hardware/control_panel_20261104/stage_workspace.sh" "$TARGET" "$BASE/src" "${2:-/home/xuyu/slaihand_ros2_ws/install/setup.bash}"
python3 "$HERE/prepare_gravitycal_candidate.py" "$TARGET"
