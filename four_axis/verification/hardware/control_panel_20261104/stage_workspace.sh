#!/usr/bin/env bash
# Stage an isolated, self-contained candidate workspace for the four-axis work.
# Builds the four project packages from the repository source so the candidate
# never depends on another workspace's stale hardware library.
set -eo pipefail
TARGET="${1:-/home/xuyu/slaihand_control_panel_ws_20261104}"
SOURCE_REPO="${2:-/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/src}"
OPENARM_SETUP="${3:-/home/xuyu/slaihand_ros2_ws/install/setup.bash}"
PACKAGES="slaihand_bringup slaihand_description slaihand_transport slaihand_hardware"
case "$TARGET" in
  /home/xuyu/slaihand_control_panel_ws_*) ;;
  *) echo "refusing unexpected target: $TARGET" >&2; exit 2 ;;
esac
[ -d "$SOURCE_REPO" ] || { echo "missing source repo: $SOURCE_REPO" >&2; exit 2; }
for package in $PACKAGES; do
  [ -d "$SOURCE_REPO/$package" ] || { echo "missing source package: $SOURCE_REPO/$package" >&2; exit 2; }
  rm -rf "$TARGET/src/$package"
  cp -a "$SOURCE_REPO/$package" "$TARGET/src/$package"
  find "$TARGET/src/$package" -type d -name __pycache__ -prune -exec rm -rf {} +
  find "$TARGET/src/$package" -name '*.pyc' -delete
done
echo "staged packages: $PACKAGES"
# Build only: OpenArmCAN (static library) comes from the repository install as a
# build-time CMake dependency and is not sourced at test runtime.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$OPENARM_SETUP"
cd "$TARGET"
# shellcheck disable=SC2086
colcon build --packages-select $PACKAGES --event-handlers console_cohesion+
echo "candidate workspace: $TARGET"
find "$TARGET/install" -maxdepth 4 \( -name 'libslaihand_hardware.so' -o -name 'libslaihand_transport.so' \) | sort
