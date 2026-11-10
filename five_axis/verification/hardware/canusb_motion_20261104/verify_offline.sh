#!/usr/bin/env bash
# Only software and generated PTYs; NEVER opens a physical CANUSB device.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_ros2_ws/install/setup.bash
source_root="$(cd "$(dirname "$0")/../../.." && pwd)"
evidence="$source_root/verification/hardware/canusb_motion_20261104"
workspace="$(mktemp -d /tmp/slai-canusb-motion-XXXXXXXX)"
mkdir -p "$workspace/src"
for package in slaihand_transport slaihand_hardware slaihand_description slaihand_bringup;do cp -a "$source_root/src/$package" "$workspace/src/";done
cd "$workspace"
echo "OFFLINE_WORKSPACE=$workspace"
colcon build --base-paths src --packages-select slaihand_transport slaihand_hardware slaihand_description slaihand_bringup --parallel-workers 2
source install/setup.bash
colcon test --packages-select slaihand_transport slaihand_hardware --event-handlers console_direct+ | tee "$evidence/ctest.log"
colcon test-result --verbose
python3 "$evidence/verify_ros_motion.py"
