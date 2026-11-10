#!/usr/bin/env bash
# Only generated PTYs are opened. No hardware launch or physical serial access.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_ros2_ws/install/setup.bash
source_root="$(cd "$(dirname "$0")/../../.." && pwd)"
evidence="$source_root/verification/hardware/canusb_ros_readonly_20261104"
check_ws="$(mktemp -d /tmp/slai-canusb-readonly-XXXXXXXX)"
mkdir -p "$check_ws/src"
for package in slaihand_transport slaihand_hardware slaihand_description slaihand_bringup; do
  cp -a "$source_root/src/$package" "$check_ws/src/"
done
cd "$check_ws"
echo "Isolated offline build: $check_ws"
python3 "$evidence/verify_parity.py" --prepare
colcon build --base-paths src --packages-select slaihand_transport slaihand_hardware slaihand_description slaihand_bringup --parallel-workers 2
source install/setup.bash
python3 "$evidence/verify_launch.py" | tee "$evidence/launch_check.log"
ros2 launch slaihand_bringup slaihand_canusb_readonly.launch.py --show-args > "$evidence/launch_arguments.txt"
colcon test --packages-select slaihand_transport slaihand_hardware --event-handlers console_direct+ | tee "$evidence/ctest.log"
colcon test-result --verbose
xacro install/slaihand_description/share/slaihand_description/urdf/slaihand_canusb_readonly.urdf.xacro port:=/dev/INVALID_UNOPENED -o "$check_ws/readonly.urdf"
"$check_ws/build/slaihand_hardware/canusb_readonly_test" "$check_ws/readonly.urdf" | tee "$evidence/fake_bus.log"
mkdir -p "$evidence/fake_audit"
cp "$check_ws/fake_audit/"*.jsonl "$evidence/fake_audit/"
"$check_ws/build/slaihand_transport/canusb_codec_test" --packets > "$evidence/cpp_read_packets.txt"
"$check_ws/build/slaihand_transport/canusb_codec_test" --replay "$evidence/rx_fixture.txt" > "$evidence/cpp_decoded.txt"
python3 "$evidence/verify_parity.py"
cp "$check_ws/readonly.urdf" "$evidence/expanded_readonly.urdf"
check_urdf "$check_ws/readonly.urdf" | tee "$evidence/check_urdf.log"
python3 "$evidence/verify_ros_graph.py"
