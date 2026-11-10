#!/usr/bin/env bash
# Software deployment only: never launches a node or opens a serial port.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_ros2_ws/install/setup.bash
source_root="$(cd "$(dirname "$0")/../../.." && pwd)"
workspace="$(mktemp -d /home/xuyu/slaihand_canusb_readonly_ws_20261104_XXXXXX)"
mkdir -p "$workspace/src"
for package in slaihand_transport slaihand_hardware slaihand_description slaihand_bringup; do
  cp -a "$source_root/src/$package" "$workspace/src/"
done
cd "$workspace"
colcon build --base-paths src --packages-select slaihand_transport slaihand_hardware slaihand_description slaihand_bringup --parallel-workers 2
source install/setup.bash
ros2 launch slaihand_bringup slaihand_canusb_readonly.launch.py --show-args
python3 - "$source_root" "$workspace" <<'PY'
import hashlib
import json
from pathlib import Path
import sys
source, workspace = map(Path, sys.argv[1:])
files = {}
for package in ('slaihand_transport', 'slaihand_hardware', 'slaihand_description', 'slaihand_bringup'):
    for path in (source/'src'/package).rglob('*'):
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        relative = path.relative_to(source)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert hashlib.sha256((workspace/relative).read_bytes()).hexdigest() == digest, str(relative)
        files[str(relative)] = digest
report = dict(workspace=str(workspace), copied_files=len(files), source_sha256=files,
              serial_opened=False, hardware_nodes_launched=False)
(source/'verification/hardware/canusb_ros_readonly_20261104/deployment.json').write_text(
    json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print('DEPLOYED_WORKSPACE='+str(workspace))
print('SOURCE_COPY_VERIFIED; NO SERIAL PORT OPENED')
PY
