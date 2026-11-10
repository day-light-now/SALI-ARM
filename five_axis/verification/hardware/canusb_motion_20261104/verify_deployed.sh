#!/usr/bin/env bash
# Verify the exact installed handoff copy; tests use generated PTYs only.
set -eo pipefail
source_root="$(cd "$(dirname "$0")/../../.." && pwd)"
evidence="$source_root/verification/hardware/canusb_motion_20261104"
workspace="$(python3 - "$evidence/deployment.json" <<'PY'
from pathlib import Path
import json,sys
path=Path(json.loads(Path(sys.argv[1]).read_text())['workspace']).resolve()
assert path.parent==Path('/home/xuyu') and path.name.startswith('slaihand_canusb_motion_ws_20261104_')
print(path)
PY
)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
cd "$workspace"
colcon test --packages-select slaihand_transport slaihand_hardware --event-handlers console_direct+ | tee "$evidence/deployed_ctest.log"
colcon test-result --verbose
python3 "$evidence/verify_ros_motion.py"
python3 - "$evidence" <<'PY'
from pathlib import Path
import hashlib,sys,xacro
from ament_index_python.packages import get_package_share_directory
out=Path(sys.argv[1]);description=Path(get_package_share_directory('slaihand_description'));bringup=Path(get_package_share_directory('slaihand_bringup'))
mapping=bringup/'config/hardware_mapping.yaml';model=description/'urdf/slaihand_four_axis.urdf';policy=bringup/'config/canusb_motion_policy.yaml'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
xml=xacro.process_file(str(description/'urdf/slaihand_canusb_motion.urdf.xacro'),mappings=dict(port='/dev/INVALID_UNOPENED',
    mapping_sha256=sha(mapping),model_sha256=sha(model),policy_sha256=sha(policy))).toxml()
(out/'expanded_motion.urdf').write_text(xml,encoding='utf-8')
xml=xacro.process_file(str(description/'urdf/slaihand_canusb_readonly.urdf.xacro'),mappings=dict(port='/dev/INVALID_UNOPENED')).toxml()
(out/'expanded_readonly_compat.urdf').write_text(xml,encoding='utf-8')
PY
"$workspace/build/slaihand_hardware/canusb_readonly_test" "$evidence/expanded_readonly_compat.urdf" | tee "$evidence/readonly_compat.log"
check_urdf "$evidence/expanded_motion.urdf" | tee "$evidence/check_urdf.log"
