"""为隔离候选显式打开自由拖动开关（测试构建专用；仓库默认 policy 不变）。"""
import re
import sys
from pathlib import Path

WORKSPACE = Path(sys.argv[1] if len(sys.argv) > 1 else
                 '/home/xuyu/slaihand_control_panel_ws_gravitycal_20261106')
if 'slaihand_control_panel_ws_' not in str(WORKSPACE):
    raise SystemExit('refusing unexpected candidate workspace: %s' % WORKSPACE)
changed = []
for relative in ('src/slaihand_bringup/config/canusb_motion_policy.yaml',
                 'install/slaihand_bringup/share/slaihand_bringup/config/canusb_motion_policy.yaml'):
    path = WORKSPACE / relative
    if not path.is_file():
        raise SystemExit('missing policy: %s' % path)
    text = path.read_text(encoding='utf-8')
    if 'gravity_float_enabled' not in text:
        text = text.rstrip('\n') + '\ngravity_float_enabled: true\n'
        path.write_text(text, encoding='utf-8')
        changed.append(str(path))
print('candidate float switch enabled:', changed or 'already present')
print('candidate workspace:', WORKSPACE)
