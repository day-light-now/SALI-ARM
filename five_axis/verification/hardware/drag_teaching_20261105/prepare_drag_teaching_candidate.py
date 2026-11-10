"""为拖动示教测试候选工作空间显式打开自由拖动开关（只改候选构建的 policy）。

不改仓库默认 policy；实机部署开关由 Codex 验收后按现场流程处理。
"""
import re
import sys
from pathlib import Path

WORKSPACE = Path(sys.argv[1] if len(sys.argv) > 1 else
                 '/home/xuyu/slaihand_control_panel_ws_dragteaching_20261106')
if 'slaihand_control_panel_ws_' not in str(WORKSPACE):
    raise SystemExit('refusing unexpected candidate workspace: %s' % WORKSPACE)
for relative in ('src/slaihand_bringup/config/canusb_motion_policy.yaml',
                 'install/slaihand_bringup/share/slaihand_bringup/config/canusb_motion_policy.yaml'):
    path = WORKSPACE / relative
    if not path.is_file():
        raise SystemExit('missing policy: %s' % path)
    text = path.read_text(encoding='utf-8')
    assert re.search(r'^kp:\s*160\.0', text, re.M)
    assert re.search(r'^kd:\s*5\.0', text, re.M)
    assert re.search(r'^gravity_ff_alpha:\s*0\.0', text, re.M)
    text = re.sub(r'^gravity_float_enabled:.*\n?', '', text, flags=re.M)
    text = text.rstrip('\n') + '\ngravity_float_enabled: true\n'
    path.write_text(text, encoding='utf-8')
print('candidate float switch enabled (test build only):', WORKSPACE)
