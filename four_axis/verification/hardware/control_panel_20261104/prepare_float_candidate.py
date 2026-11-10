"""Select the identified model and explicitly enable the bounded float service."""
from pathlib import Path
import re
import sys
import yaml

WORKSPACE = Path(sys.argv[1] if len(sys.argv) > 1 else '/home/xuyu/slaihand_control_panel_ws_float_20261105')
assert str(WORKSPACE) in ('/home/xuyu/slaihand_control_panel_ws_float_20261105',
                         '/home/xuyu/slaihand_control_panel_ws_speed1_20261105')


def main():
    source = WORKSPACE / 'src/slaihand_bringup/config/canusb_motion_policy.yaml'
    policy = yaml.safe_load(source.read_text())
    assert policy['kp'] == 160 and policy['kd'] == 5
    assert policy['estimate_guard_nm'] == [0, 0, 0, 0]
    content = source.read_text()
    content = re.sub(r'^gravity_ff_alpha:.*$', 'gravity_ff_alpha: 0.1', content, flags=re.MULTILINE)
    content = re.sub(r'^gravity_ff_ramp_s:.*$', 'gravity_ff_ramp_s: 2.0', content, flags=re.MULTILINE)
    content = re.sub(r'^gravity_profile:.*\n?', '', content, flags=re.MULTILINE)
    content = re.sub(r'^gravity_float_enabled:.*\n?', '', content, flags=re.MULTILINE)
    content += '\ngravity_profile: gravity_identified_candidate.json\ngravity_float_enabled: true\n'
    source.write_text(content)
    installed = WORKSPACE / 'install/slaihand_bringup/share/slaihand_bringup/config/canusb_motion_policy.yaml'
    installed.write_text(content)
    print('Prepared explicit float candidate. Normal arm retains alpha .1, Kp160/Kd5.')


if __name__ == '__main__':
    main()
