"""Build the isolated alpha=0.1 identified-gravity trial, without hardware access."""
from pathlib import Path
import subprocess
import re
import yaml

HERE=Path(__file__).resolve().parent
WORKSPACE=Path('/home/xuyu/slaihand_control_panel_ws_identified_trial_20261105')

def main():
    (WORKSPACE/'src').mkdir(parents=True,exist_ok=True)
    subprocess.run(['bash',str(HERE/'stage_workspace.sh'),str(WORKSPACE)],check=True)
    source=WORKSPACE/'src/slaihand_bringup/config/canusb_motion_policy.yaml'
    policy=yaml.safe_load(source.read_text())
    assert policy['kp']==160 and policy['kd']==5
    assert policy['estimate_guard_nm']==[0.,0.,0.,0.]
    policy['gravity_profile']='gravity_identified_candidate.json'
    policy['gravity_ff_alpha']=.1
    policy['gravity_ff_ramp_s']=2.
    content=source.read_text()
    content=re.sub(r'^gravity_ff_alpha:.*$', 'gravity_ff_alpha: 0.1', content, flags=re.MULTILINE)
    content=re.sub(r'^gravity_ff_ramp_s:.*$', 'gravity_ff_ramp_s: 2.0', content, flags=re.MULTILINE)
    content+='\ngravity_profile: gravity_identified_candidate.json\n'
    assert yaml.safe_load(content)['gravity_ff_alpha']==.1
    source.write_text(content)
    installed=WORKSPACE/'install/slaihand_bringup/share/slaihand_bringup/config/canusb_motion_policy.yaml'
    installed.write_text(content)
    print('Prepared isolated trial alpha=0.1 ramp=2s Kp=160 Kd=5, workspace',WORKSPACE)
if __name__=='__main__':main()
