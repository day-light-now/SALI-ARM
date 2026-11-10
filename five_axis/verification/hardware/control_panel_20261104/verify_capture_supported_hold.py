"""Exercise operator entry against a fresh PTY, in an isolated ROS domain."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import rclpy
import verify_control_panel as fixture
import calibration_core as core

root=Path(__file__).resolve().parent
rclpy.init()
observer=fixture.Observer();session=None
work=Path(tempfile.mkdtemp(prefix='codex-hold-capture-'))
try:
    session=fixture.Session('capture',work,observer,domain=os.environ['ROS_DOMAIN_ID'])
    assert session.plant.path.startswith('/dev/pts/')
    run=subprocess.run([sys.executable,str(root/'capture_supported_hold.py'),'--execute',
        '--ack-supported','--ack-clearance','--ack-native-output','--support','free',
        '--contact','none','--target-deg','0','0','0','-30',
        '--out',str(work/'captured')],capture_output=True,text=True,timeout=40)
    print(run.stdout);print(run.stderr)
    assert run.returncode==0,run.returncode
    paths=list((work/'captured/raw').glob('*.jsonl'));assert len(paths)==1,paths
    raw=core.load_raw(paths[0]);assert len(raw['samples'])==1
    row=raw['samples'][0];assert core.validate_sample(row)[0]
    assert core.exclusion_reason(row) is None
    import math
    assert max(abs(a-math.radians(b)) for a,b in zip(row['positions_rad'],[0,0,0,-30]))<math.radians(.5)
    assert observer.wait(lambda o:o.stage()==8,3)
    print('PASS real Recorder + finite hold + readonly archive; fake PTY only')
finally:
    if session:session.close()
    observer.close();rclpy.shutdown()
