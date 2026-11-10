"""Three startup raw branches, real ROS/native codec, newly allocated PTY only."""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest.mock import patch
import rclpy
import verify_control_panel as fixture
from ament_index_python.packages import get_package_prefix

def main():
    rclpy.init();original=fixture.Plant
    try:
        for turns in (-1,0,1):
            observer=fixture.Observer();session=None
            work=Path(tempfile.mkdtemp(prefix='codex-branch-'))
            def plant_factory():
                plant=original()
                assert plant.path.startswith('/dev/pts/')
                plant.raw[2]+=turns*2*math.pi
                return plant
            try:
                with patch.object(fixture,'Plant',side_effect=plant_factory):
                    session=fixture.Session('branch',work,observer,domain=os.environ['ROS_DOMAIN_ID'])
                assert observer.wait(lambda o:o.stage()==0 and abs(o.axis(2,'position'))<.5,6)
                before=session.plant.raw[2]
                client=Path(get_package_prefix('slaihand_bringup'))/'lib/slaihand_bringup/commission_supported_trial.py'
                run=subprocess.run([sys.executable,str(client),'--execute','--ack-supported',
                    '--ack-clearance','--ack-native-output','--hold-seconds','1',
                    '--output-directory',str(work/'client')],capture_output=True,text=True,timeout=22)
                assert run.returncode==0,(run.stdout,run.stderr)
                assert observer.wait(lambda o:o.stage()==8,3)
                assert abs(session.plant.raw[2]-before)<.002,(turns,before,session.plant.raw[2])
                assert not session.plant.errors,session.plant.errors
                print('PASS branch',turns,'model_deg',round(math.degrees(observer.axis(2,'position')),3),
                      'raw_hold_delta_rad',session.plant.raw[2]-before,flush=True)
            finally:
                if session:session.close()
                observer.close()
        print('PASS three PTY branches; no full-turn command or physical serial access')
    finally:rclpy.shutdown()
if __name__=='__main__':main()
