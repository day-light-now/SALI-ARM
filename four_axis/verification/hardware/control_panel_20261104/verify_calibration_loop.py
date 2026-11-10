"""Actual ROS/JTC endpoint-offset reproduction on a newly allocated PTY only."""
import json,math,os,subprocess,sys,tempfile,time
from pathlib import Path
from unittest.mock import patch
import rclpy
import verify_control_panel as f
from ament_index_python.packages import get_package_prefix

def main():
 mode=os.environ.get('CALIBRATION_LOOP_MODE','baseline');out=Path(tempfile.mkdtemp(prefix='calibration-loop-'))
 rclpy.init();observer=f.Observer();session=None;original=f.Plant
 def factory():
  p=original();p.tracking_bias=math.radians(.8);return p
 try:
  with patch.object(f,'Plant',side_effect=factory):session=f.Session('calib-loop',out,observer,domain='176')
  assert observer.wait(lambda o:o.stage()==0,6)
  plan={'coordinates':'relative_start_deg','waypoints':[{'degrees':[2,2,0,0],'move_s':4.,'dwell_s':6.}]}
  path=out/'plan.json';path.write_text(json.dumps(plan))
  argv=[sys.executable,str(Path(get_package_prefix('slaihand_bringup'))/'lib/slaihand_bringup/commission_supported_trial.py'),'--execute','--ack-supported','--ack-clearance','--ack-native-output','--plan',str(path),'--hold-seconds','1','--output-directory',str(out/'client')]
  if mode=='calibration':argv+=['--calibrate','--ack-free-static']
  run=subprocess.run(argv,capture_output=True,text=True,timeout=30)
  result=json.loads(next((out/'client').glob('*.json')).read_text())
  # Limit diagnostics to our instance's audit; never touch physical logs/ports.
  audits=list(out.rglob('canusb_motion_*.jsonl'))
  if not audits:audits=list(session.directory.rglob('canusb_motion_*.jsonl')) if hasattr(session,'directory') else []
  print('OUTPUT',run.stdout[-2500:],flush=True)
  print('RESULT',run.returncode,result.get('reason'),result.get('calibration'), 'work',out,flush=True)
  assert not session.plant.errors,session.plant.errors
  assert result.get('stop_observations_confirmed'),result
  if mode=='baseline':
   assert run.returncode!=0 and result.get('trajectory_error_code')==-5,result
   print('REPRODUCED: 0.8 degree static offset causes JTC goal abort')
  else:
   assert run.returncode==0,result
   assert result['calibration']['qualified_points']==1,result
   assert result.get('waypoints_observed') is False,'precision must remain independently failed'
   raw=Path(result['calibration']['raw_file']);assert raw.is_file()
   print('PASS: stable offset captured; precision failure kept; no speed abort')
 finally:
  if session:session.close()
  observer.close();rclpy.shutdown()
if __name__=='__main__':main()
