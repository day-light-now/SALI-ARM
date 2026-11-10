"""Real ROS/native-MIT write path for the identified alpha=0.1 finite hold."""
import hashlib,json,math,os,tempfile,time
from pathlib import Path
import rclpy
import verify_control_panel as f
from verify_gravity_ff_pty import FeedforwardTap
from control_panel_gravity import FourAxisGravityModel
from ament_index_python.packages import get_package_share_directory
os.environ['ROS_DOMAIN_ID']='177'

def main():
 rclpy.init();observer=f.Observer();tap=FeedforwardTap();session=None
 work=Path(tempfile.mkdtemp(prefix='identified-trial-'))
 try:
  session=f.Session('identified-hold',work,observer,domain='177')
  panel,plant=session.panel,session.plant
  tap.spin(1.2)
  assert tap.messages and tap.messages[-1][0]==.1 and tap.messages[-1][1]==2.,tap.messages[-1:]
  status=panel.status();shadow=status['gravity_shadow']
  assert shadow.get('metadata',{}).get('parameterization')=='identified_base',shadow
  assert shadow['metadata']['total_mass_kg'] is None
  share=Path(get_package_share_directory('slaihand_bringup'))
  profile=share/'config/gravity_identified_candidate.json'
  assert shadow['metadata']['mass_profile_sha256']==hashlib.sha256(profile.read_bytes()).hexdigest()
  model=FourAxisGravityModel(Path(get_package_share_directory('slaihand_description'))/'urdf/slaihand_four_axis.urdf',profile,gravity_base=(0.,9.80665,0.),fixed_joint_offsets={'slaihand_joint3':math.pi})
  q=[math.radians(a['actual_deg']) for a in status['axes']]
  expected=[sign*.1*tau for sign,tau in zip((1,1,1,-1),model.compute(q)['tau_model_nm'])]
  code,accepted=panel.call('POST','/api/hold',{'confirmations':f.confirmations()})
  assert code==202,accepted
  samples=[];entered=None;operation=None
  deadline=time.monotonic()+22
  while time.monotonic()<deadline:
   tap.spin(.04);observer.spin(.01)
   status=panel.status();operation=status.get('operation') or {}
   stage=status['ros']['stage']
   if stage==5:
    if entered is None:entered=time.monotonic()
    if time.monotonic()-entered>=2.:
     samples.append(list(getattr(plant,'last_tau',[0.]*4)))
   if operation and not operation.get('running'):break
  assert operation['exit_code']==0 and operation['result']['stop_observations_confirmed'],operation
  assert samples,'no full-ramp hold observations'
  decoded=samples[-1]
  tolerances=[200/4095+.001,200/4095+.001,54/4095+.001,54/4095+.001]
  assert all(abs(a-b)<=t for a,b,t in zip(decoded,expected,tolerances)),(decoded,expected)
  tap.spin(.2)
  assert tap.messages[-1][2]==0 and tap.messages[-1][13:17]==[0.]*4,tap.messages[-1]
  assert not plant.errors,plant.errors
  assert plant.fc==[1]*4 and all(x>=1 for x in plant.fd)
  result={'passed':True,'physical_port_opened':False,'workspace':str(share),'alpha':.1,'ramp_s':2.,'model':'identified_base','expected_protocol_nm':expected,'decoded_mit_nm':decoded,'stop_confirmed':True,'post_stop_applied_nm':tap.messages[-1][13:17],'work':str(work)}
  (Path(__file__).resolve().parent/'evidence/identified_trial_pty_result.json').write_text(json.dumps(result,indent=2))
  print('PASS identified finite hold',json.dumps(result),flush=True)
 finally:
  if session:session.close()
  tap.close();observer.close();rclpy.shutdown()
if __name__=='__main__':main()
