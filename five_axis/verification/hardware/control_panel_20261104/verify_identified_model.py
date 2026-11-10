"""Cross-check the deployed identified predictor against fit, Python and C++."""
import json,math,random,subprocess,tempfile
from pathlib import Path
import numpy as np
import verify_gravity_model as cross
from control_panel_gravity import FourAxisGravityModel,GravityModelError
from fit_bidirectional_campaign import geometry_regressor
from calibration_capture import URDF,PROFILE,_model_fn

HERE=Path(__file__).resolve().parent
CANDIDATE=HERE.parents[2]/'src/slaihand_bringup/config/gravity_identified_candidate.json'

def main():
 candidate=json.loads(CANDIDATE.read_text())
 fitted=FourAxisGravityModel(URDF,CANDIDATE,gravity_base=(0.,9.80665,0.),fixed_joint_offsets={'slaihand_joint3':math.pi})
 physical=FourAxisGravityModel(URDF,PROFILE,gravity_base=(0.,9.80665,0.),fixed_joint_offsets={'slaihand_joint3':math.pi})
 poses=[];rng=random.Random(20261105)
 for _ in range(80):poses.append([rng.uniform(lo,hi) for lo,hi in fitted.limits])
 report=json.loads((HERE/'evidence/paired_gravity_campaign_fit.json').read_text())
 for pair in report['pairs']:poses.extend([pair['positive_q_rad'],pair['negative_q_rad']])
 assert fitted.total_mass_kg is None
 assert fitted.parameterization=='identified_base'
 worst=0.
 with tempfile.TemporaryDirectory() as temporary:
  directory=Path(temporary);binary=directory/'dump';cross.build_dump(binary)
  text=cross.launch_module().build_gravity_model_text(URDF,CANDIDATE,[0.,9.80665,0.],math.pi)
  assert '|B:' in text and '|L:' not in text
  path=directory/'model.txt';path.write_text(text)
  rows=subprocess.run([str(binary),str(path)],input='\n'.join(' '.join(map(str,q)) for q in poses)+'\n',capture_output=True,text=True,check=True).stdout.splitlines()
  for q,row in zip(poses,rows):
   estimate=fitted.compute(q)
   expected=geometry_regressor(physical,q)@candidate['effective_coefficients']
   np.testing.assert_allclose(estimate['tau_model_nm'],expected,atol=1e-10)
   cpp=np.array(list(map(float,row.split())))
   python=np.array([*estimate['tau_model_nm'],estimate['potential_energy_j']])
   worst=max(worst,float(np.max(np.abs(cpp-python))))
   for index in range(4):
    if not fitted.limits[index][0]+1e-5<q[index]<fitted.limits[index][1]-1e-5:continue
    plus=list(q);minus=list(q);plus[index]+=1e-6;minus[index]-=1e-6
    derivative=(fitted.potential_energy(plus)-fitted.potential_energy(minus))/2e-6
    assert abs(derivative-estimate['tau_model_nm'][index])<1e-7
 assert worst<1e-10,worst
 # Wrong geometry/direction must be rejected before a driver can start.
 try:FourAxisGravityModel(URDF,CANDIDATE,gravity_base=(0,0,-9.80665),fixed_joint_offsets={'slaihand_joint3':math.pi})
 except GravityModelError:pass
 else:raise AssertionError('wrong gravity accepted')
 (HERE/'evidence/identified_model_cross_check.json').write_text(json.dumps({'passed':True,'poses':len(poses),'max_cpp_python_difference':worst,'matches_offline_fit':True,'potential_energy_difference_passed':True},indent=2))
 print('PASS identified model:',len(poses),'poses; fit/Python/C++ max difference',worst)
if __name__=='__main__':main()
