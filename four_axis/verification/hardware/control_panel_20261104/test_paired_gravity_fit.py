"""Offline checks for the paired-data fit and geometry regressor."""
import json,math,unittest
from pathlib import Path
import numpy as np
import fit_bidirectional_campaign as fit
from control_panel_gravity import FourAxisGravityModel

class PairedFitTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  policy=fit.capture._policy_values()
  cls.model=FourAxisGravityModel(fit.capture.URDF,fit.capture.PROFILE,
    gravity_base=[float(policy['gravity_base_'+axis]) for axis in 'xyz'],
    fixed_joint_offsets={'slaihand_joint3':float(policy['gravity_j3_offset_rad'])})
  cls.parameters=[]
  for link,(mass,center) in cls.model.masses.items():
   if link!=cls.model.root:cls.parameters.extend([mass,*[mass*x for x in center]])
 def test_geometry_columns_reconstruct_analytic_torque(self):
  rng=np.random.default_rng(20261105)
  for _ in range(24):
   q=[rng.uniform(a,b) for a,b in self.model.limits]
   actual=fit.geometry_regressor(self.model,q)@self.parameters
   np.testing.assert_allclose(actual,self.model.compute(q)['tau_model_nm'],atol=1e-10,rtol=1e-10)
 def test_paired_matrix_keeps_two_actual_poses(self):
  q=np.array([.2,.3,-.1,.2]);offset=np.array([.01,-.01,.005,-.005])
  matrix=(fit.geometry_regressor(self.model,q+offset)+fit.geometry_regressor(self.model,q-offset))/2
  expected=(np.array(self.model.compute(q+offset)['tau_model_nm'])+self.model.compute(q-offset)['tau_model_nm'])/2
  np.testing.assert_allclose(matrix@self.parameters,expected,atol=1e-10)
 def test_candidate_reproduces_training_only_fit(self):
  pairs,_,_=fit.load_campaign()
  train=[p for p in pairs if p['role']=='training']
  test=[p for p in pairs if p['role']=='holdout']
  self.assertEqual((len(train),len(test)),(12,2))
  candidate=json.loads((fit.HERE/'evidence/identified_gravity_candidate.json').read_text())
  matrix=np.stack([(fit.geometry_regressor(self.model,p['positive_q_rad'])+fit.geometry_regressor(self.model,p['negative_q_rad']))/2 for p in train])
  measured=np.array([p['mean_estimate_nm'] for p in train])
  coefficients=np.linalg.lstsq(matrix.reshape(-1,matrix.shape[-1]),measured.ravel(),rcond=1e-6)[0]
  np.testing.assert_allclose(coefficients,candidate['effective_coefficients'],atol=1e-10)
  self.assertFalse(set(candidate['training_pair_ids'])&set(candidate['validation_pair_ids']))
  self.assertEqual(candidate['status'],'LOCAL_OFFLINE_CANDIDATE')
 def test_saved_holdout_error_is_reproducible(self):
  pairs,_,_=fit.load_campaign()
  candidate=json.loads((fit.HERE/'evidence/identified_gravity_candidate.json').read_text())
  holdout=[p for p in pairs if p['role']=='holdout']
  prediction=[]
  for pair in holdout:
   matrix=(fit.geometry_regressor(self.model,pair['positive_q_rad'])+fit.geometry_regressor(self.model,pair['negative_q_rad']))/2
   prediction.append(matrix@candidate['effective_coefficients'])
  measured=np.array([p['mean_estimate_nm'] for p in holdout])
  rmse=float(np.sqrt(np.mean((np.array(prediction)-measured)**2)))
  self.assertAlmostEqual(rmse,candidate['holdout']['rmse_nm'],places=11)

if __name__=='__main__':unittest.main()
