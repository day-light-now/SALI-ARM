"""Physics and evidence tests; entirely offline, no ROS/port imports."""
import sys
import unittest
import numpy as np
from identify_payload import Geometry,ROOT,HERE,read,prior_parameters,fit_model


class IdentificationTests(unittest.TestCase):
    def test_gravity_is_energy_gradient(self):
        model=Geometry();rng=np.random.default_rng(40)
        for _ in range(15):
            q=rng.uniform(-.1,.6,4);parameters=rng.uniform(-.2,2.,16)
            analytic=model.regressor(q)@parameters
            numerical=[]
            for j in range(4):
                step=np.eye(4)[j]*1e-6
                numerical.append((model.energy(q+step,parameters)-model.energy(q-step,parameters))/2e-6)
            np.testing.assert_allclose(analytic,numerical,atol=2e-8,rtol=1e-8)

    def test_existing_gravity_model_agrees(self):
        sys.path.insert(0,str(ROOT/'verification'))
        from gravity_model import GravityModel
        properties=read(ROOT/'verification/gravity_preparation/provisional_gravity.json')['links']
        other=GravityModel(ROOT/'src/slaihand_description/urdf/slaihand_four_axis.urdf',properties,
                           [0,9.80665,0],fixed_joint_offsets={'slaihand_joint3':np.pi})
        _,cad,_=prior_parameters()
        for q in (np.array([.1,.2,.3,.4]),np.array([-.2,.1,.5,-.1]),np.zeros(4)):
            actual,energy=other.evaluate(q)
            np.testing.assert_allclose(Geometry().regressor(q)@cad,actual,atol=1e-11)
            self.assertAlmostEqual(Geometry().energy(q,cad),energy,places=11)

    def test_known_synthetic_payload_recovered_without_target_pose_substitution(self):
        model=Geometry();rng=np.random.default_rng(7)
        q=rng.uniform(-.1,.6,(35,4));x=np.stack([model.regressor(z) for z in q])
        prior,_,_=prior_parameters();h=3.5*np.array([.12,.025,-.04])
        y=x@prior+3.5*x[:,:,12]+x[:,:,13:16]@h
        recovered,pred,_=fit_model(x,y,prior,3.5,np.arange(35)<25)
        np.testing.assert_allclose(recovered,h,atol=1e-11)
        np.testing.assert_allclose(pred[25:],y[25:],atol=1e-11)

    def test_evidence_split_and_exclusions(self):
        data=read(HERE/'static_windows.json');result=read(HERE/'identification_result.json')
        self.assertEqual(len(data['poses']),30)
        self.assertNotIn('A_9',{p['id'] for p in data['poses']})
        self.assertEqual(len(result['frozen_training_pose_ids']),22)
        self.assertEqual(len(result['holdout_pose_ids']),8)
        self.assertTrue(all(p.startswith('C_') for p in result['holdout_pose_ids']))
        self.assertFalse(result['deployable'])
        c=next(p for p in data['poses'] if p['id']=='C_7')
        np.testing.assert_equal(c['target_rad'],np.zeros(5))
        self.assertGreater(abs(c['q_rad'][0]),.001) # evaluate actual pose, not target zero

    def test_axial_COM_is_not_independently_observable(self):
        result=read(HERE/'identification_result.json');w=np.array(result['identifiability']['nullspace_witness_parameter_delta'])
        self.assertAlmostEqual(w[15],.35,places=10)
        # Independent new poses also preserve torque: the null direction is structural.
        for q in (np.array([.3,.6,-.4,.8]),np.array([-.5,.9,.3,-.2])):
            np.testing.assert_allclose(Geometry().regressor(q)@w,np.zeros(4),atol=1e-10)

    def test_offline_evaluator_refuses_invalid_angles(self):
        from evaluate_offline_ff import evaluate
        with self.assertRaises(ValueError):evaluate([float('nan'),0,0,0])
        with self.assertRaises(ValueError):evaluate([180,180,180,180])
        self.assertFalse(evaluate([10,10,20,10])['deployable'])


if __name__=='__main__':
    unittest.main()
