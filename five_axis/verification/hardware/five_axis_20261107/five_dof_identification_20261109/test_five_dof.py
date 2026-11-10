"""Physics checks and real-data rejection checks, no device access."""
import unittest
import numpy as np
from fit_five_dof import FiveDofPotential,rigid_parameters,HERE,read


class FiveDofTests(unittest.TestCase):
    def test_all_five_torques_are_energy_gradient(self):
        model=FiveDofPotential();rng=np.random.default_rng(5)
        h,_=rigid_parameters([.1,.02,-.04,.03,-.01,.7,.4])
        for _ in range(20):
            q=rng.uniform(-.15,.6,5)
            analytic=model.base(q)+model.orbit_regressor(q)@h
            numerical=[]
            for j in range(5):
                step=np.eye(5)[j]*1e-6
                numerical.append((model.energy(q+step,h)-model.energy(q-step,h))/2e-6)
            np.testing.assert_allclose(analytic,numerical,atol=3e-8,rtol=1e-8)

    def test_rigid_circle_geometry(self):
        h,axis=rigid_parameters([.1,.02,-.04,.03,-.01,.7,.4])
        b,u,v=h.reshape(3,3)
        self.assertAlmostEqual(np.linalg.norm(axis),1.)
        self.assertAlmostEqual(np.dot(u,v),0.)
        self.assertAlmostEqual(np.linalg.norm(u),np.linalg.norm(v))
        for q in np.linspace(-2,2,20):
            r=u*np.cos(q)+v*np.sin(q)
            self.assertAlmostEqual(np.linalg.norm(r),np.linalg.norm(u))
            self.assertAlmostEqual(np.dot(axis,r),0.)

    def test_synthetic_five_coordinate_recovery_and_holdout(self):
        model=FiveDofPotential();rng=np.random.default_rng(55)
        q=rng.uniform(-.5,.7,(60,5));x=np.stack([model.orbit_regressor(z) for z in q])
        h,_=rigid_parameters([.12,.02,-.03,.04,-.01,.6,.3])
        y=x@h
        recovered=np.linalg.lstsq(x[:40].reshape(-1,9),y[:40].ravel(),rcond=1e-9)[0]
        np.testing.assert_allclose(recovered,h,atol=1e-10)
        np.testing.assert_allclose(x[40:]@recovered,y[40:],atol=1e-10)

    def test_measured_windows_and_independent_split(self):
        data=read(HERE/'five_dof_result.json')
        self.assertEqual(sum(data['accepted_pose_counts'].values()),73)
        self.assertEqual(data['accepted_pose_counts']['J5'],6)
        self.assertIn('J5_3',data['holdout_pose_ids']);self.assertIn('J5_4',data['holdout_pose_ids'])
        self.assertNotIn('J5_3',data['train_pose_ids'])
        self.assertGreater(data['J5_zero_pair']['half_difference_nm'],.7)
        self.assertFalse(data['deployable'])
        self.assertTrue(data['old_fixed_load_candidate_superseded_for_J5_motion'])

    def test_invalid_rigid_orbit_not_promoted(self):
        data=read(HERE/'five_dof_result.json')
        self.assertFalse(data['variants'][0]['orbit_diagnostics']['necessary_rigid_circle_condition_passed'])
        self.assertIsNone(read(HERE/'five_dof_offline_candidates.json')['selected_for_deployment'])
        self.assertFalse(read(HERE/'five_dof_offline_candidates.json')['deployable'])

    def test_readback_watchdog_evidence_has_no_writes(self):
        p=HERE.parent/'acquisition_20261109/artifacts/evidence_l_verify_existing.json'
        d=read(p)
        self.assertEqual(d['write_outcome'],'verified_existing')
        self.assertEqual(d['timeout_before'],2000);self.assertEqual(d['timeout_after'],2000)
        self.assertEqual(d['write_attempts'],0);self.assertEqual(d['parameter_writes'],0)


if __name__=='__main__':unittest.main()
