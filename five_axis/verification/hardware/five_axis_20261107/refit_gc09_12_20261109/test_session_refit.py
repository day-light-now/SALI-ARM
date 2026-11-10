"""Checks for data isolation, unknown friction history, and incident provenance."""
import unittest
import numpy as np
from analyze import HERE, read, split, observed, session_design, fit_model, FiveDofPotential


class SessionRefitTests(unittest.TestCase):
    def test_clean_and_prefault_split_gc13_is_unused(self):
        poses=read(HERE/'static_windows.json')['poses']
        train,history,old,aborted=split(poses)
        self.assertEqual((len(poses),sum(train),sum(history),sum(old),sum(aborted)),(159,111,29,11,8))
        self.assertFalse(any(p['session']=='GC13' for p in poses))
        self.assertFalse(np.any(train & aborted))
        cross=read(HERE/'extraction_crosscheck.json')
        self.assertEqual(len(cross),36)
        self.assertEqual(max(r['max_q_difference_deg'] for r in cross),0.)
        self.assertEqual(max(r['max_effort_difference_nm'] for r in cross),0.)

    def test_unknown_approach_is_not_zero_friction_observation(self):
        poses=read(HERE/'static_windows.json')['poses']
        _,mask=observed(poses)
        k=next(i for i,p in enumerate(poses) if p['id']=='GC09_1')
        self.assertTrue(mask[k,0]);self.assertFalse(mask[k,3]);self.assertFalse(mask[k,4])

    def test_physical_driver_session_is_shared_across_plans(self):
        poses=read(HERE/'static_windows.json')['poses']
        groups={p['session']:p['driver_session'] for p in poses}
        self.assertEqual(groups['GC09'],groups['GC10'])
        self.assertEqual(groups['GC09'],groups['GC11_ABORTED'])
        self.assertEqual(groups['GC11'],groups['GC12'])
        self.assertNotEqual(groups['GC11_ABORTED'],groups['GC11'])

    def test_holdout_values_cannot_change_gravity_or_offsets(self):
        rng=np.random.default_rng(9012);model=FiveDofPotential()
        q=rng.uniform(-.5,.7,(70,5));x=np.stack([model.orbit_regressor(z) for z in q])
        base=np.stack([model.base(z) for z in q]);direction=rng.choice([-1,1],(70,5))
        poses=[{'driver_session':'train1' if i<25 else 'train2' if i<50 else 'unseen'} for i in range(70)]
        training=np.arange(70)<50
        session,names=session_design(poses,training)
        self.assertEqual(names,['train1','train2']);self.assertEqual(np.max(abs(session[50:])),0.)
        h=np.r_[.35,.03,-.12,np.zeros(6)]
        y=base+x@h+direction*np.array([1.1,.65,.84,.87,.8])
        y[:25]+=np.array([.1,-.1,.05,.05,0.]);y[25:50]-=np.array([.1,-.1,.05,.05,0.])
        rows=np.tile(training[:,None],(1,5))
        a,_,_,_=fit_model(x,base,y,direction,rows,session,shape='fixed',sigma=.15)
        y[50:]+=rng.normal(0,1000,(20,5))
        b,_,_,_=fit_model(x,base,y,direction,rows,session,shape='fixed',sigma=.15)
        np.testing.assert_array_equal(a['h_kg_m'],b['h_kg_m'])
        np.testing.assert_array_equal(a['session_offsets_nm'],b['session_offsets_nm'])

    def test_no_model_is_deployable(self):
        r=read(HERE/'session_refit_result.json')
        self.assertFalse(r['deployable']);self.assertFalse(r['GC13_used'])
        self.assertIsNone(r['selected_for_deployment'])
        for m in r['models']+r['paired_only_models']:
            self.assertFalse(m['deployable']);self.assertTrue(m['optimizer_success'])
        self.assertEqual(r['paired_only_models'][0]['training_pair_count'],17)

    def test_fresh_motor_state13_is_not_the_stale_client_state1(self):
        r=read(HERE/'gc12_incident_review.json')
        self.assertEqual(set(r['client_last_published_motor_states'].values()),{1.})
        self.assertEqual(set(r['later_readback_states'].values()),{13})
        self.assertEqual(r['later_readback_states'],r['later_readback_payload_decoded_states'])
        self.assertFalse(r['client_confirmed_fresh_disable'])
        self.assertFalse(r['physical_torque_off_proven_by_state_code'])
        self.assertEqual(r['last_cycle_written_can_ids'],[1,2,3,4,5])
        self.assertEqual(r['last_control_cycle']['cycle_id'],10541)
        self.assertEqual(r['invalid_json_lines'],0)
        self.assertFalse(r['root_cause_determined'])


if __name__=='__main__':unittest.main()
