import math
import unittest
from .auto_load_acquisition import build_plan,AcquisitionLedger


class Tests(unittest.TestCase):
    def plan(self,**overrides):
        args=dict(joint=1,initial_pose_rad=(0.,0.,0.,0.),mechanical_limits_deg=(-30.,30.),
            mapping_sha256='a'*64,physical_limit_evidence='synthetic measured-limit fixture',
            physical_limits_confirmed=True)
        args.update(overrides)
        return build_plan(**args)

    def test_margin_finite_bidirectional_route_and_return(self):
        p=self.plan();self.assertEqual(p.scan_limits_deg,(-20.,20.))
        self.assertEqual(p.points[-1].target_pose_rad,p.initial_pose_rad)
        for point in p.points:
            self.assertEqual(point.target_pose_rad[1:],(0.,0.,0.))
            self.assertLessEqual(abs(math.degrees(point.target_pose_rad[0])),20.)
        approaches={}
        previous=p.initial_pose_rad[0]
        for point in p.points:
            target=point.target_pose_rad[0]
            self.assertLessEqual(1.875*abs(target-previous)/point.duration_s,.02+1e-12)
            if point.collect:approaches.setdefault(round(math.degrees(target),5),set()).add(point.approach)
            previous=target
        for q,directions in approaches.items():
            if min(approaches)<q<max(approaches):self.assertEqual(directions,{'positive','negative'})
        self.assertFalse(p.serializable()['hardware_motion_authorized'])

    def test_reject_unmeasured_or_model_mismatched_or_outside_start(self):
        for args in ({'physical_limits_confirmed':False},{'physical_limit_evidence':''},
            {'mechanical_limits_deg':(-100.,200.)}, {'measured_limit_override':True},
            {'mechanical_limits_deg':(20.,60.)},{'mechanical_limits_deg':(-5.,5.)},
            {'step_deg':0.},{'step_deg':1e-300},{'speed_rad_s':.021},
            {'margin_deg':9.},{'max_budget_s':10.},{'joint':5}):
            with self.subTest(args=args),self.assertRaises(ValueError):self.plan(**args)

    def test_plan_binds_zero_sign_digest_and_route(self):
        p=self.plan();self.assertNotEqual(p.fingerprint,self.plan(mapping_sha256='b'*64).fingerprint)
        self.assertNotEqual(p.fingerprint,self.plan(step_deg=10.).fingerprint)

    def test_complete_requires_fresh_arrivals_records_and_stop(self):
        p=self.plan();ledger=AcquisitionLedger(p)
        while True:
            point=ledger.next_move('a'*64)
            if point is None:break
            ledger.arrived(fresh=True,within_tolerance=True)
            if point.collect:
                ledger.record(dict(accepted=True,joint=1,mapping_sha256='a'*64,
                    pose_rad=point.target_pose_rad,holding_request_estimate_model_nm=.2))
        self.assertEqual(ledger.phase,'AWAITING_STOP')
        ledger.confirm_stop(True);self.assertEqual(ledger.phase,'COMPLETE')
        self.assertFalse(ledger.checkpoint()['full_range_gravity_model_validated'])
        self.assertFalse(ledger.checkpoint()['automatic_resume'])

    def test_stop_fault_and_mapping_change_do_not_auto_resume(self):
        ledger=AcquisitionLedger(self.plan());ledger.next_move('a'*64)
        with self.assertRaises(ValueError):ledger.arrived(fresh=False,within_tolerance=True)
        self.assertEqual(ledger.phase,'HALTED')
        with self.assertRaises(ValueError):ledger.next_move('a'*64)
        ledger=AcquisitionLedger(self.plan())
        with self.assertRaises(ValueError):ledger.next_move('b'*64)
        self.assertEqual(ledger.phase,'HALTED')

    def test_failed_or_wrong_joint_sample_halts_without_counting(self):
        for data in ({'accepted':False},{'accepted':True,'joint':2},
            {'accepted':True,'joint':1,'mapping_sha256':'b'*64},
            {'accepted':True,'joint':1,'mapping_sha256':'a'*64,'pose_rad':(0.,0.,0.,0.)}):
            ledger=AcquisitionLedger(self.plan())
            while True:
                point=ledger.next_move('a'*64)
                ledger.arrived(fresh=True,within_tolerance=True)
                if point.collect:break
            with self.assertRaises(ValueError):ledger.record(data)
            self.assertEqual(ledger.phase,'HALTED');self.assertFalse(ledger.samples)


if __name__=='__main__':unittest.main()
