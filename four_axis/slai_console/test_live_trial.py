"""Production limited-trial path with synthetic USB, clock and real audit callback."""
import io
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from . import live_trial as app
from . import test_watchdog_check as fixtures
from .watchdog_check import make_event_writer
from .actuation import FourAxisController, LimitedJ4Trial
from four_axis_monitor import load_mapping
from .test_reader_fixture import make_reader


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=fixtures.Clock();self.mapping=load_mapping()
        self.bus=fixtures.WatchdogBus(self.mapping,self.clock)
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        self.cancel=threading.Event()
        self.temp=tempfile.TemporaryDirectory(ignore_cleanup_errors=True,dir=app.ROOT/'tmp/watchdog_check')
        self.report=Path(self.temp.name)/'synthetic.json'
        self.report.write_text(json.dumps(dict(joint=4,watchdog_d_observed=True,
            provenance='SYNTHETIC_TEST_NOT_HARDWARE_EVIDENCE')),encoding='utf-8')
        self.path_patch=patch.object(app,'WATCHDOG_REPORT',self.report);self.path_patch.start()
        self.handle=io.StringIO();self.published=[]
        self.audit=make_event_writer(self.handle,self.published.append)
        self.request=app.TrialRequest(True,True,True,'synthetic-request')
        app.fill_preflight(self.reader,self.cancel,self.clock.now,self.clock.sleep)

    def tearDown(self):
        self.reader.close();self.path_patch.stop();self.temp.cleanup()

    def run_trial(self):
        return app.run_trial(self.reader,self.request,self.audit,self.cancel,
                             clock=self.clock.now,sleep=self.clock.sleep)

    def test_production_path_supported_j4_hold_then_fresh_disable(self):
        result=self.run_trial()
        self.assertTrue(result.get('position_hold_criterion_met'),result)
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertEqual(result['enabled_observed_axes'],[4])
        self.assertFalse(result['full_arm_accepted'])
        self.assertFalse(result['physical_torque_off_verified'])
        frames=self.bus.history
        self.assertEqual([cid for _,cid,p in frames if p==b'\xff'*7+b'\xfc'],[4])
        self.assertTrue(all(cid in (1,2,3,4,0x7FF) for _,cid,_ in frames))
        for _,cid,p in frames:
            if cid==0x7FF:self.assertIn(p[2],(0x33,0xCC))
            elif p[:7]!=b'\xff'*7:
                self.assertEqual(cid,4)
                self.assertEqual(((p[3]&15)<<8)|p[4],0)
                self.assertEqual((p[5]<<4)|(p[6]>>4),0)
                tau=(((p[6]&15)<<8)|p[7])*108/4095-54
                self.assertLessEqual(abs(tau),.3)
        events=[json.loads(line) for line in self.handle.getvalue().splitlines()]
        self.assertTrue(any(r['kind']=='SCOPED_HOLD_OBSERVED' for r in events))
        self.assertTrue(any(r['kind']=='SERIAL_WRITTEN' and r['frame_kind']=='enable' for r in events))
        self.assertTrue(any(r['kind']=='PD_TIER_SELECTED' for r in events))
        self.assertTrue(any(r['kind']=='TRIAL_METRICS' for r in events))

    def test_default_tier_is_verified_kp4_in_result_and_log(self):
        result=self.run_trial()
        self.assertEqual(result['pd_tier'],'kp4_verified')
        self.assertEqual(result['host_kp'],4.)
        self.assertTrue(result['pd_tier_hardware_validated'])
        self.assertEqual(result['torque_cap_nm'],.3)
        self.assertAlmostEqual(result['encoded_torque_cap_nm'],10.5*108/4095,places=12)
        self.assertEqual(result['motion_metrics']['assessment'],'NO_MOTION_REQUIRED')
        self.assertEqual(result['stop_reason_code'],'HOLD_COMPLETE')
        events=[json.loads(line) for line in self.handle.getvalue().splitlines()]
        selected=next(r for r in events if r['kind']=='PD_TIER_SELECTED')
        self.assertEqual(selected['pd_tier'],'kp4_verified')
        self.assertEqual(selected['host_kp'],4.)
        self.assertTrue(selected['is_default'])
        self.assertFalse(selected['explicit_tier_ack'])
        started=next(r for r in events if r['kind']=='TRIAL_STARTED')
        self.assertEqual(started['candidate_host_kp'],4.)
        self.assertTrue(started['pd_tier_explicitly_selected'])

    def test_stiff_tier_requires_ack_then_hold_records_kp12(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-stiff','hold','kp12_candidate')
        result=self.run_trial()
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
        self.request=app.TrialRequest(True,True,True,'synthetic-stiff','hold','kp12_candidate',True)
        result=self.run_trial()
        self.assertTrue(result.get('position_hold_criterion_met'),result)
        self.assertEqual(result['pd_tier'],'kp12_candidate')
        self.assertEqual(result['host_kp'],12.)
        self.assertFalse(result['pd_tier_hardware_validated'])
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertEqual(result['enabled_observed_axes'],[4])
        events=[json.loads(line) for line in self.handle.getvalue().splitlines()]
        selected=[r for r in events if r['kind']=='PD_TIER_SELECTED']
        self.assertTrue(selected)
        self.assertTrue(all(r['host_kp']==12. for r in selected))
        self.assertTrue(any(r['explicit_tier_ack'] for r in selected))
        started=[r for r in events if r['kind']=='TRIAL_STARTED']
        self.assertTrue(all(r['candidate_host_kp']==12. for r in started))
        self.assertTrue(all(r['candidate_host_kd']==.5 for r in started))
        self.assertEqual([cid for _,cid,p in self.bus.history if p==b'\xff'*7+b'\xfc'],[4])

    def test_brake_tier_requires_ack_then_hold_records_bounded_profile(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-brake','hold',
                                      'kp32_2nm_brake_candidate')
        refused=self.run_trial()
        self.assertIn('error',refused)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
        self.request=app.TrialRequest(True,True,True,'synthetic-brake','hold',
                                      'kp32_2nm_brake_candidate',True)
        result=self.run_trial()
        self.assertTrue(result.get('position_hold_criterion_met'),result)
        self.assertEqual(result['torque_cap_nm'],2.)
        self.assertEqual(result['host_kp'],32.)
        profile=result['candidate_control']
        self.assertEqual(profile['slew_up_nm_s'],1.)
        self.assertEqual(profile['slew_down_nm_s'],10.)
        self.assertEqual(profile['startup_cap_nm'],1.)
        self.assertEqual(profile['startup_window_s'],6.)
        self.assertEqual(profile['brake_cap_nm'],.4)
        self.assertEqual([cid for _,cid,p in self.bus.history if p==b'\xff'*7+b'\xfc'],[4])
        events=[json.loads(line) for line in self.handle.getvalue().splitlines()]
        started=next(r for r in events if r['kind']=='TRIAL_STARTED')
        self.assertEqual(started['torque_slew_up_nm_s'],1.)
        self.assertEqual(started['torque_slew_down_nm_s'],10.)
        self.assertEqual(started['startup_cap_nm'],1.)
        self.assertEqual(started['brake_gain_nm_per_rad_s'],2.)
        selected=next(r for r in events if r['kind']=='PD_TIER_SELECTED')
        self.assertFalse(selected['is_default'])
        self.assertTrue(selected['explicit_tier_ack'])

    def test_missing_operator_confirmation_blocks_without_enable(self):
        self.request=app.TrialRequest(True,False,True,'synthetic-request')
        result=self.run_trial()
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_pre_start_cancel_sends_no_enable(self):
        self.cancel.set();result=self.run_trial()
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_no_d_observation_record_blocks_without_enable(self):
        self.report.write_text(json.dumps(dict(joint=4,watchdog_d_observed=False)),encoding='utf-8')
        result=self.run_trial();self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_live_pv_or_timeout_zero_blocks(self):
        for rid,value in ((10,2),(9,0)):
            old=self.reader.state.regs[1][rid]
            self.reader.state.regs[1][rid]=value
            result=self.run_trial();self.assertIn('error',result)
            self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
            self.reader.state.regs[1][rid]=old

    def test_limited_authority_cannot_enable_other_axes_or_trajectory(self):
        session=app.build_session(self.reader,self.request,self.audit,self.clock.now)
        c=session.controller
        with self.assertRaises(ValueError):
            FourAxisController(session.endpoint,c.envelopes,c.acceptance,self.audit,
                               active_axes=(1,2,3,4),max_runtime=2.)
        with self.assertRaises(ValueError):
            FourAxisController(session.endpoint,c.envelopes,c.acceptance,self.audit,
                               active_axes=(4,),max_runtime=10.)
        session.start()
        with self.assertRaises(ValueError):c.trajectory([0]*4,2.,self.clock.now())
        self.assertEqual(self.bus.states,{1:0,2:0,3:0,4:0})

    def test_degree_branch_never_automatically_wraps_or_returns_zero(self):
        self.reader.state.feedback[3]['position_rad']+=6.283185307179586
        result=self.run_trial();self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_cancel_after_enable_attempts_global_disable(self):
        original=self.audit
        def audit(event,**fields):
            original(event,**fields)
            if event=='SERIAL_WRITTEN' and fields.get('kind')=='enable':self.cancel.set()
        self.audit=audit
        result=self.run_trial()
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertFalse(result['position_hold_criterion_met'])
        self.assertEqual(result['reason'],'operator_stop')
        self.assertEqual({cid for _,cid,p in self.bus.history if p==b'\xff'*7+b'\xfd'},{1,2,3,4})

    def test_captured_stationary_j3_speed_bin_does_not_use_command_speed_cap(self):
        original=self.bus.feedback
        def feedback(i):
            raw=original(i)
            if i==3:
                # Actual failing payload: 03 67 77 7f e7 fe 1c 1b;
                # retain fixture position, replay its VEL12=0x7FE (-.032967).
                raw=raw[:3]+bytes([0x7f,0xe0|(raw[4]&15)])+raw[5:]
            return raw
        self.bus.feedback=feedback
        app.fill_preflight(self.reader,self.cancel,self.clock.now,self.clock.sleep)
        result=self.run_trial()
        self.assertTrue(result.get('position_hold_criterion_met'),result)

    def test_high_feedback_speed_still_blocks_trial(self):
        self.reader.state.feedback[3]['velocity_rad_s']=-.15
        result=self.run_trial()
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))


if __name__=='__main__':unittest.main()
