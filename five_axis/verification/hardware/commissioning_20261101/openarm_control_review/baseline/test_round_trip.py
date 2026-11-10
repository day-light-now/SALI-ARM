"""Synthetic slow joint plant plus production round-trip controller/audit path."""
import io
import json
import math
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from . import live_trial as app
from . import test_watchdog_check as fixtures
from .watchdog_check import make_event_writer
from four_axis_monitor import Reader,load_mapping


class Plant(fixtures.WatchdogBus):
    """Simple inertia/viscous joint driven ONLY by decoded transmitted torque.

    Does not receive target positions from the test controller; static-plant
    failure is tested separately to prevent command playback counting as motion.
    """
    def __init__(self,mapping,clock):
        super().__init__(mapping,clock)
        self.q=0.;self.v=0.;self.tau=0.;self.at=clock.now()

    def advance(self):
        now=self.clock.now();remaining=now-self.at;self.at=now
        while remaining>1e-12:
            dt=min(.001,remaining);remaining-=dt
            if self.states[4]==1:
                self.v+=(self.tau-.1*self.v)/.05*dt
                self.q+=self.v*dt
            else:self.v=0.

    def feedback(self,i):
        if i!=4:return super().feedback(i)
        self.advance();sign=self.mapping[4]['hardware_sign']
        position=self.mapping[4]['zero_offset_rad']+sign*self.q
        raw=round((position+12.5)*65535/25)
        vel=round((sign*self.v+45)*4095/90)
        tau=round((sign*self.tau+54)*4095/108)
        return bytes([self.states[4]*16+4,raw>>8,raw&255,vel>>4,
                      ((vel&15)<<4)|(tau>>8),tau&255,25,25])

    def write(self,frame):
        self.advance()
        cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        if cid==4 and p[:7]!=b'\xff'*7:
            self.tau=self.mapping[4]['hardware_sign']*((((p[6]&15)<<8)|p[7])*108/4095-54)
        return super().write(frame)


class StictionPlant(Plant):
    """Same joint with a synthetic Coulomb stiction load; not a hardware claim.

    A test sets the stiction between the Kp=4 peak and the 0.30Nm encoded cap to
    reproduce 'the command never asked for the budget', and above the encoded
    cap to reproduce 'the limit was reached and it still did not move'.
    """
    def __init__(self,mapping,clock,stiction):
        super().__init__(mapping,clock)
        self.stiction=float(stiction)

    def advance(self):
        now=self.clock.now();remaining=now-self.at;self.at=now
        while remaining>1e-12:
            dt=min(.001,remaining);remaining-=dt
            if self.states[4]==1:
                if abs(self.tau)>self.stiction:
                    self.v+=(self.tau-math.copysign(self.stiction,self.tau)-.1*self.v)/.05*dt
                else:self.v=0.
                self.q+=self.v*dt
            else:self.v=0.


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=fixtures.Clock();self.mapping=load_mapping()
        self.temp=tempfile.TemporaryDirectory(dir=app.ROOT/'tmp/live_trial')
        self.hold=Path(self.temp.name)/'synthetic_hold.json'
        self.hold.write_text(json.dumps(dict(trial='supported_J4_current_pose_hold_2s',
            position_hold_criterion_met=True,fresh_stop_confirmation=True,
            final_controller_state='DISABLED',enabled_observed_axes=[4],
            provenance='SYNTHETIC_NOT_HARDWARE')),encoding='utf-8')
        self.patch=patch.object(app,'HOLD_REPORT',self.hold);self.patch.start()
        self.cancel=threading.Event();self.handle=io.StringIO()
        self.audit=make_event_writer(self.handle,lambda row:None)
        self.request=app.TrialRequest(True,True,True,'synthetic-trip','roundtrip10')
        self.reader=None

    def setup_bus(self,moving=True,stiction=None):
        if stiction is not None:
            self.bus=StictionPlant(self.mapping,self.clock,stiction)
        else:
            self.bus=(Plant(self.mapping,self.clock) if moving
                      else fixtures.WatchdogBus(self.mapping,self.clock))
        self.reader=Reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        app.fill_preflight(self.reader,self.cancel,self.clock.now,self.clock.sleep)

    def audit_events(self):
        return [json.loads(line) for line in self.handle.getvalue().splitlines()]

    def encoded_torques(self):
        values=[]
        for _,cid,p in self.bus.history:
            if cid==4 and p[:7]!=b'\xff'*7:
                values.append((((p[6]&15)<<8)|p[7])*108/4095-54)
        return values

    def tearDown(self):
        if self.reader:self.reader.close()
        self.patch.stop();self.temp.cleanup()

    def run_trial(self):
        return app.run_trial(self.reader,self.request,self.audit,self.cancel,
                             clock=self.clock.now,sleep=self.clock.sleep)

    def test_actual_plant_moves_ten_degrees_and_returns_then_disables(self):
        self.setup_bus();result=self.run_trial()
        self.assertTrue(result.get('round_trip_criterion_met'),result)
        self.assertGreaterEqual(result['actual_positive_excursion_deg'],9.5)
        self.assertLessEqual(abs(result['final_error_from_start_deg']),.5)
        self.assertEqual(result['enabled_observed_axes'],[4])
        self.assertEqual(result['reason'],'round_trip_complete')
        self.assertTrue(result['fresh_stop_confirmation'])
        for _,cid,p in self.bus.history:
            if cid==4 and p[:7]!=b'\xff'*7:
                tau=(((p[6]&15)<<8)|p[7])*108/4095-54
                self.assertLessEqual(abs(tau),.3)
            elif p==b'\xff'*7+b'\xfc':self.assertEqual(cid,4)

    def test_stationary_support_cannot_count_as_successful_round_trip(self):
        self.setup_bus(moving=False);result=self.run_trial()
        self.assertFalse(result.get('round_trip_criterion_met'),result)
        self.assertLess(result['actual_positive_excursion_deg'],.1)
        self.assertFalse(result['outward_reached'])
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertEqual(result['reason'],'Following error')
        metrics=result['motion_metrics']
        self.assertFalse(metrics['moved'])
        self.assertTrue(metrics['stalled'])
        self.assertEqual(metrics['position_code_changes'],0)
        self.assertEqual(metrics['assessment'],'COMMAND_BELOW_CAP_NO_MOTION')
        self.assertEqual(result['stop_reason_code'],'FOLLOWING_ERROR_BELOW_CAP_NO_MOTION')

    def test_kp4_stuck_joint_reports_below_cap_not_only_generic_following_error(self):
        self.setup_bus(stiction=.20)  # above the Kp=4 peak, below the encoded cap
        result=self.run_trial()
        self.assertFalse(result['round_trip_criterion_met'])
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertEqual(result['reason'],'Following error')
        metrics=result['motion_metrics']
        self.assertFalse(metrics['moved'])
        self.assertTrue(metrics['stalled'])
        self.assertLess(metrics['total_span_deg'],.05)
        self.assertEqual(metrics['position_code_changes'],0)
        self.assertLess(metrics['peak_encoded_nm'],.20)
        self.assertLess(metrics['peak_fraction_of_cap'],.6)
        self.assertLess(metrics['peak_fraction_of_encoded_cap'],.6)
        self.assertEqual(metrics['cycles_at_encoded_cap'],0)
        self.assertEqual(metrics['assessment'],'COMMAND_BELOW_CAP_NO_MOTION')
        self.assertEqual(result['stop_reason_code'],'FOLLOWING_ERROR_BELOW_CAP_NO_MOTION')
        self.assertIn('没有用满',result['next_step_hint'])
        self.assertTrue(result['no_auto_output_increase'])
        events=self.audit_events()
        logged=next(r for r in events if r['kind']=='TRIAL_METRICS')
        self.assertEqual(logged['assessment'],'COMMAND_BELOW_CAP_NO_MOTION')
        self.assertEqual(logged['stop_reason_code'],'FOLLOWING_ERROR_BELOW_CAP_NO_MOTION')
        self.assertEqual(logged['position_codes'],metrics['position_codes'])

    def test_stiff_tier_stuck_joint_reaches_encoded_cap_and_still_reports_no_motion(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-stiff','roundtrip10',
                                      'kp12_candidate',True)
        self.setup_bus(stiction=.40)  # above the encoded cap
        result=self.run_trial()
        self.assertFalse(result['round_trip_criterion_met'])
        self.assertEqual(result['reason'],'Following error')
        metrics=result['motion_metrics']
        self.assertFalse(metrics['moved'])
        self.assertTrue(metrics['stalled'])
        self.assertGreater(metrics['cycles_at_encoded_cap'],0)
        self.assertGreater(metrics['time_at_encoded_cap_s'],.5)
        self.assertAlmostEqual(metrics['peak_encoded_nm'],metrics['encoded_cap_nm'],places=12)
        self.assertLessEqual(metrics['peak_encoded_nm'],.3)
        self.assertEqual(metrics['assessment'],'OUTPUT_AT_ENCODED_CAP_NO_MOTION')
        self.assertEqual(result['stop_reason_code'],'FOLLOWING_ERROR_AT_ENCODED_CAP_NO_MOTION')
        self.assertIn('编码力矩上限',result['next_step_hint'])
        torques=self.encoded_torques()
        self.assertTrue(torques)
        self.assertLessEqual(max(abs(t) for t in torques),.3+1e-12)
        self.assertGreaterEqual(max(abs(t) for t in torques),.2769-1e-12)
        events=self.audit_events()
        started=next(r for r in events if r['kind']=='TRIAL_STARTED')
        self.assertEqual(started['candidate_host_kp'],12.)
        self.assertEqual(started['pd_tier'],'kp12_candidate')
        self.assertFalse(started['pd_tier_hardware_validated'])
        selected=next(r for r in events if r['kind']=='PD_TIER_SELECTED')
        self.assertEqual(selected['host_kp'],12.)
        self.assertTrue(selected['explicit_tier_ack'])
        self.assertFalse(selected['is_default'])

    def test_stiff_tier_moving_plant_still_requires_real_travel_and_return(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-stiff','roundtrip10',
                                      'kp12_candidate',True)
        self.setup_bus();result=self.run_trial()
        self.assertTrue(result.get('round_trip_criterion_met'),result)
        self.assertGreaterEqual(result['actual_positive_excursion_deg'],9.5)
        self.assertLessEqual(abs(result['final_error_from_start_deg']),.5)
        self.assertEqual(result['stop_reason_code'],'ROUND_TRIP_COMPLETE')
        self.assertEqual(result['motion_metrics']['assessment'],'MOVED')
        self.assertLessEqual(max(abs(t) for t in self.encoded_torques()),.3+1e-12)

    def test_stiffer_tier_requires_separate_ack_before_any_enable(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-stiff','roundtrip10',
                                      'kp12_candidate')
        self.setup_bus(stiction=.40)
        result=self.run_trial()
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_one_nm_requires_ack_and_static_axis_still_stops(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-1nm','roundtrip10',
                                      'kp32_1nm_candidate',False)
        self.setup_bus(moving=False)
        refused=self.run_trial()
        self.assertIn('error',refused)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
        self.request=app.TrialRequest(True,True,True,'synthetic-1nm','roundtrip10',
                                      'kp32_1nm_candidate',True)
        result=self.run_trial()
        self.assertEqual(result['host_kp'],32.)
        self.assertEqual(result['torque_cap_nm'],1.)
        self.assertEqual(result['reason'],'Following error')
        self.assertFalse(result['round_trip_criterion_met'])
        self.assertTrue(result['fresh_stop_confirmation'])
        torques=self.encoded_torques()
        self.assertLessEqual(max(abs(t) for t in torques),1.)
        self.assertGreater(max(abs(t) for t in torques),.9)
        self.assertEqual([cid for _,cid,p in self.bus.history if p==b'\xff'*7+b'\xfc'],[4])
        started=next(r for r in self.audit_events() if r['kind']=='TRIAL_STARTED')
        self.assertEqual(started['candidate_cap_nm'],1.)
        self.assertEqual(started['torque_slew_nm_s'],.5)
        self.assertEqual(started['feedback_speed_abort_rad_s'],.2)

    def test_two_nm_requires_ack_and_static_axis_still_stops(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-2nm','roundtrip10',
                                      'kp32_2nm_candidate',False)
        self.setup_bus(moving=False)
        refused=self.run_trial()
        self.assertIn('error',refused)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
        self.request=app.TrialRequest(True,True,True,'synthetic-2nm','roundtrip10',
                                      'kp32_2nm_candidate',True)
        result=self.run_trial()
        self.assertEqual(result['host_kp'],32.)
        self.assertEqual(result['torque_cap_nm'],2.)
        self.assertEqual(result['reason'],'Following error')
        self.assertFalse(result['round_trip_criterion_met'])
        self.assertTrue(result['fresh_stop_confirmation'])
        torques=self.encoded_torques()
        self.assertLessEqual(max(abs(t) for t in torques),2.)
        self.assertGreater(max(abs(t) for t in torques),1.9)
        self.assertEqual([cid for _,cid,p in self.bus.history if p==b'\xff'*7+b'\xfc'],[4])
        started=next(r for r in self.audit_events() if r['kind']=='TRIAL_STARTED')
        self.assertEqual(started['candidate_cap_nm'],2.)
        self.assertEqual(started['torque_slew_nm_s'],1.)
        self.assertEqual(started['feedback_speed_abort_rad_s'],.2)
        self.assertEqual(started['following_error_abort_deg'],4.)


    def test_unknown_tier_cannot_silently_use_a_gain(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-trip','roundtrip10','kp7')
        self.setup_bus()
        result=self.run_trial()
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
        self.assertEqual(app.TrialRequest(True,True,True,'x').tier,'kp4_verified')

    def test_exact_frozen_ten_degree_sequence_only(self):
        self.setup_bus();session=app.build_session(self.reader,self.request,self.audit,self.clock.now)
        c=session.controller;session.start();c.state='HOLD'
        targets=list(c.anchor);targets[3]+=math.radians(10)
        for bad in ([x+.01 for x in targets],list(c.anchor),targets[:3]+[targets[3]+.01]):
            with self.assertRaises(ValueError):c.trajectory(bad,18.,self.clock.now())
        with self.assertRaises(ValueError):c.trajectory(targets,1.,self.clock.now())
        c.trajectory(targets,18.,self.clock.now())
        c.state='HOLD';c.plan=None;c.targets=tuple(targets)
        with self.assertRaises(ValueError):c.trajectory(targets,18.,self.clock.now())
        c.trajectory(c.anchor,18.,self.clock.now())
        c.state='HOLD';c.plan=None
        with self.assertRaises(ValueError):c.trajectory(c.anchor,18.,self.clock.now())
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_no_successful_hold_record_blocks_before_enable(self):
        self.setup_bus();self.hold.write_text('{}',encoding='utf-8')
        result=self.run_trial();self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_forward_path_exceeding_model_margin_blocks_before_enable(self):
        self.setup_bus()
        self.reader.state.feedback[4]['position_rad']=self.mapping[4]['zero_offset_rad']-math.radians(45)
        result=self.run_trial();self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_cancel_mid_outward_does_not_force_return(self):
        self.setup_bus();old=self.clock.sleep
        def sleep(dt):
            old(dt)
            if self.bus.q>math.radians(3):self.cancel.set()
        result=app.run_trial(self.reader,self.request,self.audit,self.cancel,
                             clock=self.clock.now,sleep=sleep)
        self.assertFalse(result['round_trip_criterion_met'])
        self.assertFalse(result['return_reached'])
        self.assertEqual(result['reason'],'operator_stop')
        self.assertEqual(result['final_controller_state'],'DISABLED')


if __name__=='__main__':unittest.main()
