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
from four_axis_monitor import load_mapping
from .test_reader_fixture import make_reader


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


class QueryOnlyCoastBus(fixtures.WatchdogBus):
    """After the stop, a J4 0x7FF 04 00 CC query is the only thing that returns
    a state frame; each query advances a discrete coast. No spontaneous frames.

    This makes the post-stop test path a real request-reply path: a passive
    drain will not fabricate movement, and the capture must keep the visible
    frames produced by the CC polls.
    """
    def __init__(self,mapping,clock,codes,states0=True):
        super().__init__(mapping,clock)
        self.coast_codes=list(codes)
        self.coast_index=0
        self.cc_queries=[]
        self.states0=states0
        self.after_stop=False

    def _j4_frame(self,code):
        return bytes([self.states[4]*16+4,code>>8,code&255,0x7f,0xf7,0xff,25,25])

    def _append(self,payload):
        self.buf.extend(bytes([0xAA,0x11,8,0,0,0,0])+payload+b'\x55')

    def write(self,frame):
        cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        if cid==4 and p==b'\xff'*7+b'\xfd':
            self.after_stop=True
            return super().write(frame)
        if (self.after_stop and cid==0x7FF and p[:3]==bytes([4,0,0xCC])
                and self.states[4]==0):
            self.cc_queries.append((self.clock.now(),cid,p.hex(' ')))
            if self.states0 and self.coast_codes:
                code=self.coast_codes[min(self.coast_index,len(self.coast_codes)-1)]
                self.coast_index+=1
                self._append(self._j4_frame(code))
            return len(frame)
        return super().write(frame)


class CacheOverwriteBus(QueryOnlyCoastBus):
    """One J4 CC reply contains two state frames; the reader keeps only the last.

    The test uses this to prove the report does not call the cached last frame a
    guaranteed first sample.
    """
    def write(self,frame):
        cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        if cid==4 and p==b'\xff'*7+b'\xfd':
            self.after_stop=True
            return super().write(frame)
        if (self.after_stop and cid==0x7FF and p[:3]==bytes([4,0,0xCC])
                and self.states[4]==0):
            self.cc_queries.append((self.clock.now(),cid,p.hex(' ')))
            first=self.coast_codes[min(self.coast_index,len(self.coast_codes)-1)]
            second=self.coast_codes[min(self.coast_index+1,len(self.coast_codes)-1)]
            self.coast_index+=2
            self._append(self._j4_frame(first))
            self._append(self._j4_frame(second))
            return len(frame)
        return super().write(frame)


class NoDisableBus(fixtures.WatchdogBus):
    """FD is recorded but J4 never reports state 0; the stop confirmation times
    out to UNKNOWN. The post-stop code must not send any active query then."""
    def write(self,frame):
        cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        if cid==4 and p==b'\xff'*7+b'\xfd':
            self.frames.append(frame)
            return len(frame)
        return super().write(frame)


class DriftOnQueryPlant(Plant):
    """A successful round trip, then a visible post-stop position offset only
    after enough J4 CC queries have been observed (i.e. after DISABLED)."""
    def __init__(self,mapping,clock,drift_codes=30,drift_after=8):
        super().__init__(mapping,clock)
        self.drift=False
        self.drift_codes=drift_codes
        self.drift_after=drift_after
        self.cc_count=0
        self.after_stop=False

    def write(self,frame):
        cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        if cid==4 and p==b'\xff'*7+b'\xfd':
            self.after_stop=True
        if (self.after_stop and cid==0x7FF and p[:3]==bytes([4,0,0xCC])
                and self.states[4]==0):
            self.cc_count+=1
            if self.cc_count>=self.drift_after:
                self.drift=True
        return super().write(frame)

    def feedback(self,i):
        raw=super().feedback(i)
        if i==4 and self.drift:
            d=bytearray(raw)
            code=((d[1]<<8)|d[2])-self.drift_codes
            code=max(0,min(65535,code))
            d[1]=code>>8;d[2]=code&255
            return bytes(d)
        return raw


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=fixtures.Clock();self.mapping=load_mapping()
        self.temp=tempfile.TemporaryDirectory(ignore_cleanup_errors=True,dir=app.ROOT/'tmp/live_trial')
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
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        app.fill_preflight(self.reader,self.cancel,self.clock.now,self.clock.sleep)

    def setup_custom_bus(self,bus):
        self.bus=bus
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        app.fill_preflight(self.reader,self.cancel,self.clock.now,self.clock.sleep)

    def anchor_code(self):
        return round((self.mapping[4]['zero_offset_rad']+12.5)*65535/25)

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
        self.assertEqual(result['motion_metrics']['assessment'],'EFFECTIVE_TRACKING')
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


    def test_brake_tier_moving_plant_still_requires_real_travel_and_return(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-brake','roundtrip10',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_bus();result=self.run_trial()
        self.assertTrue(result.get('round_trip_criterion_met'),result)
        self.assertGreaterEqual(result['actual_positive_excursion_deg'],9.5)
        self.assertLessEqual(abs(result['final_error_from_start_deg']),.5)
        self.assertEqual(result['stop_reason_code'],'ROUND_TRIP_COMPLETE')
        self.assertEqual(result['motion_metrics']['assessment'],'EFFECTIVE_TRACKING')
        self.assertIsNone(result['trigger_frame'])
        self.assertLessEqual(max(abs(t) for t in self.encoded_torques()),2.+1e-12)
        events=self.audit_events()
        started=next(r for r in events if r['kind']=='TRIAL_STARTED')
        self.assertEqual(started['torque_slew_up_nm_s'],1.)
        self.assertEqual(started['torque_slew_down_nm_s'],10.)
        self.assertEqual(started['startup_cap_nm'],1.)
        self.assertEqual(started['startup_window_s'],6.)
        self.assertEqual(started['brake_cap_nm'],.4)
        self.assertTrue(any(r['kind']=='TRIAL_METRICS' for r in events))

    def test_brake_tier_production_startup_stop_is_bounded_and_reported(self):
        self.request=app.TrialRequest(True,True,True,'synthetic-brake','roundtrip10',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_bus(stiction=1.2)
        result=self.run_trial()
        self.assertIn('Startup no breakaway',result['reason'])
        self.assertEqual(result['stop_reason_code'],'STARTUP_NO_BREAKAWAY')
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertTrue(result['fresh_stop_confirmation'])
        metrics=result['motion_metrics']
        self.assertIn(metrics['motion_class'],('NO_CODE_CHANGE','CODE_JITTER'))
        self.assertLess(metrics['total_span_deg'],.05)
        self.assertLessEqual(metrics['peak_encoded_nm'],1.+1e-9)
        self.assertFalse(result['round_trip_criterion_met'])
        trigger=result['trigger_frame']
        self.assertIsNotNone(trigger)
        self.assertIn('Startup no breakaway',trigger['reason'])
        self.assertIsNotNone(trigger['position_code'])
        self.assertEqual(trigger['startup_capped'],True)
        self.assertIn('保护触发帧',result['stop_summary'])
        self.assertIn('受控停止',result['next_step_hint'])
        self.assertTrue(any(r['kind']=='TRIAL_METRICS' for r in self.audit_events()))

    def test_speed_abort_reports_trigger_and_post_stop_feedback(self):
        # Inject one high J4 VEL12 while enabled; the existing protection must
        # stop at once, and the production path must retain the trigger frame and
        # the first new state-0 feedback frames.
        self.request=app.TrialRequest(True,True,True,'synthetic-speed','hold',
                                      'kp32_2nm_candidate',True)
        self.setup_bus(moving=False)
        original=self.bus.feedback
        def feedback(i):
            raw=original(i)
            if i==4 and self.bus.states[4]==1:
                # VEL12=2059 -> +0.25rad/s raw, preserving the tau nibble.
                raw=raw[:3]+bytes([0x80,0xB0|(raw[4]&0x0F)])+raw[5:]
            return raw
        self.bus.feedback=feedback
        app.fill_preflight(self.reader,self.cancel,self.clock.now,self.clock.sleep)
        result=self.run_trial()
        self.assertIn('feedback speed',result['reason'])
        self.assertEqual(result['stop_reason_code'],'FEEDBACK_SPEED_ABORT')
        trigger=result['trigger_frame']
        self.assertIsNotNone(trigger)
        self.assertGreaterEqual(trigger['velocity_rad_s'],.24)
        self.assertLess(trigger['velocity_rad_s'],.27)
        self.assertLess(trigger['model_velocity_rad_s'],0.)
        self.assertIsNotNone(trigger['position_code'])
        self.assertIn('feedback speed',trigger['reason'])
        post=result['post_stop']
        self.assertIsNotNone(post)
        self.assertGreaterEqual(post['count'],1)
        self.assertEqual(post['frames'][0]['state'],0)
        self.assertIn('停止后新反馈',result['stop_summary'])
        events=self.audit_events()
        self.assertTrue(any(r['kind']=='POST_STOP_FEEDBACK' for r in events))
        self.assertTrue(any(r['kind']=='TRIAL_METRICS' for r in events))
        self.assertEqual(result['final_controller_state'],'DISABLED')

    def test_post_stop_query_only_discrete_coast_is_visible(self):
        base=self.anchor_code()
        codes=[base-2,base-4,base-6,base-8,base-10,base-12]
        self.request=app.TrialRequest(True,True,True,'post-query','hold',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_custom_bus(QueryOnlyCoastBus(self.mapping,self.clock,codes))
        result=self.run_trial()
        post=result['post_stop']
        self.assertGreaterEqual(post['state0_count'],2,post)
        self.assertIn('cc_poll',post['sources'])
        self.assertGreater(post['observation']['tx'],0)
        self.assertGreater(post['observation']['polls'],0)
        self.assertFalse(post['first_state0_guaranteed'])
        self.assertTrue(post['may_miss_frames'])
        self.assertEqual(post['visibility'],'visible_only')
        self.assertGreater(post['travel_deg'],0.)
        self.assertIsNotNone(post['peak_speed_rad_s'])
        self.assertTrue(post['queries'])
        self.assertEqual(post['queries'][0]['kind'],'cc_j4')
        self.assertIn('04 00 cc',post['queries'][0]['payload'])
        visible_codes=[f['position_code'] for f in post['frames'] if f['state']==0]
        self.assertTrue(any(c in codes for c in visible_codes))
        self.assertTrue(any(r['kind']=='POST_STOP_FEEDBACK' for r in self.audit_events()))

    def test_post_stop_new_enabled_or_fault_feedback_revokes_disabled(self):
        class ChangedStateBus(QueryOnlyCoastBus):
            unexpected=1
            bad_query=None
            def _j4_frame(bus,code):
                payload=super()._j4_frame(code)
                if bus.coast_index>=5:
                    if bus.bad_query is None:bus.bad_query=len(bus.cc_queries)
                    return bytes([bus.unexpected*16+4])+payload[1:]
                return payload
        self.request=app.TrialRequest(True,True,True,'post-state-change','hold',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_custom_bus(ChangedStateBus(self.mapping,self.clock,[self.anchor_code()]))
        result=self.run_trial()
        self.assertEqual(result['final_controller_state'],'UNKNOWN')
        self.assertFalse(result['fresh_stop_confirmation'])
        self.assertFalse(result['position_hold_criterion_met'])
        self.assertTrue(result['post_stop_observation']['unknown_lock'])
        self.assertEqual(len(self.bus.cc_queries),self.bus.bad_query)

    def test_post_stop_no_state_feedback_is_insufficient_not_zero_coast(self):
        self.request=app.TrialRequest(True,True,True,'post-none','hold',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_custom_bus(QueryOnlyCoastBus(self.mapping,self.clock,[],states0=False))
        result=self.run_trial()
        post=result['post_stop']
        self.assertTrue(post['insufficient'])
        self.assertIn(post['insufficient_reason'],
                              ('no_new_feedback','no_new_state0_feedback',
                               'too_few_state0_frames'))
        self.assertLessEqual(post['state0_count'],1)
        self.assertIsNone(post['travel_deg'])
        self.assertIsNone(post['peak_speed_rad_s'])
        self.assertIn('不能判断滑行',result['stop_summary'])
        self.assertIn('不能据此声称0滑行',result['stop_summary'])
        self.assertGreater(post['observation']['tx'],0)

    def test_post_stop_cache_overwrite_is_visible_only_not_first(self):
        base=self.anchor_code()
        codes=[base-2,base-4,base-6,base-8,base-10,base-12]
        self.request=app.TrialRequest(True,True,True,'post-cache','hold',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_custom_bus(CacheOverwriteBus(self.mapping,self.clock,codes))
        result=self.run_trial()
        post=result['post_stop']
        self.assertFalse(post['first_state0_guaranteed'])
        self.assertTrue(post['may_miss_frames'])
        self.assertGreaterEqual(post['state0_count'],2)
        first_visible=post['first_state0']['position_code']
        self.assertNotEqual(first_visible,base-2)
        self.assertIn(first_visible,(base-4,base-6,base-8,base-10,base-12))

    def test_post_stop_unknown_sends_no_active_query(self):
        self.request=app.TrialRequest(True,True,True,'post-unknown','hold',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_custom_bus(NoDisableBus(self.mapping,self.clock))
        result=self.run_trial()
        self.assertEqual(result['final_controller_state'],'UNKNOWN')
        post=result['post_stop']
        self.assertEqual(post['observation']['tx'],0)
        self.assertTrue(post['observation']['unknown_lock'])
        self.assertFalse(post['observation']['active_query'])
        self.assertTrue(post['insufficient'])

    def test_post_stop_visible_drift_revokes_round_trip(self):
        self.request=app.TrialRequest(True,True,True,'post-drift','roundtrip10',
                                      'kp32_2nm_brake_candidate',True)
        self.setup_custom_bus(DriftOnQueryPlant(self.mapping,self.clock,
                                                drift_codes=30,drift_after=8))
        result=self.run_trial()
        self.assertTrue(result.get('outward_reached'),result.get('reason'))
        self.assertTrue(result.get('return_reached'),result.get('reason'))
        self.assertFalse(result.get('round_trip_criterion_met'))
        self.assertTrue(result.get('post_stop_invalidates_return'))
        self.assertGreater(abs(result.get('post_stop_final_error_deg') or 0.),.5)
        self.assertIn('revoked',result.get('post_stop_criterion_note',''))
        self.assertGreaterEqual(result['post_stop']['state0_count'],2)

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
