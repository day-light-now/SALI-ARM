"""Production native runner with a synthetic motor-side PD loop; no hardware."""
import io
import json
import math
import threading
import unittest

from . import native_mit_console as app
from .test_watchdog_check import WatchdogBus, Clock
from .test_round_trip import Plant
from .test_reader_fixture import make_reader
from .openarm_mit import decode_mit_control_data, NATIVE_KP, NATIVE_KD
from .commissioning import Profile
from .watchdog_check import make_event_writer
from four_axis_monitor import load_mapping


class NativePlant(Plant):
    """The decoded MIT packet drives a 1ms simulated PD loop inside the motor.

    The host cannot set this plant's position or total torque directly. This is
    a synthetic low-friction plant, not evidence of physical breakaway/load.
    """
    def __init__(self,mapping,clock,static_friction=0.,kinetic_friction=0.,inertia=.05,joint=4):
        self.param=None
        self.joint=joint
        self.static_friction=static_friction;self.kinetic_friction=kinetic_friction;self.inertia=inertia
        super().__init__(mapping,clock)
        self.profile=Profile(joint,1,0,2000,12.5,45.,54.,mapping[joint]['hardware_sign'],
                             mapping[joint]['zero_offset_rad'])

    def advance(self):
        now=self.clock.now();remaining=now-self.at;self.at=now
        sign=self.mapping[self.joint]['hardware_sign'];zero=self.mapping[self.joint]['zero_offset_rad']
        while remaining>1e-12:
            dt=min(.001,remaining);remaining-=dt
            if self.states[self.joint]==1 and self.param is not None:
                d=self.param
                motor_tau=d.kp*(d.q-(zero+sign*self.q))+d.kd*(d.dq-sign*self.v)+d.tau
                self.tau=sign*max(-54.,min(54.,motor_tau))
                if abs(self.v)<1e-4 and abs(self.tau)<=self.static_friction:
                    self.v=0.
                    continue
                direction=math.copysign(1.,self.v if abs(self.v)>1e-8 else self.tau)
                new_v=self.v+(self.tau-.1*self.v-direction*self.kinetic_friction)/self.inertia*dt
                if new_v*self.v<0 and abs(self.tau)<=self.static_friction:new_v=0.
                self.v=new_v
                self.q+=self.v*dt
            else:self.v=0.;self.tau=0.

    def write(self,frame):
        self.advance()
        cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        if cid==self.joint and p[:7]!=b'\xff'*7:
            self.param=decode_mit_control_data(self.profile,p)
        return WatchdogBus.write(self,frame)

    def feedback(self,i):
        if i!=self.joint:return WatchdogBus.feedback(self,i)
        self.advance();p=self.profile
        raw=round((p.zero+p.sign*self.q+p.pmax)*65535/(2*p.pmax))
        vel=round((p.sign*self.v+p.vmax)*4095/(2*p.vmax))
        tau=round((p.sign*self.tau+p.tmax)*4095/(2*p.tmax))
        return bytes([self.states[i]*16+i,raw>>8,raw&255,vel>>4,
                      ((vel&15)<<4)|(tau>>8),tau&255,25,25])


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.mapping=load_mapping();self.cancel=threading.Event()
        self.bus=NativePlant(self.mapping,self.clock)
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        self.handle=io.StringIO();self.audit=make_event_writer(self.handle,lambda r:None)

    def tearDown(self):self.reader.close()

    def run_trial(self,mode='hold',ack=True,prior=None):
        return app.run_native_trial(self.reader,app.NativeTrialRequest(True,True,ack,'fake',mode),
            self.audit,self.cancel,connection_id='same-fake-connection',prior_hold=prior,
            clock=self.clock.now,sleep=self.clock.sleep)

    def test_no_native_ack_emits_no_enable(self):
        result=self.run_trial(ack=False)
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))

    def test_roundtrip_includes_fresh_takeover_without_separate_hold(self):
        result=self.run_trial('roundtrip10')
        self.assertTrue(result['round_trip_criterion_met'],result)
        self.assertTrue(result['takeover_stability_verified'])
        events=[json.loads(line) for line in self.handle.getvalue().splitlines()]
        ready=next(i for i,r in enumerate(events) if r['kind']=='NATIVE_TAKEOVER_STABLE')
        plan=next(i for i,r in enumerate(events) if r['kind']=='PLAN_ACCEPTED_NOT_SENT')
        self.assertLess(ready,plan)
        self.assertEqual(sum(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history),1)

    def test_unstable_takeover_stops_without_starting_trajectory(self):
        original=self.bus.feedback
        def feedback(i):
            payload=original(i)
            if i==4 and self.clock.now()>11.5:
                # >0.5deg instability but below the Kp160/2Nm estimate guard.
                data=bytearray(payload);raw=((data[1]<<8)|data[2])+28
                data[1]=raw>>8;data[2]=raw&255;return bytes(data)
            return payload
        self.bus.feedback=feedback
        result=self.run_trial('roundtrip10')
        self.assertEqual(result['reason'],'native_takeover_not_stable')
        self.assertFalse(result['round_trip_criterion_met'])
        self.assertFalse(result['takeover_stability_verified'])
        self.assertTrue(result['fresh_stop_confirmation'])
        self.assertFalse(any(json.loads(line)['kind']=='PLAN_ACCEPTED_NOT_SENT'
                             for line in self.handle.getvalue().splitlines()))

    def test_native_hold_then_roundtrip_uses_drive_pd_and_fresh_disable(self):
        hold=self.run_trial()
        self.assertTrue(hold['native_hold_criterion_met'],hold)
        result=self.run_trial('roundtrip10',prior=hold)
        self.assertTrue(result['round_trip_criterion_met'],result)
        self.assertFalse(result['host_total_torque_clamp'])
        self.assertFalse(result['driver_current_clamp_verified'])
        self.assertEqual(result['final_controller_state'],'DISABLED')
        self.assertGreaterEqual(result['actual_positive_excursion_deg'],9.5)
        self.assertLessEqual(abs(result['final_error_from_start_deg']),.5)
        native=[]
        for _,cid,payload in self.bus.history:
            if cid==0x7FF:
                self.assertIn(payload[0],(1,2,3,4))
                self.assertIn(payload[2],(0x33,0xCC))
            elif payload[:7]!=b'\xff'*7:
                self.assertEqual(cid,4)
                native.append(decode_mit_control_data(self.bus.profile,payload))
            elif payload[-1]==0xfc:self.assertEqual(cid,4)
        self.assertTrue(any(p.kp>NATIVE_KP-.13 and p.kd>NATIVE_KD-.002 for p in native))
        self.assertTrue(all(abs(p.tau)<.03 and abs(p.dq)<=.02 for p in native))
        events=[json.loads(line) for line in self.handle.getvalue().splitlines()]
        self.assertTrue(any(r['kind']=='NATIVE_TRIAL_RESULT' for r in events))

    def test_full_native_roundtrip_with_static_friction_release(self):
        self.reader.close()
        self.bus=NativePlant(self.mapping,self.clock,static_friction=1.2,
                             kinetic_friction=.2,inertia=.05)
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        result=self.run_trial('roundtrip10')
        self.assertTrue(result['round_trip_criterion_met'],result)
        self.assertEqual(result['drive_kp_requested'],160.)
        self.assertEqual(result['drive_kd_requested'],5.)
        self.assertEqual(result['feedback_speed_abort_rad_s'],1.)
        self.assertLessEqual(result['peak_observed_speed_rad_s'],.2)
        self.assertLessEqual(result['peak_observed_estimate_nm'],2.)

    def test_j3_roundtrip_enables_and_drives_only_j3(self):
        self.reader.close()
        self.bus=NativePlant(self.mapping,self.clock,joint=3)
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        request=app.NativeTrialRequest(True,True,True,'fake-j3','roundtrip10',joint=3)
        result=app.run_native_trial(self.reader,request,self.audit,self.cancel,
            connection_id='fake-j3',clock=self.clock.now,sleep=self.clock.sleep)
        self.assertTrue(result['round_trip_criterion_met'],result)
        self.assertEqual(result['joint'],3)
        self.assertGreater(result['peak_observed_estimate_nm'],0.)
        self.assertEqual(result['enabled_axes_permitted'],[3])
        self.assertTrue(result['fresh_stop_confirmation'])
        self.assertTrue(result['post_stop_visible_frames'])
        for _,cid,payload in self.bus.history:
            if cid==0x7FF:
                self.assertIn(payload[0],(1,2,3,4));self.assertIn(payload[2],(0x33,0xCC))
            elif payload[:7]!=b'\xff'*7 or payload[-1]==0xfc:self.assertEqual(cid,3)
        self.assertGreaterEqual(result['actual_positive_excursion_deg'],9.5)

    def test_j5_request_is_rejected_before_any_enable(self):
        request=app.NativeTrialRequest(True,True,True,'fake-reject-j5','roundtrip10',joint=5)
        result=app.run_native_trial(self.reader,request,self.audit,self.cancel,
            connection_id='fake-reject-j5',clock=self.clock.now,sleep=self.clock.sleep)
        self.assertIn('error',result)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in self.bus.history))
        self.assertFalse(any(cid==0x7FF and p[0]==5 for _,cid,p in self.bus.history))


if __name__=='__main__':unittest.main()
