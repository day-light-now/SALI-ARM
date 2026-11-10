"""Bounded integral + real native session path using a synthetic disturbance."""
from dataclasses import replace
import io
import math
import threading
import unittest

from .holding_bias import HoldingBias, BiasSettings, bias_packet
from .commissioning import Profile, Observation, Snapshot
from .openarm_mit import model_command, decode_mit_control_data, NativeMITAcceptance
from .actuation import Envelope, USB2CANEndpoint
from .native_actuation import NativeMITController
from .test_native_console import NativePlant
from .test_watchdog_check import Clock
from .test_reader_fixture import make_reader
from .native_mit_console import NativeTrialRequest, run_native_trial
from .watchdog_check import make_event_writer
from unittest.mock import patch
from four_axis_monitor import load_mapping


class LoadedPlant(NativePlant):
    """Declared test disturbance, never loaded from CAD/simulation mass data."""
    def advance(self):
        now=self.clock.now();remaining=now-self.at;self.at=now
        p=self.profile
        while remaining>1e-12:
            dt=min(.001,remaining);remaining-=dt
            if self.states[self.joint]==1 and self.param is not None:
                d=self.param
                self.tau=max(-p.tmax,min(p.tmax,
                    d.kp*(d.q-(p.zero+p.sign*self.q))+d.kd*(d.dq-p.sign*self.v)+d.tau))
                # Disturbance smoothly appears after initial supported takeover.
                load=getattr(self,'disturbance_nm',1.6)*max(0.,min(1.,
                    (now-remaining-getattr(self,'disturbance_start_time',13.))/getattr(self,'disturbance_ramp_s',4.)))
                self.v+=(p.sign*self.tau-load-.1*self.v)/self.inertia*dt
                self.q+=self.v*dt
            else:self.v=0.;self.tau=0.


class Tests(unittest.TestCase):
    def test_cap_no_windup_and_opposite_error_unwinds_immediately(self):
        b=HoldingBias()
        for i in range(2001):r=b.step(math.radians(1.),0.,i*.01,allow=True)
        self.assertEqual(b.value,2.)
        self.assertEqual(r['reason'],'bias_cap')
        b.step(-math.radians(1.),0.,20.01,allow=True)
        self.assertLess(b.value,2.)

    def test_freeze_preload_speed_error_and_deadband(self):
        b=HoldingBias();b.step(.01,0.,0.,allow=True)
        b.step(.01,0.,.01,allow=True);before=b.value
        for n,(err,v,allow,reason) in enumerate(((.01,0.,False,'preload_or_gain_ramp'),
            (.01,.051,True,'feedback_speed'),(.1,0.,True,'error_window'),
            (0.,0.,True,'deadband')),2):
            r=b.step(err,v,n*.01,allow=allow)
            self.assertEqual(r['reason'],reason);self.assertEqual(b.value,before)

    def test_slew_and_fault_latch_and_reset(self):
        b=HoldingBias(replace(BiasSettings(),ki_nm_per_rad_s=10000.))
        b.step(.01,0.,0.,allow=True)
        b.step(.01,0.,.01,allow=True);self.assertLessEqual(b.value,.005)
        with self.assertRaises(ValueError):b.step(.01,0.,.06,allow=True)
        self.assertEqual(b.value,0.)
        with self.assertRaises(ValueError):b.step(.01,0.,.07,allow=True)
        b.reset();self.assertEqual(b.step(.01,0.,1.,allow=True)['model_bias_nm'],0.)
        with self.assertRaises(ValueError):b.step(float('nan'),0.,1.01,allow=True)

    def test_packet_bound_sign_and_zero_preserves_baseline(self):
        for sign in (-1,1):
            p=Profile(2,1,0,2000,12.5,25.,200.,sign,-1.8)
            original=model_command(p,q=0.,dq=0.,kp=160.,kd=5.)
            self.assertEqual(bias_packet(p,original,0.,2.),original)
            for value in (-2.,-.5,.5,2.):
                packet=bias_packet(p,original,value,2.)
                d=decode_mit_control_data(p,packet)
                self.assertLessEqual(abs(d.tau),2.)
                self.assertAlmostEqual(sign*d.tau,value,delta=400/4095)
                self.assertEqual(packet[:6],original[:6])
                self.assertEqual(packet[6]&0xf0,original[6]&0xf0)
            with self.assertRaises(ValueError):bias_packet(p,original,2.01,2.)

    def trial(self,enabled,joint=2,cancelled=False):
        clock=Clock();mapping=load_mapping();bus=LoadedPlant(mapping,clock,joint=2 if joint==5 else joint)
        reader=make_reader('FAKE_ONLY',mapping,serial_factory=lambda *a,**k:bus)
        handle=io.StringIO();cancel=threading.Event()
        if cancelled:cancel.set()
        try:
            result=run_native_trial(reader,NativeTrialRequest(True,True,True,'bias-fake',
                'roundtrip10',joint=joint,holding_bias_acknowledged=enabled),
                make_event_writer(handle,lambda row:None),cancel,
                connection_id='fake',clock=clock.now,sleep=clock.sleep)
            return result,bus.history,handle.getvalue()
        finally:reader.close()

    def test_native_candidate_improves_known_loaded_roundtrip(self):
        baseline,_,_=self.trial(False)
        candidate,history,audit=self.trial(True)
        self.assertEqual(baseline['reason'],'Trajectory arrival timeout',baseline)
        self.assertFalse(baseline['round_trip_criterion_met'])
        self.assertTrue(candidate['round_trip_criterion_met'],candidate)
        self.assertTrue(candidate['fresh_stop_confirmation'])
        self.assertGreater(candidate['peak_requested_bias_nm'],0.)
        self.assertLessEqual(candidate['peak_requested_bias_nm'],2.)
        self.assertLessEqual(candidate['peak_encoded_ff_nm'],2.)
        self.assertLessEqual(candidate['peak_observed_estimate_nm'],10.)
        self.assertFalse(candidate['simulated_mass_used'])
        self.assertIn('holding_bias',audit)
        for _,cid,payload in history:
            if cid==0x7ff:self.assertIn(payload[0],(1,2,3,4))
            elif payload[:7]!=b'\xff'*7 or payload[-1]==0xfc:self.assertEqual(cid,2)

    def test_wrong_axis_rejected_without_enable_and_cancel_is_clean(self):
        for joint in (3,4,5):
            r,history,_=self.trial(True,joint=joint)
            self.assertIn('error',r)
            self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in history))
        r,history,_=self.trial(True,cancelled=True)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in history))

    def test_runtime_requires_no_gravity_model_or_simulation_imports(self):
        with patch.dict('sys.modules',{'slai_console.gravity_feedforward':None,
            'verification.gravity_model':None,'mujoco':None}):
            result,_,_=self.trial(True)
        self.assertTrue(result['round_trip_criterion_met'],result)

    def test_total_estimate_fault_sends_fd_only_and_clears_bias(self):
        class Wire:
            def __init__(self):self.frames=[]
            def write(self,frame):self.frames.append(frame);return len(frame)
        wire=Wire();now=[10.];audit=lambda *a,**k:None
        endpoint=USB2CANEndpoint(wire,audit,lambda:now[0])
        profiles=tuple(Profile(i,1,0,2000,12.5,25.,200.,1,0.) for i in range(1,5))
        envelopes=tuple(Envelope(-math.radians(4),math.radians(14),.02,
            10. if i==2 else .5,math.radians(4),160. if i==2 else 4.,5. if i==2 else .5,
            50.,feedback_speed=1. if i==2 else .1,arrival_error=math.radians(.5))
            for i in range(1,5))
        ticket=NativeMITAcceptance(True,True,True,'bias-fault','fake',joint=2,
                                   holding_bias_acknowledged=True)
        c=NativeMITController(endpoint,envelopes,ticket,audit,active_axes=(2,),max_runtime=45.,ramp=1.)
        def snapshot(enabled=False,velocity=0.):
            return Snapshot('fake',now[0],tuple(Observation(p,0.,velocity if p.joint==2 else 0.,
                1 if enabled and p.joint==2 else 0,now[0],25.,25.,'') for p in profiles),'mapping','fake','perf_counter')
        c.start(snapshot(),now[0]);c.enabled={2};c.state='HOLD';c.ramp_at[2]=9.
        c.holding_bias.value=2.;c.holding_bias.last=10.
        c.targets=(0.,math.radians(1.4),0.,0.)
        now[0]=10.011;c.tick(snapshot(True,-.9),now[0])
        self.assertEqual(c.state,'STOPPING')
        self.assertIn('10Nm',c.reason)
        self.assertEqual(c.holding_bias.value,0.)
        self.assertEqual([f[13] for f in wire.frames],[1,2,3,4])
        self.assertTrue(all(f[21:29]==b'\xff'*7+b'\xfd' for f in wire.frames))


if __name__=='__main__':unittest.main()
