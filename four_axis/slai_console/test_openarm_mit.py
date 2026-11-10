"""No hardware: official MIT wire vectors and native supervisor integration."""
import math
from dataclasses import replace
import unittest

from .commissioning import Profile, Observation, Snapshot
from .actuation import Envelope, FourAxisController, USB2CANEndpoint
from .native_actuation import NativeMITController
from .openarm_mit import (MITParam, NativeMITAcceptance, NATIVE_KP, NATIVE_KD, NATIVE_SPEED_ABORT, pack_mit_control_data,
                         decode_mit_control_data, model_command, estimate_drive_request, native_estimate_guard_nm)


class CodecTests(unittest.TestCase):
    def setUp(self):
        self.p = Profile(4, 1, 0, 2000, 12.5, 45., 54., -1, -.4)

    def test_official_zero_vector_truncates_not_rounds(self):
        data = pack_mit_control_data(self.p, MITParam(0., 0., 0., 0., 0.))
        self.assertEqual(data.hex(' '), '7f ff 7f f0 00 00 07 ff')
        self.assertLess(decode_mit_control_data(self.p, data).tau, 0.)

    def test_official_mixed_vector(self):
        # Independently specified integer fields: q=0x1234, dq=0x567,
        # kp=0x789, kd=0xabc, tau=0xdef; choose inside each quantizer bin.
        data = pack_mit_control_data(self.p, MITParam(
            (0x789+.25)*500/4095, (0xabc+.25)*5/4095,
            (0x1234+.25)*25/65535-12.5,
            (0x567+.25)*90/4095-45, (0xdef+.25)*108/4095-54))
        self.assertEqual(data.hex(' '), '12 34 56 77 89 ab cd ef')

    def test_native_gain_fields_and_coordinate_mapping(self):
        data = model_command(self.p, q=.1, dq=.01, kp=32., kd=2., tau_ff=.2)
        d = decode_mit_control_data(self.p, data)
        self.assertAlmostEqual(d.q, -.5, delta=25/65535)
        self.assertAlmostEqual(d.dq, -.01, delta=90/4095)
        self.assertGreater(d.kp, 31.8)
        self.assertGreater(d.kd, 1.99)
        self.assertAlmostEqual(d.tau, -.2, delta=108/4095)

    def test_invalid_fields_rejected_without_silent_clipping(self):
        for params in (MITParam(501, 0, 0, 0, 0), MITParam(0, 6, 0, 0, 0),
                       MITParam(0, 0, 13, 0, 0), MITParam(0, 0, 0, 46, 0),
                       MITParam(0, 0, 0, 0, 55), MITParam(float('nan'), 0, 0, 0, 0)):
            with self.assertRaises(ValueError):pack_mit_control_data(self.p, params)

    def test_encoded_reference_velocity_stays_inside_bench_limit(self):
        for v in (-.02,-.018,-.01,0.,.01,.018,.02):
            payload=model_command(self.p,q=0.,dq=v,kp=32.,kd=2.)
            self.assertLessEqual(abs(decode_mit_control_data(self.p,payload).dq),.02)

    def test_feedforward_is_not_total_effort_limit(self):
        data = model_command(self.p, q=.07, dq=0, kp=32, kd=2)
        estimated = estimate_drive_request(self.p, data, position=self.p.zero, velocity=0.)
        self.assertGreater(abs(estimated), 2.)
        self.assertLess(abs(decode_mit_control_data(self.p, data).tau), .03)


class Wire:
    def __init__(self):self.frames=[]
    def write(self, frame):self.frames.append(frame);return len(frame)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.wire=Wire();self.events=[]
        self.audit=lambda event,**fields:self.events.append((event,fields))
        self.endpoint=USB2CANEndpoint(self.wire,self.audit,lambda:self.now)
        self.profiles=tuple(Profile(i,1,0,2000,12.5,45.,54.,-1 if i==4 else 1,-.4)
                            for i in range(1,5))
        self.envelopes=tuple(Envelope(-math.radians(4),math.radians(14),.02,2.,
                            math.radians(4),NATIVE_KP if i==4 else 4.,NATIVE_KD if i==4 else .5,
                            50.,feedback_speed=NATIVE_SPEED_ABORT if i==4 else .1,
                            arrival_error=math.radians(.5)) for i in range(1,5))
        self.ticket=NativeMITAcceptance(True,True,True,'fake-native-request',
                                        'SYNTHETIC_ACK_NOT_HARDWARE_VERIFICATION')

    def snapshot(self, states=(0,0,0,0), velocities=None):
        velocities=velocities or (0.,)*4
        rows=tuple(Observation(p,p.zero,v,state,self.now,25.,25.,'')
                   for p,v,state in zip(self.profiles,velocities,states))
        return Snapshot('fake-native',self.now,rows,'mapping','fake','perf_counter')

    def controller(self,ticket=None):
        return NativeMITController(self.endpoint,self.envelopes,ticket or self.ticket,
                    self.audit,active_axes=(4,),max_runtime=45.,ramp=1.)

    def hold(self,c):
        c.start(self.snapshot(),self.now)
        for _ in range(135):
            self.now+=.010001
            c.tick(self.snapshot((0,0,0,1 if c.pending==4 or 4 in c.enabled else 0)),self.now)
        self.assertEqual(c.state,'HOLD',c.reason)

    def test_j2_ten_nm_scope_and_actual_frame_guard(self):
        ticket=replace(self.ticket,joint=2)
        envelopes=tuple(replace(e,torque=10.,kp=NATIVE_KP,kd=NATIVE_KD,feedback_speed=1.)
                        if i==2 else replace(e,kp=4.,kd=.5,feedback_speed=.1)
                        for i,e in enumerate(self.envelopes,1))
        ticket.validate_scope(envelopes,(2,),45.,1.)
        with self.assertRaises(ValueError):
            ticket.validate_scope(tuple(replace(e,torque=11.) if i==2 else e
                for i,e in enumerate(envelopes,1)),(2,),45.,1.)
        c=NativeMITController(self.endpoint,envelopes,ticket,self.audit,
                            active_axes=(2,),max_runtime=45.,ramp=1.)
        c.start(self.snapshot(),self.now)
        observation=self.snapshot().rows[1]
        c.ramp_at[2]=self.now-2.
        c._torque_frame(observation,envelopes[1],.055,0.,self.now)
        self.assertGreater(c.output[2]['observed_state_estimate_nm'],8.)
        self.assertLess(c.output[2]['observed_state_estimate_nm'],10.)
        with self.assertRaisesRegex(ValueError,'10Nm'):
            c._torque_frame(observation,envelopes[1],.07,0.,self.now)
        self.assertEqual([native_estimate_guard_nm(i) for i in (1,2,3,4)],[8.,10.,2.,2.])
        with self.assertRaises(ValueError):replace(ticket,joint=1).validate()

    def test_no_ack_rejected_before_any_write(self):
        ticket=NativeMITAcceptance(True,True,False,'fake','fake')
        c=self.controller(ticket)
        with self.assertRaises(ValueError):c.start(self.snapshot(),self.now)
        self.assertEqual(self.wire.frames,[])

    def test_host_controller_cannot_consume_native_ticket(self):
        with self.assertRaises(ValueError):
            FourAxisController(self.endpoint,self.envelopes,self.ticket,self.audit,
                               active_axes=(4,),max_runtime=45.,ramp=1.)

    def test_actual_native_fields_not_host_pd(self):
        c=self.controller();self.hold(c)
        frames=[f[21:29] for f in self.wire.frames if f[21:28]!=b'\xff'*7]
        preload=decode_mit_control_data(self.profiles[3],frames[0])
        last=decode_mit_control_data(self.profiles[3],frames[-1])
        self.assertEqual(preload.kp,0.)
        self.assertGreater(last.kp,NATIVE_KP-.13);self.assertGreater(last.kd,NATIVE_KD-.002)
        self.assertEqual(c.anchor,(0.,)*4)
        self.assertEqual({f[13] for f in self.wire.frames},{4})
        self.assertTrue(any(e=='NATIVE_MIT_TAKEOVER' for e,_ in self.events))

    def test_native_velocity_fault_stops_all_without_mit_on_fault_frame(self):
        c=self.controller();self.hold(c);before=len(self.wire.frames)
        self.now+=.010001
        c.tick(self.snapshot((0,0,0,1),(0.,0.,0.,-1.001)),self.now)
        self.assertEqual(c.state,'STOPPING')
        self.assertIn('feedback speed',c.reason)
        self.assertEqual([f[13] for f in self.wire.frames[before:]],[1,2,3,4])
        self.assertTrue(all(f[21:29]==b'\xff'*7+b'\xfd' for f in self.wire.frames[before:]))

    def test_native_point25_no_longer_trips_old_point2_threshold(self):
        c=self.controller();self.hold(c);self.now+=.010001
        c.tick(self.snapshot((0,0,0,1),(0.,0.,0.,-.25)),self.now)
        self.assertEqual(c.state,'HOLD',c.reason)
        self.assertLess(abs(c.output[4]['observed_state_estimate_nm']),2.)

    def test_native_observed_estimate_guard_is_not_disabled(self):
        c=self.controller();self.hold(c)
        c.targets=(0.,0.,0.,math.radians(3.9))
        self.now+=.010001;c.tick(self.snapshot((0,0,0,1)),self.now)
        self.assertEqual(c.state,'STOPPING')
        self.assertIn('estimate',c.reason)

    def test_only_frozen_j4_roundtrip_is_allowed(self):
        c=self.controller();self.hold(c)
        with self.assertRaises(ValueError):c.trajectory((.1,0,0,0),18.,self.now)
        with self.assertRaises(ValueError):c.trajectory((0,0,0,math.radians(10)),2.,self.now)
        c.trajectory((0,0,0,math.radians(10)),18.,self.now)
        self.assertEqual(c.trial_leg,1)


if __name__=='__main__':unittest.main()
