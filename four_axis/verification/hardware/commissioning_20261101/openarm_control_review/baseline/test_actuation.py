"""Synthetic controller and injected USB wire only. Never opens a real port."""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch
from .commissioning import Profile, Observation, Snapshot
from .actuation import (Acceptance, Envelope, FourAxisController, USB2CANEndpoint,
                        LimitedJ4Trial, LimitedJ4RoundTrip, PD_TIERS, DEFAULT_PD_TIER,
                        MAX_CANDIDATE_KP, pd_tier, torque_raw_bounds,
                        encoded_cap_torque_nm)


class Wire:
    def __init__(self):
        self.frames=[]
        self.fail=False
        self.closed=False

    def write(self,frame):
        self.frames.append(frame)
        if self.fail:
            raise IOError('synthetic unplug')
        return len(frame)

    def close(self):
        self.closed=True


class Tests(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.wire=Wire()
        self.events=[]
        self.audit=lambda event,**kw:self.events.append((event,kw))
        self.endpoint=USB2CANEndpoint(self.wire,self.audit,lambda:self.now)
        self.envelopes=tuple(Envelope(-.3,.3,.02,.5,.05,2.,.1,55.) for _ in range(4))
        self.acceptance=Acceptance(*(['SYNTHETIC_FIXTURE_NOT_HARDWARE_ACCEPTANCE']*6))
        self.controller=FourAxisController(self.endpoint,self.envelopes,self.acceptance,self.audit)

    def snapshot(self,states=None,positions=None):
        states=states or [0]*4
        positions=positions or [.1]*4
        rows=[]
        for i in range(1,5):
            p=Profile(i,1,0,2000,12.5,45.,54.,-1 if i in (1,4) else 1,.4)
            # raw payload is not used as an acknowledgement; positions are fixtures.
            rows.append(Observation(p,p.zero+p.sign*positions[i-1],0.,states[i-1],self.now,25.,25.,''))
        return Snapshot('fixture',self.now,tuple(rows),'mapping','fake','perf_counter')

    def advance(self,states=None,positions=None,delta=.010001):
        self.now+=delta
        self.controller.tick(self.snapshot(states,positions),self.now)

    def hold(self):
        self.controller.start(self.snapshot(),self.now)
        states=[0]*4
        for _ in range(600):
            self.advance(states)
            states=[1 if i+1 in self.controller.enabled or i+1==self.controller.pending else 0 for i in range(4)]
            if self.controller.state=='HOLD':
                return
        self.fail(self.controller.reason)

    def test_current_pose_not_zero_and_staggered_enable(self):
        self.controller.start(self.snapshot(),self.now)
        self.assertEqual(self.wire.frames,[])
        self.advance()
        self.assertEqual([p[13] for p in self.wire.frames],[1,1])
        self.assertEqual(self.wire.frames[-1][21:29],b'\xff'*7+b'\xfc')
        self.assertTrue(all(abs(v-.1)<1e-12 for v in self.controller.targets))
        self.advance([1,0,0,0])
        self.assertEqual(self.controller.enabled,{1})
        self.assertEqual(sum(f[21:29]==b'\xff'*7+b'\xfc' for f in self.wire.frames),1)

    def test_four_axis_hold_and_quantized_total_output_cap(self):
        self.hold()
        self.assertEqual(self.controller.enabled,{1,2,3,4})
        self.assertEqual([f[13] for f in self.wire.frames if f[21:29]==b'\xff'*7+b'\xfc'],[1,2,3,4])
        for frame in self.wire.frames:
            data=frame[21:29]
            self.assertEqual(len(frame),30)
            self.assertIn(frame[13],(1,2,3,4))
            if data[:7]==b'\xff'*7:
                continue
            kp=((data[3]&15)<<8)|data[4]; kd=(data[5]<<4)|(data[6]>>4)
            self.assertEqual((kp,kd),(0,0))
            tau=(((data[6]&15)<<8)|data[7])*108/4095-54
            self.assertLessEqual(abs(tau),.5)

    def test_plan_speed_bound_and_immutable_target(self):
        self.hold()
        with self.assertRaises(ValueError):
            self.controller.trajectory([.3]*4,1.,self.now)
        target=[.11]*4
        self.controller.trajectory(target,2.,self.now)
        target[0]=.3
        self.assertEqual(self.controller.plan[2],(.11,)*4)
        for _ in range(205):
            self.advance([1]*4,positions=list(self.controller.targets))
        self.assertEqual(self.controller.state,'HOLD')
        self.assertTrue(any(e[0]=='PLAN_REACHED_OBSERVED' for e in self.events))

    def test_one_fault_globally_disables_and_needs_fresh_confirmation(self):
        self.hold()
        before=len(self.wire.frames)
        self.advance([1,1,4,1])
        self.assertEqual(self.controller.state,'STOPPING')
        frames=self.wire.frames[before:]
        self.assertEqual([f[13] for f in frames],[1,2,3,4])
        self.assertTrue(all(f[21:29]==b'\xff'*7+b'\xfd' for f in frames))
        self.controller.observe_stop(self.snapshot(),self.now)
        self.assertEqual(self.controller.state,'STOPPING')
        self.advance()
        self.assertEqual(self.controller.state,'DISABLED')
        with self.assertRaises(ValueError):
            self.controller.start(self.snapshot(),self.now)

    def test_unplug_attempts_every_axis_and_reports_unknown(self):
        self.hold();self.wire.fail=True
        before=len(self.wire.frames)
        self.advance([1]*4)
        self.assertEqual(self.controller.state,'UNKNOWN')
        self.assertEqual([f[13] for f in self.wire.frames[before:] if f[28]==0xFD],[1,2,3,4])

    def test_stale_feedback_cancels_plan_before_any_control_write(self):
        self.hold();before=len(self.wire.frames)
        self.now+=.01
        s=self.snapshot([1]*4)
        rows=list(s.rows);rows[2]=replace(rows[2],received_at=self.now-.2)
        self.controller.tick(replace(s,rows=tuple(rows)),self.now)
        self.assertEqual(self.controller.state,'STOPPING')
        self.assertTrue(all(f[28]==0xFD for f in self.wire.frames[before:]))

    def test_profile_or_mapping_change_globally_stops(self):
        for field,value in [('mapping_sha256','changed'),('session_id','changed')]:
            self.setUp();self.hold();self.now+=.01
            self.controller.tick(replace(self.snapshot([1]*4),**{field:value}),self.now)
            self.assertEqual(self.controller.state,'STOPPING')

    def test_disabled_watchdog_pv_and_missing_evidence_block_before_tx(self):
        for mode,timeout in ((2,2000),(1,0)):
            self.setUp();s=self.snapshot();rows=list(s.rows)
            rows[0]=replace(rows[0],profile=replace(rows[0].profile,mode=mode,timeout=timeout))
            with self.assertRaises(ValueError):self.controller.start(replace(s,rows=tuple(rows)),self.now)
            self.assertEqual(self.wire.frames,[])
        self.controller.acceptance=replace(self.acceptance,watchdog_action='')
        with self.assertRaises(ValueError):self.controller.start(self.snapshot(),self.now)

    def test_enable_is_not_confirmed_by_serial_write(self):
        self.controller.start(self.snapshot(),self.now);self.advance()
        self.assertEqual(self.controller.enabled,set())
        self.assertEqual(self.endpoint.receipts[-1]['status'],'serial_written_not_drive_ack')
        for _ in range(32):self.advance()
        self.assertIn(self.controller.state,('STOPPING','DISABLED'))
        self.assertEqual(self.controller.reason,'Enable confirmation timeout')

    def test_host_overrun_and_following_error_stop(self):
        self.hold();self.advance([1]*4,delta=.06)
        self.assertEqual(self.controller.state,'STOPPING')
        self.setUp();self.hold();self.advance([1]*4,positions=[.2]*4)
        self.assertEqual(self.controller.state,'STOPPING')

    def test_no_j5_zero_flash_or_parameter_writes(self):
        for joint,payload,kind in ((5,b'\0'*8,'mit'),(1,b'\xff'*7+b'\xfe','mit'),
                                    (1,b'\xff'*7+b'\xfe','enable'),(0x7FF,b'\0'*8,'mit'),
                                    (1,b'\0'*8,'parameter')):
            with self.assertRaises(ValueError):self.endpoint.send(joint,payload,'test',kind)
        self.assertEqual(self.wire.frames,[])

    def test_partial_write_never_claims_sent(self):
        self.wire.write=lambda frame:4
        with self.assertRaises(IOError):self.endpoint.send(1,b'\0'*8,'test','mit')
        self.assertEqual(self.endpoint.receipts[-1]['status'],'delivery_unknown')

    def test_logging_failure_does_not_block_disable_attempts(self):
        self.hold()
        def broken(*args,**kwargs):raise IOError('synthetic disk full')
        self.endpoint.audit=broken;self.controller.audit=broken
        before=len(self.wire.frames)
        self.controller.stop('operator_stop',self.now)
        self.assertEqual([f[13] for f in self.wire.frames[before:]],[1,2,3,4])

    def test_torque_saturates_inside_quantized_bound(self):
        self.controller.envelopes=tuple(replace(e,kp=1000.) for e in self.envelopes)
        self.hold();self.advance([1]*4,positions=[.06]*4)
        for frame in self.wire.frames[-4:]:
            d=frame[21:29];tau=(((d[6]&15)<<8)|d[7])*108/4095-54
            self.assertGreater(abs(tau),.45)
            self.assertLessEqual(abs(tau),.5)

    def test_no_catchup_burst_within_period(self):
        self.hold();before=len(self.wire.frames)
        self.advance([1]*4,delta=.001)
        self.assertEqual(len(self.wire.frames),before)

    # ---- explicit host PD tiers (J4 limited trial) ---------------------

    def limited_envelopes(self,tier_name,kp=None,kd=None):
        tier=pd_tier(tier_name)
        envelopes=list(self.envelopes)
        q=.1
        envelopes[3]=Envelope(q-math.radians(2),q+math.radians(12),.02,tier.torque_cap,
                              math.radians(2),tier.kp if kp is None else kp,
                              tier.kd if kd is None else kd,55.,feedback_speed=tier.feedback_speed_abort,
                              arrival_error=math.radians(.5))
        return tuple(envelopes)

    def limited_controller(self,tier_name,kp=None,kd=None,round_trip=True):
        ticket=(LimitedJ4RoundTrip(True,True,True,'unused',hold_report='unused',
                                   pd_tier=tier_name)
                if round_trip else LimitedJ4Trial(True,True,True,'unused',pd_tier=tier_name))
        return FourAxisController(self.endpoint,
                                  self.limited_envelopes(tier_name,kp,kd),ticket,self.audit,
                                  active_axes=(4,),max_runtime=45. if round_trip else 2.,ramp=1.)

    def tick(self,controller,states,positions=None,delta=.010001):
        self.now+=delta
        controller.tick(self.snapshot(states,positions),self.now)

    def drive_limited_to_hold(self,controller):
        controller.start(self.snapshot(),self.now)
        states=[0,0,0,0]
        for _ in range(600):
            self.tick(controller,states)
            states=[1 if i+1 in controller.enabled or i+1==controller.pending else 0
                    for i in range(4)]
            if controller.state=='HOLD':break
        self.assertEqual(controller.state,'HOLD',controller.reason)
        self.assertEqual(controller.enabled,{4})
        for _ in range(120):  # finish the one-second output ramp
            self.tick(controller,[0,0,0,1])

    def measure_error(self,controller,degrees_now):
        q=controller.anchor[3]
        controller.targets=(q,q,q,q+math.radians(degrees_now)-1e-6)
        self.tick(controller,[0,0,0,1])
        return controller.output[4]

    @patch.object(LimitedJ4RoundTrip,"validate",lambda self:None)
    def test_one_nm_tier_slew_and_quantized_cap(self):
        tier=pd_tier('kp32_1nm_candidate')
        self.assertEqual(tier.torque_cap,1.)
        c=self.limited_controller(tier.name)
        self.drive_limited_to_hold(c)
        previous=c.output[4]
        for _ in range(230):
            current=self.measure_error(c,1.9)
            self.assertLessEqual(abs(current['limited_nm']-previous['limited_nm']),
                                 .5*(current['at']-previous['at'])+1e-12)
            self.assertLessEqual(abs(current['encoded_nm']),1.)
            previous=current
        self.assertGreater(abs(current['encoded_nm']),.95)
        self.assertEqual(c.active_axes,(4,))
        self.assertFalse(tier.hardware_validated)

    @patch.object(LimitedJ4RoundTrip,"validate",lambda self:None)
    def test_one_nm_accepts_point165_but_stops_above_point2(self):
        from dataclasses import replace
        c=self.limited_controller('kp32_1nm_candidate')
        self.drive_limited_to_hold(c)
        for speed in (-.164835,-.201):
            self.now+=.010001
            snapshot=self.snapshot([0,0,0,1])
            rows=list(snapshot.rows);rows[3]=replace(rows[3],velocity=speed)
            c.tick(replace(snapshot,rows=tuple(rows)),self.now)
            if abs(speed)<.2:
                self.assertEqual(c.state,'HOLD',c.reason)
            else:
                self.assertEqual(c.state,'STOPPING')
                self.assertIn('feedback speed',c.reason)
        self.assertEqual([f[13] for f in self.wire.frames if f[21:29]==b'\xff'*7+b'\xfd'],[1,2,3,4])

    def test_feedback_speed_threshold_is_scoped_to_one_nm_j4(self):
        from dataclasses import replace
        self.assertEqual(pd_tier('kp4_verified').feedback_speed_abort,.1)
        self.assertEqual(pd_tier('kp12_candidate').feedback_speed_abort,.1)
        self.assertEqual(pd_tier('kp32_1nm_candidate').feedback_speed_abort,.2)
        envelopes=list(self.limited_envelopes('kp32_1nm_candidate'))
        envelopes[0]=replace(envelopes[0],feedback_speed=.2)
        with self.assertRaises(ValueError):
            FourAxisController(self.endpoint,envelopes,
                LimitedJ4RoundTrip(True,True,True,'unused',hold_report='unused',
                                  pd_tier='kp32_1nm_candidate'),self.audit,
                active_axes=(4,),max_runtime=45.,ramp=1.)

    def test_one_nm_tier_cannot_expand_other_protections(self):
        from dataclasses import replace
        for changes in (dict(torque=1.01),dict(speed=.03),
                        dict(following_error=math.radians(3)),dict(kp=33.),
                        dict(feedback_speed=.201)):
            envelopes=list(self.limited_envelopes('kp32_1nm_candidate'))
            envelopes[3]=replace(envelopes[3],**changes)
            with self.assertRaises(ValueError):
                FourAxisController(self.endpoint,envelopes,
                    LimitedJ4RoundTrip(True,True,True,'unused',hold_report='unused',
                                      pd_tier='kp32_1nm_candidate'),self.audit,
                    active_axes=(4,),max_runtime=45.,ramp=1.)

    def test_limited_ticket_tier_must_match_envelope_and_ceiling(self):
        self.assertEqual(DEFAULT_PD_TIER,'kp4_verified')
        self.limited_controller('kp4_verified')
        self.limited_controller('kp12_candidate')
        for name,kp,kd in (('kp4_verified',12.,.5),('kp12_candidate',4.,.5),
                           ('kp12_candidate',12.,.4),('kp12_candidate',12.5,.5),
                           ('kp8_candidate',8.,.5)):
            with self.assertRaises(ValueError):
                self.limited_controller(name,kp,kd)
        self.assertEqual(MAX_CANDIDATE_KP,32.)

    def test_stiffer_tier_cannot_widen_cap_speed_or_error_window(self):
        base=PD_TIERS['kp12_candidate']
        for changes in (dict(torque=.31),dict(speed=.03)):
            envelopes=list(self.limited_envelopes('kp12_candidate'))
            envelopes[3]=replace(envelopes[3],**changes)
            ticket=LimitedJ4RoundTrip(True,True,True,'unused',hold_report='unused',
                                      pd_tier=base.name)
            with self.assertRaises(ValueError):
                FourAxisController(self.endpoint,tuple(envelopes),ticket,self.audit,
                                   active_axes=(4,),max_runtime=45.,ramp=1.)
        with self.assertRaises(ValueError):
            replace(self.limited_envelopes('kp12_candidate')[3],temperature=61.)
        wide=list(self.limited_envelopes('kp12_candidate'))
        wide[3]=replace(wide[3],upper=wide[3].lower+math.radians(15))
        with self.assertRaises(ValueError):
            FourAxisController(self.endpoint,tuple(wide),
                               LimitedJ4RoundTrip(True,True,True,'unused',hold_report='unused',
                                                  pd_tier=base.name),self.audit,
                               active_axes=(4,),max_runtime=45.,ramp=1.)

    def test_kp4_at_two_degrees_requests_less_than_half_the_cap(self):
        with patch.object(LimitedJ4RoundTrip,'validate',lambda self:None):
            c=self.limited_controller('kp4_verified')
            self.drive_limited_to_hold(c)
            out=self.measure_error(c,2.)
        self.assertAlmostEqual(out['requested_nm'],4.*(math.radians(2.)-1e-6),places=9)
        self.assertLess(out['requested_nm'],.5*.3)
        self.assertLess(abs(out['encoded_nm']),.3)
        self.assertFalse(out['at_encoded_cap'])
        self.assertFalse(out['request_clipped'])
        # The quantizer can move the encoded magnitude by at most half a quantum.
        self.assertLessEqual(abs(abs(out['encoded_nm'])-out['requested_nm']),.0264/2+1e-12)

    def test_stiff_tier_asks_more_early_and_saturates_without_exceeding_cap(self):
        with patch.object(LimitedJ4RoundTrip,'validate',lambda self:None):
            soft=self.limited_controller('kp4_verified')
            self.drive_limited_to_hold(soft)
            stiff=self.limited_controller('kp12_candidate')
            self.drive_limited_to_hold(stiff)
            one_soft=self.measure_error(soft,1.)
            one_stiff=self.measure_error(stiff,1.)
            two_stiff=self.measure_error(stiff,2.)
        self.assertGreater(abs(one_stiff['encoded_nm']),2.5*abs(one_soft['encoded_nm']))
        self.assertLessEqual(abs(one_stiff['encoded_nm']),.3)
        self.assertFalse(one_stiff['request_clipped'])
        self.assertGreater(two_stiff['requested_nm'],.3)
        self.assertTrue(two_stiff['request_clipped'])
        self.assertTrue(two_stiff['at_encoded_cap'])
        self.assertAlmostEqual(abs(two_stiff['encoded_nm']),
                               encoded_cap_torque_nm(.3,54.),places=12)
        self.assertLessEqual(abs(two_stiff['encoded_nm']),.3)

    def test_encoded_torque_never_exceeds_cap_across_full_raw_range(self):
        for cap,tmax in ((2.,54.),(1.,54.),(.3,54.),(.5,54.),(.5,12.5)):
            low,high=torque_raw_bounds(cap,tmax)
            self.assertAlmostEqual(abs(low*2*tmax/4095-tmax),
                                   abs(high*2*tmax/4095-tmax),places=12)
            for raw in range(low,high+1):
                decoded=raw*2*tmax/4095-tmax
                self.assertLessEqual(abs(decoded),cap+1e-12)
            self.assertLess(encoded_cap_torque_nm(cap,tmax),cap)
        self.assertAlmostEqual(encoded_cap_torque_nm(.3,54.),10.5*108/4095,places=12)

    def test_no_method_escalates_gains_after_a_stop(self):
        with patch.object(LimitedJ4RoundTrip,'validate',lambda self:None):
            c=self.limited_controller('kp4_verified')
            self.drive_limited_to_hold(c)
            before=(c.envelopes[3].kp,c.envelopes[3].kd,c.envelopes[3].torque)
            c.stop('operator_stop',self.now)
            self.assertEqual((c.envelopes[3].kp,c.envelopes[3].kd,c.envelopes[3].torque),before)
            self.assertEqual(c.tier.name,'kp4_verified')
            self.assertFalse(any(getattr(c,name,None) for name in
                                 ('set_gains','set_tier','boost','retune')))


if __name__=='__main__':
    unittest.main()
