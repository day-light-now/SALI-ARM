"""Commissioning/shadow invariants with synthetic feedback; no hardware."""
from dataclasses import replace
import json
import math
import struct
import tempfile
import time
import unittest
from unittest.mock import patch
from .commissioning import (JointCommand,Profile,Observation,Snapshot,ShadowEngine,
                            capture,encode_shadow,readiness)
from .clock import now as clock_now, CLOCK_NAME


def profile(i,mode=None):
    return Profile(i,mode or (2 if i==1 else 1),0,0,12.5,45.,54.,-1 if i in (1,4) else 1,.4)


def snapshot(now=10.,state=0):
    rows=tuple(Observation(profile(i),.45,0.,state,now,25.,26.,bytes([i,0x81,0,0x7f,0xf7,0xff,25,26]).hex(' ')) for i in range(1,5))
    return Snapshot('test-session',now,rows,'test-mapping-hash','synthetic-fixture')


class CommissioningTests(unittest.TestCase):
    def test_j5_and_unsupported_modes_rejected(self):
        for i,mode in ((5,1),(0,1),(3,3),(3,4)):
            with self.assertRaises(ValueError):
                profile(i,mode)

    def test_position_velocity_uses_speed_bound_and_mapping(self):
        p=profile(1)
        out=encode_shadow(p,JointCommand(.1,speed_limit=.02))
        q,v=struct.unpack('<ff',bytes.fromhex(out['candidate_payload']))
        self.assertEqual(out['candidate_can_id'],0x101)
        self.assertAlmostEqual(q,.3,places=6)
        self.assertAlmostEqual(v,.02,places=6)
        self.assertFalse(out['effort_limit_supported'])

    def test_pv_does_not_silently_ignore_mit_fields(self):
        for cmd in (JointCommand(0,kp=1),JointCommand(0,kd=1),JointCommand(0,tau_ff=.1),JointCommand(0,vel=.01)):
            with self.assertRaises(ValueError):
                encode_shadow(profile(1),cmd)

    def test_mit_packet_layout_and_residual_zero_torque(self):
        out=encode_shadow(profile(3),JointCommand(0))
        raw=bytes.fromhex(out['candidate_payload'])
        self.assertEqual(len(raw),8)
        kp=((raw[3]&15)<<8)|raw[4]
        kd=(raw[5]<<4)|(raw[6]>>4)
        tau=((raw[6]&15)<<8)|raw[7]
        self.assertEqual((kp,kd),(0,0))
        self.assertNotEqual(out['decoded_tau_ff_nm'],0.)
        self.assertFalse(out['zero_tau_is_exact'])
        self.assertAlmostEqual(tau*108/4095-54,out['decoded_tau_ff_nm'])

    def test_torque_and_velocity_sign_map(self):
        p=profile(4)
        out=encode_shadow(p,JointCommand(.1,vel=.01,tau_ff=.5))
        raw=bytes.fromhex(out['candidate_payload'])
        vel=(raw[2]<<4)|(raw[3]>>4)
        self.assertLess(vel*90/4095-45,0)
        self.assertLess(out['decoded_tau_ff_nm'],0)

    def test_no_silent_protocol_clipping(self):
        for c in (JointCommand(math.nan),JointCommand(20),JointCommand(0,kp=501),
                  JointCommand(0,kd=6),JointCommand(0,tau_ff=100),JointCommand(0,vel=.1)):
            with self.assertRaises(ValueError):
                encode_shadow(profile(3),c)

    def test_takeover_candidate_captures_current_not_zero(self):
        s=snapshot()
        e=ShadowEngine(s)
        self.assertTrue(all(abs(q)>.01 for q in e.targets))
        r=e.step(s,10.)
        self.assertEqual(len(r['frames']),4)
        self.assertEqual(r['actuation_tx'],0)
        for out,target in zip(r['frames'],e.targets):
            self.assertEqual(out['command']['pos'],target)
            self.assertEqual(out['command']['kp'],0.)

    def test_global_fault_discards_all_candidates_and_latches(self):
        s=snapshot()
        e=ShadowEngine(s)
        rows=list(s.rows)
        rows[2]=replace(rows[2],state=4)
        r=e.step(replace(s,rows=tuple(rows)),10.)
        self.assertEqual(r['frames'],[])
        self.assertIn('J3',r['global_latch'])
        self.assertEqual(e.step(s,10.01)['frames'],[])

    def test_stale_feedback_and_negative_age_rejected(self):
        s=snapshot()
        for now in (10.4,9.9):
            r=ShadowEngine(s).step(s,now)
            self.assertEqual(r['frames'],[])
            self.assertIn('STALE',r['global_latch'])

    def test_host_overrun_has_no_catchup_or_partial_frames(self):
        s=snapshot()
        e=ShadowEngine(s)
        e.step(s,10.)
        self.assertEqual(e.step(s,10.1)['global_latch'],'HOST_CLOCK_OR_LOOP_DELAY')

    def test_connection_profile_mapping_changes_invalidate_all(self):
        s=snapshot()
        variants=[replace(s,session_id='new'),replace(s,mapping_sha256='changed'),replace(s,clock='different-domain'),
            replace(s,rows=(replace(s.rows[0],profile=replace(s.rows[0].profile,timeout=2000)),*s.rows[1:]))]
        for changed in variants:
            self.assertEqual(ShadowEngine(s).step(changed,10.)['frames'],[])

    def test_current_feedback_reference_departure_is_diagnostic(self):
        s=snapshot()
        rows=list(s.rows)
        rows[1]=replace(rows[1],position=rows[1].position+.05)
        r=ShadowEngine(s).step(replace(s,rows=tuple(rows)),10.)
        self.assertEqual(r['frames'],[])
        self.assertIn('REFERENCE_DEPARTURE',r['global_latch'])

    def test_100hz_compute_does_not_imply_100hz_feedback(self):
        s=snapshot()
        e=ShadowEngine(s)
        for step in range(20):
            e.step(s,10.+step*.01)
        summary=e.summary()
        self.assertEqual(summary['cycles'],20)
        self.assertGreater(summary['cycles_exceeding_80ms_feedback_age'],0)
        self.assertEqual(summary['bus_four_axis_control_cycle'],'NOT_MEASURED')
        self.assertFalse(summary['motion_permitted'])

    def test_zero_watchdog_never_grants_motion(self):
        result=readiness(snapshot())
        self.assertFalse(result['motion_permitted'])
        self.assertTrue(all(f'J{i}_WATCHDOG_DISABLED' in result['blocks'] for i in range(1,5)))

    def test_partial_snapshot_and_payload_mismatch_rejected(self):
        mapping={i:dict(hardware_sign=1,zero_offset_rad=0.) for i in range(1,5)}
        with self.assertRaises(ValueError):
            capture(dict(session_id='test',rows={}),mapping,10.)
        rows={i:dict(regs={7:0,8:i,9:0,10:1,21:12.5,22:45.,23:54.},age=0,
            position_rad=0.,velocity_rad_s=0.,driver_temp_c=25,motor_temp_c=25,state_code=0,
            payload=bytes([i,0x7f,0xff,0x7f,0xf7,0xff,25,25]).hex(' ')) for i in range(1,5)}
        s=capture(dict(session_id='test',at=10.,rows=rows),mapping,10.)
        self.assertEqual(len(s.rows),4)
        rows[2]['payload']='05 7f ff 7f f7 ff 19 19'
        with self.assertRaises(ValueError):
            capture(dict(session_id='test',at=10.,rows=rows),mapping,10.)

    def test_shadow_worker_stops_without_opening_transport(self):
        from .shadow_worker import ShadowWorker
        mapping={i:dict(hardware_sign=1,zero_offset_rad=0.) for i in range(1,5)}
        with tempfile.TemporaryDirectory() as directory, patch('serial.Serial',side_effect=AssertionError('No COM')):
            worker=ShadowWorker(snapshot(clock_now()),mapping,{},duration=.05,directory=directory)
            worker.start()
            self.assertTrue(worker.wait(3000))
            rows=[json.loads(x) for x in worker.path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(rows[-1]['kind'],'summary')
            self.assertEqual(rows[-1]['actuation_tx'],0)
            self.assertIsNotNone(rows[-1]['global_latch'])

    def test_shadow_worker_valid_session_records_candidates_without_transport(self):
        from .shadow_worker import ShadowWorker
        mapping={i:dict(hardware_sign=1,zero_offset_rad=0.) for i in range(1,5)}
        at=clock_now()
        rows={i:dict(regs={7:0,8:i,9:0,10:2 if i==1 else 1,21:12.5,22:45.,23:54.},age=0,
            received_at=at,position_rad=0.,velocity_rad_s=0.,driver_temp_c=25,motor_temp_c=25,state_code=0,
            payload=bytes([i,0x7f,0xff,0x7f,0xf7,0xff,25,25]).hex(' ')) for i in range(1,5)}
        sample=dict(session_id='synthetic',at=at,rows=rows)
        initial=capture(sample,mapping,at)
        with tempfile.TemporaryDirectory() as directory, patch('serial.Serial',side_effect=AssertionError('No COM')):
            worker=ShadowWorker(initial,mapping,sample,duration=.04,directory=directory)
            worker.start()
            self.assertTrue(worker.wait(3000))
            data=[json.loads(x) for x in worker.path.read_text(encoding='utf-8').splitlines()]
            cycles=[x for x in data if x['kind']=='shadow_cycle']
            self.assertGreater(len(cycles),0)
            self.assertTrue(all(len(x['frames'])==4 and x['actuation_tx']==0 for x in cycles))
            self.assertIsNone(data[-1]['global_latch'])


if __name__=='__main__':
    unittest.main()
