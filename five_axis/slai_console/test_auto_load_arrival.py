"""Replay the captured endpoint. Replayed positions cannot prove new motion."""
from dataclasses import fields
from dataclasses import replace
import json
import math
import unittest
from types import SimpleNamespace

from .core import ROOT
from .auto_load_acquisition import AutoLoadPlan,ScanPoint
from .auto_load_runner import session_for_plan
from .commissioning import Profile,Observation,Snapshot
from .holding_bias import HoldingBias


class Tests(unittest.TestCase):
    def recorded_controller(self):
        f=json.loads((ROOT/'verification/hardware/load_calibration/auto_acquisition_20261102/arrival_timeout_163155/endpoint_fixture.json').read_text(encoding='utf-8'))
        kwargs={key:f['plan'][key] for key in (d.name for d in fields(AutoLoadPlan)) if key in f['plan']}
        kwargs['points']=tuple(ScanPoint(**{**p,'target_pose_rad':tuple(p['target_pose_rad'])}) for p in kwargs['points'])
        kwargs['initial_pose_rad']=tuple(kwargs['initial_pose_rad'])
        plan=AutoLoadPlan(**kwargs);profiles=tuple(Profile(**p) for p in f['profiles'])
        class Wire:
            def __init__(self):self.frames=[]
            def write(self,b):self.frames.append(b);return len(b)
        wire=Wire();clock=[0.]
        session=session_for_plan(SimpleNamespace(serial=wire),plan,lambda *a,**k:None,
            acknowledged=True,clock=lambda:clock[0])
        c=session.controller
        def snapshot(pose,enabled,velocity=0.):
            return Snapshot('recorded',clock[0],tuple(Observation(p,p.zero+p.sign*q,
                velocity if p.joint==1 else 0.,1 if p.joint==1 and enabled else 0,
                clock[0],31.,27.,'') for p,q in zip(profiles,pose)),plan.mapping_sha256,'recorded','perf_counter')
        c.start(snapshot(plan.initial_pose_rad,False),0.)
        point=plan.points[f['point_index']];duration=point.duration_s
        c.enabled={1};c.pending=None;c.ramp_at[1]=-2.;c.state='TRACK'
        c.plan=('recorded-endpoint',plan.points[1].target_pose_rad,point.target_pose_rad,duration,0.)
        c.targets=point.target_pose_rad;c.next_point=3;c.last_time=duration-.02
        c.holding_bias.value=f['requested_bias_nm'];c.holding_bias.last=c.last_time
        return c,clock,lambda:snapshot(f['actual_pose_rad'],True,f['velocity_motor_rad_s']),duration

    def test_recorded_point_does_not_abort_after_only_point3_seconds(self):
        c,clock,snapshot,duration=self.recorded_controller()
        for n in range(-1,36):
            clock[0]=duration+n*.01;c.tick(snapshot(),clock[0])
        self.assertEqual(c.state,'SETTLING',c.reason)
        self.assertLess(abs(c.output[1]['observed_state_estimate_nm']),8.)

    def test_recorded_error_can_continue_unwinding_below_total_guard(self):
        legacy,oldclock,oldsnapshot,oldduration=self.recorded_controller()
        legacy.holding_bias=HoldingBias(replace(legacy.holding_bias.settings,cap_nm=2.))
        legacy.holding_bias.value=-2.;legacy.holding_bias.last=oldduration-.02
        oldclock[0]=oldduration-.009;legacy.tick(oldsnapshot(),oldclock[0])
        self.assertEqual(legacy.holding_bias.value,-2.)
        c,clock,snapshot,duration=self.recorded_controller()
        clock[0]=duration-.009;c.tick(snapshot(),clock[0])
        self.assertLess(c.holding_bias.value,-2.)
        self.assertLess(abs(c.output[1]['observed_state_estimate_nm']),8.)

    def test_nonconverging_replayed_position_stops_at_bounded_settle_deadline(self):
        c,clock,snapshot,duration=self.recorded_controller()
        for n in range(-1,803):
            clock[0]=duration+n*.01;c.tick(snapshot(),clock[0])
            if c.state=='STOPPING':break
        self.assertEqual(c.state,'STOPPING')
        self.assertEqual(c.reason,'arrival_settle_timeout')
        self.assertEqual(c.holding_bias.value,0.)

    def test_converged_actual_position_proceeds_without_waiting_full_deadline(self):
        c,clock,snapshot,duration=self.recorded_controller()
        for n in range(-1,70):
            clock[0]=duration+n*.01;c.tick(snapshot(),clock[0])
        self.assertEqual(c.state,'SETTLING')
        measured=snapshot();rows=list(measured.rows);o=rows[0]
        rows[0]=replace(o,position=o.profile.zero+o.profile.sign*c.targets[0])
        clock[0]=duration+.711
        rows=[replace(row,received_at=clock[0]) for row in rows]
        c.tick(replace(measured,observed_at=clock[0],rows=tuple(rows)),clock[0])
        self.assertEqual(c.state,'HOLD',c.reason)

    def test_speed_guard_still_stops_during_settling(self):
        c,clock,snapshot,duration=self.recorded_controller()
        for n in range(-1,35):
            clock[0]=duration+n*.01;c.tick(snapshot(),clock[0])
        measured=snapshot();rows=list(measured.rows)
        clock[0]=duration+.351
        rows=[replace(row,received_at=clock[0]) for row in rows]
        rows[0]=replace(rows[0],velocity=1.01)
        c.tick(replace(measured,observed_at=clock[0],rows=tuple(rows)),clock[0])
        self.assertEqual(c.state,'STOPPING')
        self.assertIn('feedback speed',c.reason)


if __name__=='__main__':unittest.main()
