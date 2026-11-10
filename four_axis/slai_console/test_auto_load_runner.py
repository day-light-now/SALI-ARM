import io
import threading
import unittest

from .auto_load_acquisition import build_plan
from .auto_load_runner import run_acquisition
from .native_mit_console import fill_preflight
from .test_holding_bias import LoadedPlant
from .test_watchdog_check import Clock
from .test_reader_fixture import make_reader
from .watchdog_check import make_event_writer
from four_axis_monitor import load_mapping
from .core import ROOT
import hashlib
import json
import tempfile
from pathlib import Path
from .load_calibration import save_campaign_points,inventory


class Tests(unittest.TestCase):
    def trial(self,ack=True,hook=None,ignore_fd=False,disturbance_nm=1.6,scan_limits_deg=(-5.,5.),
              disturbance_start_time=13.,disturbance_ramp_s=4.):
        class Bus(LoadedPlant):
            def write(self,frame):
                cid=int.from_bytes(frame[13:17],'little');payload=frame[21:29]
                if ignore_fd and payload==b'\xff'*7+b'\xfd':
                    self.history.append((self.clock.now(),cid,payload));return len(frame)
                return super().write(frame)
        clock=Clock();mapping=load_mapping();bus=Bus(mapping,clock,joint=1)
        bus.disturbance_nm=disturbance_nm
        bus.disturbance_start_time=disturbance_start_time;bus.disturbance_ramp_s=disturbance_ramp_s
        reader=make_reader('FAKE_ONLY',mapping,serial_factory=lambda *a,**k:bus)
        cancel=threading.Event();stream=io.StringIO()
        try:
            fill_preflight(reader,cancel,clock.now,clock.sleep)
            q=[mapping[i]['hardware_sign']*(reader.state.feedback[i]['position_rad']-mapping[i]['zero_offset_rad']) for i in range(1,5)]
            digest=hashlib.sha256((ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml').read_bytes()).hexdigest()
            plan=build_plan(joint=1,initial_pose_rad=q,mechanical_limits_deg=(-30.,30.),
                scan_limits_deg=scan_limits_deg,step_deg=10.,mapping_sha256=digest,
                physical_limits_confirmed=True,physical_limit_evidence='synthetic fixture')
            result=run_acquisition(reader,plan,make_event_writer(stream,lambda row:None),cancel,
                acknowledged=ack,clock=clock.now,sleep=clock.sleep,
                progress=(lambda text:hook(text,cancel)) if hook else None)
            return result,bus.history,[json.loads(line) for line in stream.getvalue().splitlines()]
        finally:reader.close()

    def test_finite_bidirectional_collection_real_session_and_all_axis_tags(self):
        result,history,audit=self.trial()
        self.assertTrue(result['campaign_complete'],result)
        self.assertTrue(result['fresh_stop_confirmation'])
        self.assertGreaterEqual(result['collected_points'],3)
        self.assertFalse(result['simulated_mass_used'])
        self.assertFalse(result['full_range_gravity_model_validated'])
        observations=[r for r in audit if r['kind']=='AUTO_LOAD_OBSERVATION']
        self.assertTrue(observations)
        for r in observations:
            self.assertEqual([a['joint'] for a in r['axes']],[1,2,3,4])
            for a in r['axes'][1:]:
                self.assertFalse(a['stationary_candidate'])
                self.assertIsNone(a['request_estimate_model_nm'])
                self.assertFalse(a['static_fit_qualified'])
        for _,cid,p in history:
            if cid==0x7ff:self.assertIn(p[0],(1,2,3,4))
            elif p[:7]!=b'\xff'*7 or p[-1]==0xfc:self.assertEqual(cid,1)

    def test_no_route_ack_no_enable(self):
        r,history,_=self.trial(ack=False)
        self.assertIn('error',r)
        self.assertFalse(any(p==b'\xff'*7+b'\xfc' for _,_,p in history))

    def test_four_nm_declared_disturbance_is_collected_without_raising_total_guard(self):
        result,_,audit=self.trial(disturbance_nm=4.,scan_limits_deg=(-20.,20.))
        self.assertTrue(result['campaign_complete'],result)
        self.assertEqual(result['bias_cap_nm'],4.)
        self.assertTrue(result['qualified_points'])
        self.assertTrue(all(abs(p['holding_request_estimate_model_nm'])<8. for p in result['qualified_points']))

    def test_initial_load_after_gain_ramp_can_settle_before_scan(self):
        result,_,audit=self.trial(disturbance_nm=4.,scan_limits_deg=(-20.,20.),
            disturbance_start_time=11.4,disturbance_ramp_s=.2)
        self.assertTrue(result['campaign_complete'],result)

    def test_cancel_no_resume_and_confirm_stop(self):
        r,history,_=self.trial(hook=lambda text,event:event.set())
        self.assertFalse(r['campaign_complete'])
        self.assertTrue(r['fresh_stop_confirmation'],r)
        self.assertEqual(r['checkpoint']['phase'],'HALTED')
        self.assertEqual(sum(p==b'\xff'*7+b'\xfc' for _,_,p in history),1)

    def test_unknown_stop_retains_only_pending_points(self):
        def hook(text,event):
            if text.startswith('已记录'):event.set()
        result,_,_=self.trial(hook=hook,ignore_fd=True)
        self.assertEqual(result['final_controller_state'],'UNKNOWN',result)
        self.assertGreater(result['collected_points'],0)
        self.assertFalse(result['qualified_points'])
        self.assertFalse(result['campaign_complete'])
        self.assertFalse(result['fresh_stop_confirmation'])

    def test_archive_requires_stop_and_keeps_measured_pose_scope(self):
        result,_,_=self.trial()
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);source=d/'batch.json';source.write_text(json.dumps(result),encoding='utf-8')
            records=save_campaign_points(result,source,pose_label='synthetic posture',
                load_note='declared test disturbance',conditions_confirmed=True,path=d/'records.jsonl')
            self.assertEqual(len(records),result['collected_points'])
            self.assertTrue(all(not r['auto_apply_authorized'] for r in records))
            self.assertEqual(len(inventory(d/'records.jsonl')[1]),len(records))
            with self.assertRaises(ValueError):
                save_campaign_points({**result,'fresh_stop_confirmation':False},source,
                    pose_label='bad',load_note='bad',conditions_confirmed=True,path=d/'bad.jsonl')


if __name__=='__main__':unittest.main()
