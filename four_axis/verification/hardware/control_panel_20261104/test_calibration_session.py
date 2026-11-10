"""Capture at measured poses; never infer success from a completed action."""
import copy,json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[2]/'src/slaihand_bringup/scripts'))
sys.path.insert(0,str(HERE))
import calibration_session as cs
from test_calibration_core import make_sample,MODEL
from control_panel_model import preview_payload,project_result

class FakeRecorder:
 def __init__(self):
  self.sample=make_sample([.02]*4,stage=6)
  self.sample.update(command_id=7,controller_reference_rad=[0.]*4,controller_reference_age_s=.01)
  self.sample['runtime']['identity']['boot_id']='unit-boot'
  self.sample['driver_ff'].update(stage=6)
 def _runtime_view(self):return self.sample['runtime']
 def _build(self):return copy.deepcopy(self.sample)

class CaptureTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  self.rec=FakeRecorder()
  with patch.object(cs,'provenance',return_value=MODEL):
   self.session=cs.CalibrationSession(self.rec,Path(self.tmp.name),7,[{'arrival_s':10.,'end_s':16.,'target_rad':[0.]*4}],100.,'f'*64)
  self.addCleanup(lambda:self.session.finish(False))
 def feed(self,start=111.,count=31,repeated=False):
  for i in range(count):
   now=start+i*.1
   self.rec.sample.update(_monotonic=now,dynamic_sequence=0 if repeated else i)
   self.session.observe(now)
 def test_static_offset_is_measured_and_saved(self):
  self.feed();r=self.session.finish(True)
  self.assertTrue(r['campaign_complete']);self.assertEqual(r['qualified_points'],1)
  rows=[json.loads(x) for x in Path(r['raw_file']).read_text(encoding='utf-8').splitlines()]
  self.assertEqual(rows[1]['positions_rad'],[.02]*4)
  self.assertEqual(rows[1]['target_rad'],[0.]*4)
  self.assertGreaterEqual(rows[1]['static_window']['span_s'],2)
  self.assertFalse(Path(r['raw_file']).stat().st_mode&0o222)
 def test_repeated_cache_never_qualifies(self):
  self.feed(repeated=True);self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_moving_never_qualifies(self):
  self.rec.sample['velocities_rad_s']=[.1]*4
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_disabled_never_qualifies(self):
  self.rec.sample['motor_state'][0]=0
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_stale_ff_never_qualifies(self):
  self.rec.sample['driver_ff']['age_s']=120.
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_changed_command_never_qualifies(self):
  self.rec.sample['command_id']=8
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_changed_boot_never_qualifies(self):
  self.rec.sample['runtime']['identity']['boot_id']='other'
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_unknown_identity_never_qualifies(self):
  self.rec.sample['runtime']['verified']=False
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_wrong_reference_never_qualifies(self):
  self.rec.sample['controller_reference_rad']=[.1]*4
  self.feed();self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_first_disabled_frame_not_spliced(self):
  self.rec.sample['motor_state'][0]=0;self.feed(count=1)
  self.rec.sample['motor_state'][0]=1;self.feed(start=111.1,count=15)
  self.assertEqual(self.session.finish(True)['qualified_points'],0)
 def test_missing_window_halts_before_next_point(self):
  with self.assertRaisesRegex(RuntimeError,'CALIBRATION_POINT_INSUFFICIENT:1'):
   self.session.observe(116.1)
 def test_partial_capture_does_not_claim_completed_action(self):
  self.feed();r=self.session.finish(False)
  self.assertEqual(r['qualified_points'],1);self.assertFalse(r['campaign_complete'])
 def test_effective_times_shown_in_frozen_preview(self):
  p=preview_payload({'coordinates':'absolute_deg','waypoints':[{'degrees':[0]*4,'move_s':4,'dwell_s':.5}]},[0]*4,[(-1,1)]*4,mode='calibration')
  self.assertEqual(p['waypoints'][0]['move_s'],7.5)
  self.assertEqual(p['waypoints'][0]['dwell_s'],6)
  self.assertEqual(p['frozen']['waypoints'][0]['dwell_s'],6)
 def test_training_batch_fits_existing_session_and_waypoint_budget(self):
  doc=cs.group_document('G01-G06')
  bounds=[(math.radians(a),math.radians(b)) for a,b in [(-80,200),(-10,190),(-90,90),(-90,50)]]
  p=preview_payload(doc,[0.]*4,bounds,mode='calibration')
  self.assertEqual(len(p['waypoints']),12)
  self.assertEqual(p['total_s'],162.)
  self.assertTrue(p['budget']['ok'])
 def test_precision_failure_still_visible_with_qualified_capture(self):
  r=project_result({'mode':'calibration','reason':'finite_trial_complete','stop_observations_confirmed':True,'standard_jtc_action_succeeded':True,'trajectory_error_code':0,'waypoint_results':[{'passed':False}],'waypoints_observed':False,'calibration':{'campaign_complete':True}})
  self.assertTrue(r['overall_ok']);self.assertFalse(r['accuracy_passed'])
 def test_no_stop_confirmation_cannot_complete(self):
  r=project_result({'mode':'calibration','reason':'finite_trial_complete','standard_jtc_action_succeeded':True,'trajectory_error_code':0,'calibration':{'campaign_complete':True}})
  self.assertFalse(r['overall_ok'])

class HoldoutTests(unittest.TestCase):
 def test_explicit_holdout_does_not_change_training_fit(self):
  from test_calibration_core import diverse_samples,gravity
  import calibration_core as core
  training=diverse_samples(12,scale=1.0)
  test=diverse_samples(2,scale=3.0)
  for row in test:row['role']='holdout'
  report=core.analyze(training+test,gravity)
  self.assertEqual(report['counts']['usable'],12)
  self.assertEqual(report['counts']['independent_holdout'],2)
  self.assertEqual(report['status'],'INSUFFICIENT')
  self.assertIsNone(report['fit'])
  report=core.analyze(training+[dict(row,role='holdout') for row in diverse_samples(2)],gravity)
  self.assertEqual(report['fit']['scale'],1.)
  self.assertEqual(report['holdout']['split'],'explicit_independent')

if __name__=='__main__':unittest.main()
