"""Regression checks for bounded bidirectional plans and paired raw metadata."""
import json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[2]/'src/slaihand_bringup/scripts'))
sys.path.insert(0,str(HERE))
import calibration_session as cs
from control_panel_model import preview_payload
from test_calibration_session import FakeRecorder
from test_calibration_core import MODEL
BOUNDS=[(math.radians(a),math.radians(b)) for a,b in [(-80,200),(-10,190),(-90,90),(-90,50)]]

class BatchTests(unittest.TestCase):
 def test_all_batches_fit_existing_budget_and_full_limits(self):
  targets=[]
  for name,poses in cs.BIDIRECTIONAL_GROUPS.items():
   doc=cs.group_document(name)
   preview=preview_payload(doc,[0.]*4,BOUNDS,mode='calibration')
   self.assertTrue(preview['budget']['ok'])
   self.assertEqual(len(preview['waypoints']),8 if name=='V01' else 12)
   self.assertEqual(preview['total_s'],108. if name=='V01' else 162.)
   for index,target in enumerate(poses):
    rows=doc['waypoints'][index*4:index*4+4]
    self.assertEqual(rows[1]['degrees'],rows[3]['degrees'])
    self.assertEqual(rows[1]['degrees'],target)
    self.assertEqual(rows[0]['degrees'],[q-3 for q in target])
    self.assertEqual(rows[2]['degrees'],[q+3 for q in target])
   if name!='V01':targets.extend(poses)
  self.assertEqual(len(targets),12)
  self.assertEqual(len({tuple(p) for p in targets}),12)
 def make_session(self,name):
  temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
  recorder=FakeRecorder()
  preview=preview_payload(cs.group_document(name),[0.]*4,BOUNDS,mode='calibration')
  windows=[{'arrival_s':r['arrival_s'],'end_s':r['end_s'],'target_rad':[math.radians(q) for q in r['target_deg']]} for r in preview['waypoints']]
  with patch.object(cs,'provenance',return_value=MODEL):
   session=cs.CalibrationSession(recorder,Path(temporary.name),7,windows,100.,'a'*64)
  self.addCleanup(lambda:session.finish(False))
  return recorder,session
 def feed_point(self,recorder,session,index,actual=None):
  window=session.windows[index]
  actual=actual or window['target_rad']
  for count in range(31):
   now=100+window['arrival_s']+1+count*.1
   recorder.sample.update(_monotonic=now,dynamic_sequence=index*100+count,
      controller_reference_rad=window['target_rad'],positions_rad=list(actual))
   session.observe(now)
 def test_raw_records_keep_direction_and_training_role(self):
  recorder,session=self.make_session('S01')
  for index in range(4):self.feed_point(recorder,session,index)
  result=session.finish(False)
  self.assertEqual(result['group'],'S01')
  self.assertEqual(result['qualified_points'],4)
  self.assertEqual(len(result['paired_targets']),1)
  rows=[json.loads(line) for line in Path(result['raw_file']).read_text(encoding='utf-8').splitlines()][1:]
  self.assertEqual([r['approach'] for r in rows],['conditioning','positive','conditioning','negative'])
  self.assertTrue(all(row['role']=='training' for row in rows))
  for delta in rows[1]['approach_displacement_deg']:
   self.assertAlmostEqual(delta,3.)
 def test_actual_direction_must_follow_the_prepose(self):
  recorder,session=self.make_session('S01')
  self.feed_point(recorder,session,0)
  self.feed_point(recorder,session,1,actual=session.windows[0]['target_rad'])
  self.assertFalse(session.points[1]['qualified'])
  self.assertIn('方向',session.points[1]['reason'])
 def test_validation_batch_is_explicitly_held_out(self):
  recorder,session=self.make_session('V01')
  self.feed_point(recorder,session,0)
  result=session.finish(False)
  self.assertEqual(result['group'],'V01')
  self.assertEqual(result['role'],'holdout')
  row=json.loads(Path(result['raw_file']).read_text(encoding='utf-8').splitlines()[1])
  self.assertEqual(row['role'],'holdout')
 def test_missing_direction_cannot_generate_a_pair(self):
  point={'qualified':True,'pair_id':'p','approach':'positive'}
  self.assertEqual(cs.paired_estimates([point]),[])
 def test_pair_keeps_actual_pose_difference(self):
  base={'qualified':True,'pair_id':'p','target_deg':[0.]*4}
  positive=dict(base,approach='positive',actual_deg=[-.3]*4,effort_estimate_nm=[3.]*4)
  negative=dict(base,approach='negative',actual_deg=[.3]*4,effort_estimate_nm=[1.]*4)
  pair=cs.paired_estimates([positive,negative])[0]
  self.assertEqual(pair['midpoint_nm'],[2.]*4)
  self.assertEqual(pair['half_difference_nm'],[1.]*4)
  self.assertEqual(pair['pose_difference_deg'],[-.6]*4)

if __name__=='__main__':unittest.main()
