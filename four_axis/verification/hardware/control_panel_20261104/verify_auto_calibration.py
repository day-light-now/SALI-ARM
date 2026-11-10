"""HTTP -> protected client -> real ROS/JTC -> twelve PTY static captures."""
import json,math,os,tempfile
from pathlib import Path
from unittest.mock import patch
import rclpy
import verify_control_panel as f
# Separate from the existing domain-176 end-to-end suite; no physical port.
os.environ['ROS_DOMAIN_ID']='177'
from calibration_session import group_document
import calibration_core as core

def main():
 rclpy.init();observer=f.Observer();session=None;original=f.Plant
 work=Path(tempfile.mkdtemp(prefix='auto-calibration-'))
 def factory():
  p=original();p.tracking_bias=math.radians(.8);return p
 try:
  with patch.object(f,'Plant',side_effect=factory):session=f.Session('auto-capture',work,observer,domain='177')
  panel=session.panel
  code,groups=panel.call('GET','/api/calibration/groups');assert code==200 and len(groups['groups']['G01-G06']['waypoints'])==12
  summaries=[]
  names=('G01','G07') if os.environ.get('CALIBRATION_SHORT') else ('G01-G06','G07')
  if os.environ.get('BIDIRECTIONAL_CHECK'):
   names=('S01','V01')
  for batch_index,name in enumerate(names):
   if batch_index:
    assert panel.reset()[0]==202
    assert observer.wait(lambda o:o.stage()==0,5)
   doc=groups['groups'][name]
   if os.environ.get('CALIBRATION_REACTIVATION_REPRO') and name=='G01':
    doc['waypoints']=doc['waypoints'][:1]
    doc['waypoints'][0]['move_s']=7.5
   code,preview=panel.call('POST','/api/preview',{'document':doc,'execution_mode':'calibration'})
   assert code==200,preview
   ticket=preview['ticket']['id']
   confirmations={'supported':True,'clearance_and_local_power_cut':True,'native_output_not_hardware_clamp':True}
   code,denied=panel.call('POST','/api/execute',{'ticket':ticket,'document':doc,'execution_mode':'calibration','confirmations':confirmations})
   assert code==403 and denied['code']=='FREE_STATIC_REQUIRED',denied
   confirmations['free_static']=True
   code,accepted=panel.call('POST','/api/execute',{'ticket':ticket,'document':doc,'execution_mode':'calibration','confirmations':confirmations})
   assert code==202,accepted
   operation=panel.wait_operation(observer,timeout=178)
   result=operation['result'];print(name,'phase',operation['phase'],'result',result['reason'],flush=True)
   assert operation['phase']=='finished',operation
   c=result['calibration'];assert c['campaign_complete'] and c['qualified_points']==len(doc['waypoints']),c
   assert result['stop_observations_confirmed'] and not result['accuracy_passed'],result
   rows=core.load_raw(c['raw_file'])['samples']
   assert len(rows)==len(doc['waypoints'])
   for row in rows:
    ok,reasons=core.validate_sample(row);assert ok,reasons
    assert core.exclusion_reason(row) is None
    assert row['static_window']['span_s']>=2
    assert row['role']==('holdout' if name in ('G07','V01') else 'training')
   if os.environ.get('BIDIRECTIONAL_CHECK'):
    assert len(c['paired_targets'])==len(rows)//4,c
    assert [r['approach'] for r in rows[:4]]==['conditioning','positive','conditioning','negative']
   summaries.append({'group':name,'calibration':c,'precision_passed':result['accuracy_passed'],'stop_confirmed':True})
   print('PASS',name,'qualified',len(rows),'precision still failed independently; raw schema/exclusion pass',flush=True)
  assert session.plant.fc==[2]*4,session.plant.fc
  assert not session.plant.errors,session.plant.errors
  reasons=[json.loads(line).get('reason') for p in (session.dir/'audit').glob('*.jsonl') for line in p.open() if '"kind":"STAGE"' in line]
  assert 'REFERENCE_SPEED' not in reasons,reasons
  filename='evidence/auto_calibration_short_result.json' if os.environ.get('CALIBRATION_SHORT') else 'evidence/auto_calibration_pty_result.json'
  if os.environ.get('BIDIRECTIONAL_CHECK'):
   filename='evidence/bidirectional_batches_pty_result.json'
  evidence=Path(__file__).resolve().parent/filename
  evidence.write_text(json.dumps({'passed':True,'physical_port_opened':False,'domain':177,'work':str(work),'groups':summaries},ensure_ascii=False,indent=2))
  print('PASS automatic batches',names,'no reference-speed abort; two explicit executions',flush=True)
 finally:
  if session:session.close()
  observer.close();rclpy.shutdown()
if __name__=='__main__':main()
