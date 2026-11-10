"""Read-only retrospective diagnostics; does not create qualified calibration samples."""
import json,math,statistics
from pathlib import Path
from collections import defaultdict,deque
ROOT=Path('/home/xuyu/.ros/slaihand')
OUT=Path(__file__).resolve().parent/'evidence/gravity_groups_20261105_audit.json'
commands={};results=[]
for d in (ROOT/'control_panel/operations').iterdir():
 for p in (d/'client').glob('20261105_*.json'):
  s=json.loads(p.read_text())
  if 'canusb_motion_KHKK0A.jsonl' not in s.get('backend_arm_receipt',''):continue
  row={'file':str(p),'command_id':s.get('command_id'),'plan':s.get('plan'),
       'reason':s.get('reason'),'waypoints_observed':s.get('waypoints_observed'),
       'waypoint_results':s.get('waypoint_results'),'stop_confirmed':s.get('stop_observations_confirmed')}
  results.append(row);commands[s.get('command_id')]=row
samples=defaultdict(list);reasons=[];latest={};zeros=None;meta=None;last_sample={}
log=ROOT/'canusb_motion/canusb_motion_KHKK0A.jsonl'
size=log.stat().st_size
with log.open() as f:
 while f.tell()<size:
  line=f.readline()
  if not line:break
  try:r=json.loads(line)
  except json.JSONDecodeError:continue
  k=r['kind'];cmd=r.get('command_id');t=r.get('steady_s',0)
  if k=='SESSION':meta=r;zeros=[a['zero'] for a in r['joint_policy']]
  elif k=='STAGE':
   zeros=r.get('effective_zero_rad',zeros);reasons.append({key:r.get(key) for key in ['steady_s','command_id','control_state','reason']})
  elif k=='RX' and r['can_id']==0:
   b=bytes.fromhex(r['payload']);axis=(b[0]&15)-1
   if not 0<=axis<4 or b[0]>>4!=1:continue
   vlim=25 if axis<2 else 45;tlim=200 if axis<2 else 54
   raw=((b[1]<<8)|b[2])*25/65535-12.5
   sign=meta['joint_policy'][axis]['sign']
   latest[axis]=(t,sign*(raw-zeros[axis]),sign*(((b[3]<<4)|(b[4]>>4))*2*vlim/4095-vlim),sign*((((b[4]&15)<<8)|b[5])*2*tlim/4095-tlim))
  elif k=='CONTROL_CYCLE' and r['control_state'] in (5,6) and cmd in commands:
   if t-last_sample.get(cmd,0)<.09 or len(latest)!=4:continue
   if any(t-latest[i][0]>.06 or t<latest[i][0] for i in range(4)):continue
   last_sample[cmd]=t
   samples[cmd].append({'t':t,'q':[latest[i][1] for i in range(4)],'v':[latest[i][2] for i in range(4)],'tau':[latest[i][3] for i in range(4)],'target':r['target']})
for row in results:
 cmd=row['command_id'];ss=samples[cmd];row['diagnostic_snapshots']=len(ss);row['pose_diagnostics']=[]
 for wp in (row.get('plan') or {}).get('waypoints',[]):
  goal=[math.radians(q) for q in wp['degrees']]
  candidates=[]
  for end in range(len(ss)):
   t=ss[end]['t'];window=[s for s in ss[max(0,end-25):end+1] if t-s['t']<=2.05]
   if len(window)<5 or window[-1]['t']-window[0]['t']<.8:continue
   if max(window[i+1]['t']-window[i]['t'] for i in range(len(window)-1))>.3:continue
   if max(abs(v) for s in window for v in s['v'])>.03:continue
   if max(max(s['q'][i] for s in window)-min(s['q'][i] for s in window) for i in range(4))>.005:continue
   err=max(abs(ss[end]['q'][i]-goal[i]) for i in range(4))
   candidates.append((err,window))
  item={'requested_deg':wp['degrees'],'stable_window_found':bool(candidates)}
  if candidates:
   err,w=min(candidates,key=lambda v:v[0]);item.update({'nearest_static_error_deg':math.degrees(err),'span_s':w[-1]['t']-w[0]['t'],'samples':len(w),'actual_deg':[math.degrees(statistics.mean(s['q'][i] for s in w)) for i in range(4)],'effort_estimate_nm':[statistics.mean(s['tau'][i] for s in w) for i in range(4)]})
  row['pose_diagnostics'].append(item)
report={'status':'DIAGNOSTIC_ONLY','qualified_calibration_samples_created':0,'reason':'No synchronized calibration capture/identity/window records were created by the webpage operations; retrospective decoding is diagnostic only.','audit_file':str(log),'bytes_read':size,'session':meta,'operations':sorted(results,key=lambda r:r['command_id']),'stage_transitions':reasons}
OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2))
for r in report['operations']:
 print('command',r['command_id'],r['reason'],'poses',[(x['requested_deg'],round(x.get('nearest_static_error_deg',999),2),x.get('span_s')) for x in r['pose_diagnostics']])
print('Saved diagnostic report',OUT)
