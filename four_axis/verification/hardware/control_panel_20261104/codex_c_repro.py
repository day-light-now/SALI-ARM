"""Reviewer counterexamples; no ROS initialization, serial or motor access."""
import copy
import time
from test_calibration_core import diverse_samples, gravity
import calibration_core as core
from calibration_capture import Recorder, NAMES

base=diverse_samples()
cases={}
for label in ('moving','disabled','mixed_versions','negative_scale'):
    rows=copy.deepcopy(base)
    for i,row in enumerate(rows):
        if label=='moving':row['velocities_rad_s']=[.8]*4;row['motion']='up'
        if label=='disabled':row['stage']=8;row['effort_estimate_nm']=[0.]*4
        if label=='mixed_versions':row['model']['mapping_sha256']=('e' if i%2 else 'c')*64
        if label=='negative_scale':row['effort_estimate_nm']=[-v for v in row['effort_estimate_nm']]
    report=core.analyze(rows,gravity)
    cases[label]={'status':report['status'],'usable':report['counts']['usable'],
                  'scale':(report['fit'] or {}).get('scale')}
print(cases)

r=Recorder.__new__(Recorder)
at=time.monotonic()-.20
r.joint={'at':at,'positions':{name:0. for name in NAMES}}
r.dynamic={'at':at,'axes':{name:dict(velocity=0.,effort=0.,feedback_age=.01,
    driver_temperature=30.,motor_temperature=30.,feedback_valid=1.,control_state=8.) for name in NAMES}}
r.reference=r.reference_at=r.ff=None
snapshot=r._build()
print('cached_snapshot_age_s=0.20; returned_feedback_age=',None if snapshot is None else snapshot['feedback_age_s'])
