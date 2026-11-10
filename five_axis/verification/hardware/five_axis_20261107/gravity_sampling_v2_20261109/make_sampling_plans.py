"""J5 output-load bidirectional sampling plans. Local file generation only."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'src/slaihand_bringup/scripts'))
from joint_waypoint_plan import compile_plan

ZERO=[0.,0.,0.,0.,0.]
SPECS=[
    ('GC05_J5_ZERO_PAIRED_ZERO','GC05 新采集 J5双向配对 基座零姿态 · 绝对零终点',
     [0.,0.,0.,0.],'training'),
    ('GC06_J5_J2_20_PAIRED_ZERO','GC06 新采集 J5双向配对 J2=20度 · 绝对零终点',
     [0.,20.,0.,0.],'training'),
    ('GC07_J5_J3_J4_20_PAIRED_ZERO','GC07 新采集 J5双向配对 J3/J4=20度 · 绝对零终点',
     [0.,0.,20.,20.],'training'),
    ('GC08_J5_MIXED_HOLDOUT_ZERO','GC08 新采集 J5混合姿态独立验证 · 绝对零终点',
     [10.,10.,10.,10.],'independent_holdout_do_not_train_on'),
]
SWEEP=[0.,10.,20.,10.,0.,-10.,-20.,-10.,0.]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(base):
    active=[base+[q5] for q5 in SWEEP]
    poses=active if base==ZERO[:4] else [ZERO]+active+[ZERO]
    return {'schema':'slaihand_joint_plan/2','coordinates':'absolute_deg',
        'waypoints':[{'degrees':list(q),'move_s':8.,'dwell_s':8. if i==len(poses)-1 else 5.}
                     for i,q in enumerate(poses)]}


def bounds():
    urdf=ROOT/'src/slaihand_description/urdf/slaihand_four_axis.urdf'
    robot=ET.parse(urdf).getroot();limits=[]
    for i in range(1,5):
        limit=robot.find(f"joint[@name='slaihand_joint{i}']/limit")
        limits.append((float(limit.get('lower')),float(limit.get('upper'))))
    calibration=HERE.parent/'acquisition_20261109/artifacts/artifact_l.json'
    limits.append(tuple(json.loads(calibration.read_text(encoding='utf-8'))['payload']['j5']['limits_rad']))
    return limits,{'urdf_sha256':sha(urdf),'historical_j5_calibration_sha256':sha(calibration)}


def validate(plan,limits,start_deg):
    if plan['coordinates']!='absolute_deg':raise ValueError('absolute coordinates required')
    if plan['waypoints'][0]['degrees']!=ZERO or plan['waypoints'][-1]['degrees']!=ZERO:
        raise ValueError('first/final references must be absolute five-axis zero')
    if len(start_deg)!=5 or any(not math.isfinite(v) or abs(v)>2 for v in start_deg):
        raise ValueError('requires fresh near-zero entry, +/-2 degrees; otherwise replan')
    points,windows=compile_plan(plan,[math.radians(v) for v in start_deg],limits)
    previous=start_deg;peak=0.
    for w in plan['waypoints']:
        if w['degrees'][:4]!=[0,0,0,0] and min(w['degrees'][:4])<0:
            raise ValueError('candidate does not use negative upstream target angles')
        if abs(w['degrees'][4])>20:raise ValueError('J5 candidate exceeds +/-20 degrees')
        peak=max(peak,max(1.875*math.radians(abs(a-b))/w['move_s'] for a,b in zip(w['degrees'],previous)))
        previous=w['degrees']
    if peak>.15+1e-12:raise ValueError('candidate reference speed exceeds 0.15 rad/s')
    total=points[-1][0]
    if total+30>180:raise ValueError('insufficient session reserve')
    if points[-1][1]!=[0.]*5:raise ValueError('compiled final target is not zero')
    return {'total_s':total,'reference_peak_rad_s':peak,'session_reserve_s':180-total,
        'compiled_final_rad':points[-1][1],'waypoint_windows':windows}


def main():
    limits,identity=bounds();entries=[]
    for name,label,base,role in SPECS:
        plan=build(base);path=HERE/(name+'.json')
        path.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        plan=json.loads(path.read_text(encoding='utf-8'))
        checked=[validate(plan,limits,start) for start in ([0]*5,[1,-1,.5,1,1],[-2,2,-2,2,-2])]
        offset=0 if base==ZERO[:4] else 1
        # Exact repeated references approached from opposite directions in J5.
        pairs=[]
        for a,b in ((1,3),(5,7),(4,8)):
            ia,ib=a+offset,b+offset
            va,vb=plan['waypoints'][ia]['degrees'],plan['waypoints'][ib]['degrees']
            if va!=vb:raise ValueError('paired target mismatch')
            da=va[4]-plan['waypoints'][ia-1]['degrees'][4]
            db=vb[4]-plan['waypoints'][ib-1]['degrees'][4]
            if da*db>=0:raise ValueError('pair does not approach from opposite directions')
            pairs.append({'target_j5_deg':va[4],'waypoint_indices_zero_based':[ia,ib],
                          'approach_signs':[int(math.copysign(1,da)),int(math.copysign(1,db))]})
        entry={'name':name,'display_name':label,'file':path.name,'sha256':sha(path),
            'base_J1_J4_absolute_deg':base,'J5_sequence_deg':SWEEP,'role':role,
            'waypoint_count':len(plan['waypoints']),'paired_windows':pairs,
            'total_s':checked[0]['total_s'],'session_reserve_s':checked[0]['session_reserve_s'],
            'reference_peak_rad_s':max(c['reference_peak_rad_s'] for c in checked),
            'final_absolute_deg':ZERO,'final_dwell_s':8.,
            'compiled_final_rad':checked[0]['compiled_final_rad']}
        entries.append(entry)
        print(name,'points',entry['waypoint_count'],'duration_s',entry['total_s'],'pairs',len(pairs),'final',ZERO)
    manifest={'schema':'slaihand_j5_bidirectional_sampling_library/2','source_head':'0ddfb08',
        'coordinates':'absolute_deg','plans':entries,'identity':identity,
        'entry_feedback_max_abs_deg':2.,'purpose':'separate J5 approach dependence and validate changing upstream gravity orientation',
        'box_mass_kg':3.5,'mass_excludes_J5_motor':True,
        'mode':'ordinary five-axis trajectory with recorded raw state; not formal calibrated gravity mode',
        'compensation_enabled':False,'hardware_executed':False,'collision_checked':False,
        'admission':'TIMEOUT must read back 2000; fresh verify-existing evidence allowed with zero writes; no mandatory power cycle',
        'fault_behavior':'STOP/UNKNOWN retains priority, no forced return after a fault',
        'physical_identification_limit':'paired paths cannot replace a measured J5 axis/pivot or guarantee unique COM'}
    (HERE/'SAMPLING_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
