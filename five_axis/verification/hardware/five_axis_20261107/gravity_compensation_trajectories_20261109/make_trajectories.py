"""Generate/validate offline gravity-compensation trial trajectories.

Only writes local JSON/Markdown. No ROS, serial, CAN or driver operations.
All five-axis targets are absolute degrees and each final target is exact zero.
"""
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

ZERO=[0,0,0,0,0]
PLANS={
    'GC01_J1_J2_PAIRED_ZERO': {
        'label':'J1/J2 10°正反接近配对，20°转向，绝对零点结束',
        'role':'paired_diagnostic',
        'poses':[ZERO,[10,0,0,0,0],[20,0,0,0,0],[10,0,0,0,0],ZERO,
                 [0,10,0,0,0],[0,20,0,0,0],[0,10,0,0,0],ZERO],
        'pairs':[[1,3],[5,7]],
    },
    'GC02_J3_J4_PAIRED_ZERO': {
        'label':'J3/J4 10°正反接近配对，20°转向，绝对零点结束',
        'role':'paired_diagnostic',
        'poses':[ZERO,[0,0,10,0,0],[0,0,20,0,0],[0,0,10,0,0],ZERO,
                 [0,0,0,10,0],[0,0,0,20,0],[0,0,0,10,0],ZERO],
        'pairs':[[1,3],[5,7]],
    },
    'GC03_MIXED_HOLDOUT_ZERO': {
        'label':'混合姿态独立验证，10–30°，绝对零点结束',
        'role':'independent_holdout_do_not_train_on',
        'poses':[ZERO,[10,10,10,10,0],[20,20,20,20,0],[10,10,10,10,0],ZERO,
                 [20,10,30,10,0],[10,20,20,30,0],[20,20,30,20,0],
                 [10,10,10,10,0],ZERO],
        'pairs':[[1,3]],
    },
    'GC04_STAGED_40_ZERO': {
        'label':'分阶段10/20/30/40°及回程，J1不超过20°，绝对零点结束',
        'role':'later_stage_large_angle_validation',
        'poses':[ZERO,[5,10,10,10,0],[10,20,20,20,0],[15,30,30,30,0],
                 [20,40,40,40,0],[15,30,30,30,0],[10,20,20,20,0],
                 [5,10,10,10,0],ZERO],
        'pairs':[[1,7],[2,6],[3,5]],
    },
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_bounds():
    urdf=ROOT/'src/slaihand_description/urdf/slaihand_four_axis.urdf'
    robot=ET.parse(urdf).getroot()
    bounds=[]
    for i in range(1,5):
        limit=robot.find(f"joint[@name='slaihand_joint{i}']/limit")
        bounds.append((float(limit.get('lower')),float(limit.get('upper'))))
    artifact=HERE.parent/'acquisition_20261109/artifact_f.json'
    j5=json.loads(artifact.read_text(encoding='utf-8'))['payload']['j5']['limits_rad']
    return bounds+[tuple(j5)],{'urdf_sha256':sha(urdf),'j5_historical_calibration_artifact_sha256':sha(artifact)}


def document(poses):
    return {'schema':'slaihand_joint_plan/2','coordinates':'absolute_deg',
            'waypoints':[{'degrees':list(pose),'move_s':8.0,'dwell_s':8.0 if k==len(poses)-1 else 6.0}
                         for k,pose in enumerate(poses)]}


def validate(plan,bounds,start_deg):
    if plan['coordinates']!='absolute_deg' or plan['waypoints'][-1]['degrees']!=ZERO:
        raise ValueError('every final target must be absolute five-axis zero')
    if plan['waypoints'][0]['degrees']!=ZERO:
        raise ValueError('entry target must be absolute zero')
    if len(start_deg)!=5 or any(not math.isfinite(v) or abs(v)>2 for v in start_deg):
        raise ValueError('this candidate assumes fresh feedback within +/-2 deg of absolute zero')
    if any(w['degrees'][4]!=0 for w in plan['waypoints']):
        raise ValueError('J5 must remain zero in every target')
    start=[math.radians(v) for v in start_deg]
    points,windows=compile_plan(plan,start,bounds)
    peak=0.;previous=start
    for row in plan['waypoints']:
        target=[math.radians(v) for v in row['degrees']]
        peak=max(peak,max(1.875*abs(a-b)/row['move_s'] for a,b in zip(previous,target)))
        previous=target
    if peak>.15+1e-12:
        raise ValueError('candidate exceeds conservative 0.15 rad/s reference envelope')
    total=points[-1][0]
    if total+30>180:
        raise ValueError('insufficient 30s startup/stop budget under 180s session')
    if points[-1][1]!=[0.]*5 or windows[-1]['target_rad']!=[0.]*5:
        raise ValueError('compiled final target is not absolute zero')
    return {'total_s':total,'peak_quintic_reference_rad_s':peak,
            'session_reserve_s':180-total,'compiled_final_target_rad':points[-1][1],
            'windows':windows}


def main():
    bounds,identity=load_bounds();entries=[]
    for name,spec in PLANS.items():
        plan=document(spec['poses']);path=HERE/(name+'.json')
        path.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        # Re-read the actual artifacts. A nonzero initial pose proves final-zero
        # semantics are not accidentally relative to the session start.
        disk=json.loads(path.read_text(encoding='utf-8'))
        checks=[validate(disk,bounds,start) for start in ([0]*5,[1,-1,.5,1,1],[-2,2,-2,2,-2])]
        for a,b in spec['pairs']:
            if disk['waypoints'][a]['degrees']!=disk['waypoints'][b]['degrees']:
                raise ValueError('paired target differs')
        compiled=checks[0]
        entries.append({'name':name,'label':spec['label'],'role':spec['role'],
                        'file':path.name,'sha256':sha(path),'waypoint_count':len(spec['poses']),
                        'total_s':compiled['total_s'],
                        'peak_reference_rad_s':max(c['peak_quintic_reference_rad_s'] for c in checks),
                        'session_reserve_s':compiled['session_reserve_s'],
                        'paired_waypoint_indices_zero_based':spec['pairs'],
                        'final_absolute_deg':ZERO,'final_dwell_s':8.,
                        'compiled_final_absolute_rad':compiled['compiled_final_target_rad'],
                        'validated_start_examples_deg':[[0]*5,[1,-1,.5,1,1],[-2,2,-2,2,-2]]})
        print(name,'points=',len(spec['poses']),'duration_s=',compiled['total_s'],'final=',ZERO)
    manifest={'schema':'slaihand_offline_gravity_trial_trajectory_library/1',
              'generated_for_date':'2026-11-09','source_head':'99b28e9',
              'coordinates':'absolute_deg','identity':identity,
              'all_final_targets_absolute_zero':True,'zero_final_dwell_s':8.,
              'entry_feedback_max_abs_deg':2.,'j5_excitation':False,
              'minimum_operational_checks':['fresh five-axis feedback and active identity',
                  'actual entry within +/-2 deg of absolute zero; otherwise replan',
                  'current artifact/evidence and TIMEOUT readback, not archived admission',
                  'local clearance, operating limits and complete path preview',
                  'FF model must be separately validated before enabling compensation'],
              'gravity_candidate_deployable':False,'ff_enable_or_deployment_performed':False,
              'collision_checked':False,'hardware_executed':False,
              'on_fault':'existing STOP/UNKNOWN takes priority; no forced zero-return after fault',
              'plans':entries}
    (HERE/'TRAJECTORY_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
