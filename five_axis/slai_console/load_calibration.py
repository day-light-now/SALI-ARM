"""Real-feedback holding calibration records. No CAD mass or gravity model.

Each record describes one full four-axis pose and the estimated holding input
for one active joint. It is not a calibrated torque measurement, a full-range
gravity model, or permission to apply a feedforward table automatically.
"""
import hashlib
import json
import math
from pathlib import Path
import statistics
import uuid

from .core import ROOT

ARCHIVE=ROOT/'verification/hardware/load_calibration/records.jsonl'


def summarize_calibration(samples,controller,result):
    tail=[s for s in samples if s['at_s']>=28. and s['state']==1]
    reasons=[];joint=controller.acceptance.joint
    if controller.state!='DISABLED' or not result.get('fresh_stop_confirmation'):
        reasons.append('stop_not_confirmed')
    if controller.reason!='calibration_hold_complete':reasons.append('hold_not_completed')
    if len(tail)<2 or tail[-1]['at_s']-tail[0]['at_s']<1.5:
        reasons.append('insufficient_stable_window')
    summary=dict(accepted=False,joint=joint,source_request_id=result['request_id'],
        simulated_mass_used=False,not_a_gravity_model=True,
        quantity='drive_request_estimate_at_feedback_state_not_measured_torque',
        reasons=reasons)
    if not tail:return summary
    poses=[[s['all_angles_rad'][i] for s in tail] for i in range(4)]
    if any(max(q)-min(q)>math.radians(.15) for q in poses):reasons.append('pose_not_stationary')
    if any(abs(s['motor_velocity_rad_s'])>.05 for s in tail):reasons.append('velocity_not_stationary')
    torque=[controller.profiles[joint-1].sign*s['command']['observed_state_estimate_nm'] for s in tail]
    bias=[(s['command'].get('holding_bias') or {}).get('model_bias_nm',0.) for s in tail]
    if max(torque)-min(torque)>.5:reasons.append('request_estimate_not_stable')
    if max(bias)-min(bias)>.15:reasons.append('bias_still_learning')
    summary.update(accepted=not reasons,pose_rad=[statistics.mean(q) for q in poses],
        holding_request_estimate_model_nm=statistics.mean(torque),
        request_estimate_range_nm=[min(torque),max(torque)],
        bias_model_nm=statistics.mean(bias),bias_range_nm=[min(bias),max(bias)],
        window_span_s=tail[-1]['at_s']-tail[0]['at_s'],sample_count=len(tail),
        mapping_sha256=controller.last_snapshot.mapping_sha256,
        profile=vars(controller.profiles[joint-1]))
    return summary


def save_record(result,trial_path,*,pose_label,approach,load_note,conditions_confirmed,path=ARCHIVE):
    """Append one qualified point after explicit physical-condition confirmation.

    No control code reads this archive automatically. Existing records remain
    evidence if the hardware mapping or payload changes, never silently reused.
    """
    point=result.get('calibration_record',{})
    if (conditions_confirmed is not True or point.get('accepted') is not True or
        result.get('final_controller_state')!='DISABLED' or
        result.get('fresh_stop_confirmation') is not True):
        raise ValueError('Qualified stable point, physical confirmation and confirmed stop required')
    if result.get('mode')!='calibration_hold' or result.get('joint') not in (1,2,3,4):
        raise ValueError('Only J1-J4 current-pose calibration results can be archived')
    if not pose_label.strip() or not load_note.strip() or approach not in ('positive','negative','unknown'):
        raise ValueError('Pose label, load note and approach direction required')
    source=Path(trial_path);mapping=ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml'
    if hashlib.sha256(mapping.read_bytes()).hexdigest()!=point['mapping_sha256']:
        raise ValueError('Mapping changed after calibration; do not mix zero/sign records')
    record=dict(point,record_id=uuid.uuid4().hex,pose_label=pose_label.strip(),
        approach=approach,load_note=load_note.strip(),conditions_confirmed=True,
        source_trial=str(source.resolve()),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        full_range_calibration_complete=False,auto_apply_authorized=False)
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    # One append per run; do not count a repeated import as a new measurement.
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            if json.loads(line).get('source_request_id')==record['source_request_id']:
                raise ValueError('This calibration trial has already been archived')
    with path.open('a',encoding='utf-8') as handle:handle.write(json.dumps(record,ensure_ascii=False)+'\n')
    return record


def inventory(path=ARCHIVE):
    rows=[json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines()] if Path(path).exists() else []
    return {i:[r for r in rows if r['joint']==i] for i in range(1,5)}


def save_campaign_points(result,trial_path,*,pose_label,load_note,conditions_confirmed,path=ARCHIVE):
    if (result.get('mode')!='auto_load_acquisition' or conditions_confirmed is not True or
        result.get('final_controller_state')!='DISABLED' or not result.get('fresh_stop_confirmation')):
        raise ValueError('Automatic points require physical confirmation and a confirmed final stop')
    if not pose_label.strip() or not load_note.strip():raise ValueError('Pose and payload labels required')
    source=Path(trial_path);digest=hashlib.sha256(source.read_bytes()).hexdigest()
    mapping=hashlib.sha256((ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml').read_bytes()).hexdigest()
    existing={r['source_request_id'] for group in inventory(path).values() for r in group}
    records=[]
    for point in result.get('qualified_points',[]):
        if point.get('accepted') is not True or point['mapping_sha256']!=mapping:
            raise ValueError('Automatic point has invalid quality or changed mapping')
        if point['joint']!=result['joint'] or point['plan_sha256']!=result['plan_sha256']:
            raise ValueError('Automatic point scope mismatch')
        rid=result['source_request_id']+':'+str(point['waypoint_index'])
        if rid in existing:raise ValueError('Automatic point already archived')
        records.append(dict(point,record_id=uuid.uuid4().hex,source_request_id=rid,
            source_trial=str(source.resolve()),source_sha256=digest,
            pose_label=pose_label.strip(),load_note=load_note.strip(),conditions_confirmed=True,
            campaign_complete=bool(result['campaign_complete']),
            full_range_calibration_complete=False,auto_apply_authorized=False))
    target=Path(path);target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('a',encoding='utf-8') as handle:
        for record in records:handle.write(json.dumps(record,ensure_ascii=False)+'\n')
    return records
