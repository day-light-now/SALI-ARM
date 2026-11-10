"""Synchronous static capture alongside the existing guarded ROS client.

This module never sends a motor command or changes a policy/model parameter.
"""
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import time

import calibration_core as core
from calibration_capture import _window_aggregate, _open_raw, provenance

GROUPS = {
    'G01': [[0, 0, 0, -30], [0, 0, 0, 30]],
    'G02': [[-15, 15, 0, 0], [15, 15, 0, 0]],
    'G03': [[0, 10, 0, 0], [0, 30, 0, 0]],
    'G04': [[0, 20, -20, 0], [0, 20, 20, 0]],
    'G05': [[0, 20, 0, -20], [0, 20, 0, 20]],
    'G06': [[15, 30, 20, -20], [-15, 15, -20, 20]],
    'G07': [[10, 25, -10, -15], [-10, 10, 10, 15]],
}

_training_targets = [pose for name in ('G01', 'G02', 'G03', 'G04', 'G05', 'G06')
                     for pose in GROUPS[name]]
# 大角度补采组（2026-11-06 计划）：J1 40…65°、J2 35…45°，含 J3/J4 组合；
# 每组 3 个目标 × 正反来向 4 路点，L01–L04 训练、V02 独立留出。
LARGE_ANGLE_TRAINING = {
    'L01': [[50, 15, 0, 0], [60, 20, 0, 0], [65, 10, 0, 0]],
    'L02': [[35, 40, 0, 0], [45, 45, 10, 0], [55, 35, -10, 0]],
    'L03': [[50, 30, 20, 20], [60, 25, -20, -20], [45, 45, 0, 30]],
    'L04': [[40, 15, 25, -25], [60, 40, -25, 25], [65, 30, 10, -15]],
}
LARGE_ANGLE_VALIDATION = {'V02': [[58, 22, 5, 10], [42, 38, -15, -10]]}
HOLDOUT_GROUPS = ('G07', 'V01', 'V02')
BIDIRECTIONAL_GROUPS = {
    'S01': _training_targets[0:3],
    'S02': _training_targets[3:6],
    'S03': _training_targets[6:9],
    'S04': _training_targets[9:12],
    'V01': GROUPS['G07'],
    **LARGE_ANGLE_TRAINING,
    **LARGE_ANGLE_VALIDATION,
}


# A future J5/3.5kg profile can register a qualified group here (or in the
# optional config file) with load_id + mass profile hash + J5 geometry.
QUALIFIED_GROUPS = {}
QUALIFICATION_FILE = Path(__file__).resolve().parent.parent / "config" / "calibration_qualification.json"


def _load_qualified_groups():
    if not QUALIFICATION_FILE.is_file():
        return
    try:
        payload = json.loads(QUALIFICATION_FILE.read_text(encoding="utf-8"))
        groups = payload.get("groups") if isinstance(payload, dict) else None
        if not isinstance(groups, dict):
            return
        for name, spec in groups.items():
            if isinstance(name, str) and isinstance(spec, dict) \
                    and isinstance(spec.get("waypoints"), list) \
                    and isinstance(spec.get("load_id"), str) \
                    and isinstance(spec.get("mass_profile_sha256"), str) \
                    and isinstance(spec.get("model_sha256"), str) \
                    and isinstance(spec.get("mapping_sha256"), str) \
                    and isinstance(spec.get("j5_geometry_sha256"), str) \
                    and spec.get("j5_geometry") is True:
                QUALIFIED_GROUPS[name] = copy.deepcopy(spec)
    except (OSError, ValueError):
        return


_load_qualified_groups()


def group_qualification(name):
    spec = QUALIFIED_GROUPS.get(name)
    return copy.deepcopy(spec) if isinstance(spec, dict) else None


def qualified_group_names():
    return list(QUALIFIED_GROUPS)


def _qualified_document(spec):
    return {"coordinates": "absolute_deg", "waypoints": copy.deepcopy(spec["waypoints"])}


def bidirectional_document(targets):
    rows = []
    for target in targets:
        for offset in (-3., 0., 3., 0.):
            rows.append({'degrees': [q + offset for q in target],
                         'move_s': 7.5, 'dwell_s': 6.})
    return {'coordinates': 'absolute_deg', 'waypoints': rows}


def paired_estimates(points):
    """Local bidirectional arithmetic, retaining both measured poses."""
    pairs = {}
    for point in points:
        if point.get('qualified') and point.get('approach') in ('positive', 'negative'):
            pairs.setdefault(point['pair_id'], {})[point['approach']] = point
    result = []
    for pair_id, rows in pairs.items():
        if set(rows) != {'positive', 'negative'}:
            continue
        positive, negative = rows['positive'], rows['negative']
        result.append({
            'pair_id': pair_id, 'target_deg': positive['target_deg'],
            'positive_actual_deg': positive['actual_deg'],
            'negative_actual_deg': negative['actual_deg'],
            'midpoint_nm': [(a+b)/2 for a,b in zip(positive['effort_estimate_nm'], negative['effort_estimate_nm'])],
            'half_difference_nm': [(a-b)/2 for a,b in zip(positive['effort_estimate_nm'], negative['effort_estimate_nm'])],
            'pose_difference_deg': [a-b for a,b in zip(positive['actual_deg'], negative['actual_deg'])],
        })
    return result


def effective_document(document):
    result = copy.deepcopy(document)
    for row in result['waypoints']:
        row['move_s'] = max(7.5, row['move_s'])
        row['dwell_s'] = max(6., row['dwell_s'])
    return result


def group_document(name):
    if name in QUALIFIED_GROUPS:
        return _qualified_document(QUALIFIED_GROUPS[name])
    if name in BIDIRECTIONAL_GROUPS:
        return bidirectional_document(BIDIRECTIONAL_GROUPS[name])
    if name == 'G01-G06':
        return {'coordinates':'absolute_deg', 'waypoints':[
            {'degrees':list(q), 'move_s':7.5, 'dwell_s':6.}
            for group in ('G01','G02','G03','G04','G05','G06') for q in GROUPS[group]]}
    return {'coordinates': 'absolute_deg', 'waypoints': [
        {'degrees': list(q), 'move_s': 10., 'dwell_s': 6.} for q in GROUPS[name]]}


class CalibrationSession:
    """Collect only during each frozen dwell, at its measured pose, alpha=0."""

    def __init__(self, recorder, out, command_id, windows, movement_at, plan_sha256,
                 load_id='no_end_load'):
        self.recorder = recorder
        self.command_id = command_id
        self.windows = windows
        self.movement_at = movement_at
        self.boot_id = (recorder._runtime_view().get('identity') or {}).get('boot_id')
        if not self.boot_id or not recorder._runtime_view().get('verified'):
            raise RuntimeError('CALIBRATION_IDENTITY_UNVERIFIED')
        self.load_id = str(load_id)
        self.model = provenance(self.load_id)
        self.buffers = [deque() for _ in windows]
        self.points = [dict(index=i+1, target_deg=[math.degrees(q) for q in w['target_rad']],
                            qualified=False, reason='尚未取得持续静态窗口')
                       for i, w in enumerate(windows)]
        group = self._group_name()
        if group in BIDIRECTIONAL_GROUPS:
            for index, point in enumerate(self.points):
                phase = index % 4
                point.update(calibration_group=group, pair_id='%s_%02d' % (group, index//4+1),
                             approach=('positive' if phase == 1 else 'negative' if phase == 3 else 'conditioning'))
        self.last_sequence = None
        self.last_sample_at = [-1.] * len(windows)
        self.path, self.stream = _open_raw(out, 'web_static')
        self.stream.write(json.dumps(dict(schema=core.SCHEMA_CAPTURE,
            tool='web_calibration', recorded_at=time.time(), model=self.model,
            runtime=recorder._runtime_view(), load_id=self.load_id,
            command_id=command_id, plan_sha256=plan_sha256,
            requested_count=len(windows)))+'\n')
        self.stream.flush()

    def observe(self, now):
        sample = self.recorder._build()
        elapsed = now-self.movement_at
        for index, window in enumerate(self.windows):
            point = self.points[index]
            if point['qualified']:
                continue
            if elapsed > window['end_s']:
                raise RuntimeError('CALIBRATION_POINT_INSUFFICIENT:'+str(index+1)+':'+point['reason'])
            # Exclude arrival transients and leave time to persist before leaving.
            if not window['arrival_s']+1. <= elapsed <= window['end_s']-.2:
                continue
            buffer = self.buffers[index]
            ff = (sample or {}).get('driver_ff') or {}
            runtime = (sample or {}).get('runtime') or {}
            reference = (sample or {}).get('controller_reference_rad')
            valid = bool(sample and sample['stage'] == 6 and sample['motor_state'] == [1]*4
                and runtime.get('verified') and (runtime.get('identity') or {}).get('boot_id') == self.boot_id
                and ff.get('alpha') == 0 and ff.get('stage') == 6
                and ff.get('command_id') == self.command_id
                and sample.get('command_id') == self.command_id
                and 0 <= ff.get('age_s', math.inf) <= core.FF_MAX_AGE_S
                and reference and sample.get('controller_reference_age_s', math.inf) <= .12
                and max(abs(a-b) for a,b in zip(reference, window['target_rad'])) < 1e-6)
            if not valid:
                buffer.clear()
                point['reason'] = '反馈/命令/运行身份/前馈时效或目标参考未一致'
                continue
            if sample['dynamic_sequence'] == self.last_sequence:
                continue
            if now-self.last_sample_at[index] < .095:
                continue
            self.last_sequence = sample['dynamic_sequence']
            self.last_sample_at[index] = now
            buffer.append(sample)
            while buffer and now-buffer[0]['_monotonic'] > 2.2:
                buffer.popleft()
            if len(buffer) < 10 or buffer[-1]['_monotonic']-buffer[0]['_monotonic'] < 2.:
                point['reason'] = '正在等待至少2秒持续静态窗口'
                continue
            aggregate = _window_aggregate(list(buffer))
            if not aggregate['valid']:
                point['reason'] = ('窗口不稳：' + '；'.join(aggregate.get('reasons') or []) +
                    '（n=%d span=%.2fs max|dq|=%.4f rad/s pos_delta=%.5f rad gap=%.3fs stage=%s cc=%d）' % (
                        aggregate['samples'], aggregate['span_s'], aggregate['max_speed_rad_s'],
                        aggregate['max_position_delta_rad'], aggregate['max_gap_s'],
                        aggregate['stage'], len(aggregate['command_ids'])))
                continue
            record = {key:value for key,value in sample.items() if not key.startswith('_')}
            record.update(schema=core.SCHEMA_SAMPLE, recorded_at=time.time(), support='free',
                contact='none', motion='static', load_id=self.load_id, model=self.model,
                note='网页显式确认自由承重无接触；按实际姿态拟合，不使用目标姿态代替反馈',
                static_window=aggregate, positions_rad=aggregate['mean_positions_rad'],
                velocities_rad_s=aggregate['mean_velocity_rad_s'],
                effort_estimate_nm=aggregate['mean_effort_estimate_nm'],
                target_rad=window['target_rad'], waypoint_index=index+1,
                role='holdout' if self._group_name() in HOLDOUT_GROUPS else 'training')
            for key in ('calibration_group', 'pair_id', 'approach'):
                if key in point:
                    record[key] = point[key]
            if point.get('approach') in ('positive', 'negative'):
                previous = self.points[index-1]
                direction = 1 if point['approach'] == 'positive' else -1
                movement = [math.degrees(q)-p for q,p in zip(record['positions_rad'], previous.get('actual_deg', []))]
                if len(movement) != 4 or any(direction*delta < .5 for delta in movement):
                    point['reason'] = '实测预接近点与采样点未形成四轴预期方向的位移'
                    continue
                record['approach_displacement_deg'] = movement
            ok, reasons = core.validate_sample(record)
            exclusion = core.exclusion_reason(record) if ok else None
            if not ok or exclusion:
                point['reason'] = ';'.join(reasons) if not ok else exclusion
                continue
            self.stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
            self.stream.flush()
            os.fsync(self.stream.fileno())
            point.update(qualified=True, reason=None, samples=aggregate['samples'],
                span_s=aggregate['span_s'], actual_deg=[math.degrees(q) for q in record['positions_rad']],
                effort_estimate_nm=record['effort_estimate_nm'])
            print('标定点%d/%d：合格，已写入原始记录（实际姿态 %s）' % (
                index+1, len(self.points), [round(q,3) for q in point['actual_deg']]), flush=True)

    def _group_name(self):
        targets=[point['target_deg'] for point in self.points]
        rows_by_name=dict(GROUPS)
        rows_by_name['G01-G06']=[q for key in ('G01','G02','G03','G04','G05','G06') for q in GROUPS[key]]
        for name, poses in BIDIRECTIONAL_GROUPS.items():
            rows_by_name[name] = [row['degrees'] for row in bidirectional_document(poses)['waypoints']]
        for name, rows in rows_by_name.items():
            if len(rows)==len(targets) and all(abs(a-b)<1e-6 for q,r in zip(rows,targets) for a,b in zip(q,r)):
                return name
        return 'CUSTOM'

    def finish(self, action_succeeded):
        if not self.stream.closed:
            try:
                self.stream.flush()
                os.fsync(self.stream.fileno())
            finally:
                self.stream.close()
                os.chmod(self.path, 0o444)
        count=sum(point['qualified'] for point in self.points)
        complete=action_succeeded and count==len(self.points)
        return dict(status='QUALIFIED_SAMPLES' if complete else 'INCOMPLETE',
            campaign_complete=complete, qualified_points=count, requested_points=len(self.points),
            group=self._group_name(), role='holdout' if self._group_name() in HOLDOUT_GROUPS else 'training',
            paired_targets=paired_estimates(self.points),
            raw_file=str(self.path), points=self.points,
            calibrated=False, alpha_changed=False)
