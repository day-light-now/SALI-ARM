"""Generate offline follow-up candidates; no panel import or hardware execution."""
from __future__ import annotations
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
sys.path.insert(0, str(BASE / 'gravity_sampling_v2_20261109'))
import make_sampling_plans as previous

OUT = HERE / 'next_sampling_candidates'
ZERO = [0.] * 5
SPECS = [
    ('GC09_J1_PAIRED_REPEAT_ZERO', 0, 'priority_1_upstream_direction_identification'),
    ('GC10_J4_PAIRED_REPEAT_ZERO', 3, 'priority_1_upstream_direction_identification'),
    ('GC11_J2_PAIRED_REPEAT_ZERO', 1, 'priority_2_upstream_direction_identification'),
    ('GC12_J3_PAIRED_REPEAT_ZERO', 2, 'priority_2_upstream_direction_identification'),
]


def main():
    OUT.mkdir(exist_ok=True)
    limits, identity = previous.bounds()
    entries = []
    for name, axis, role in SPECS:
        poses = []
        for value in (0., 10., 20., 10., 0., 10., 20., 10., 0.):
            q = ZERO.copy()
            q[axis] = value
            poses.append(q)
        plan = {'schema': 'slaihand_joint_plan/2', 'coordinates': 'absolute_deg',
            'waypoints': [{'degrees': q, 'move_s': 8., 'dwell_s': 8. if i == len(poses)-1 else 5.}
                          for i, q in enumerate(poses)]}
        pairs = [{'waypoint_indices_zero_based': [a, b], 'axis_one_based': axis+1,
                  'target_deg': 10., 'approach_signs': [1, -1]} for a, b in ((1, 3), (5, 7))]
        entries.append(write(name, plan, role, pairs, limits))
    # A future final holdout, frozen before acquiring any of its effort values.
    name = 'GC13_NEW_MIXED_J5_HOLDOUT_ZERO'
    plan = previous.build([5., 15., 5., 15.])
    pairs = [{'waypoint_indices_zero_based': [a, b], 'axis_one_based': 5,
              'target_deg': plan['waypoints'][a]['degrees'][4],
              'approach_signs': [int(math.copysign(1, plan['waypoints'][a]['degrees'][4]-plan['waypoints'][a-1]['degrees'][4])),
                                 int(math.copysign(1, plan['waypoints'][b]['degrees'][4]-plan['waypoints'][b-1]['degrees'][4]))]}
             for a, b in ((2, 4), (6, 8), (5, 9))]
    entries.append(write(name, plan, 'future_final_holdout_do_not_train_on', pairs, limits))
    manifest = {'schema': 'slaihand_gravity_followup_candidates/1', 'source_commit': 'e48f2c6',
        'hardware_executed': False, 'panel_imported': False, 'collision_checked': False,
        'compensation_enabled': False, 'identity': identity,
        'entry_feedback_max_abs_deg': 2., 'plans': entries,
        'purpose': 'test upstream direction confounding and repeatability before a spatial COM claim',
        'preserved_requirements': ['TIMEOUT readback 2000; verify-existing zero-write mode allowed',
            'fresh disabled admission', '4deg/STOP/limits/freshness unchanged',
            'absolute zero final reference; on fault STOP wins, no forced return'],
        'sequence': 'GC09/10 first; inspect results before GC11/12; lock geometry/model before acquiring GC13'}
    (OUT/'PLAN_MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def write(name, plan, role, pairs, limits):
    checks = [previous.validate(plan, limits, start) for start in
              ([0.]*5, [1., -1., .5, 1., 1.], [-2., 2., -2., 2., -2.])]
    for pair in pairs:
        a, b = pair['waypoint_indices_zero_based']
        if plan['waypoints'][a]['degrees'] != plan['waypoints'][b]['degrees']:
            raise ValueError('pair targets differ')
        if pair['approach_signs'][0] * pair['approach_signs'][1] != -1:
            raise ValueError('directions are not opposed')
    path = OUT / (name + '.json')
    path.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return {'name': name, 'file': path.name, 'sha256': previous.sha(path), 'role': role,
        'waypoint_count': len(plan['waypoints']), 'paired_windows': pairs,
        'duration_s': checks[0]['total_s'], 'session_reserve_s': checks[0]['session_reserve_s'],
        'max_reference_peak_rad_s': max(c['reference_peak_rad_s'] for c in checks),
        'final_absolute_deg': ZERO, 'compiled_final_rad': checks[0]['compiled_final_rad']}


if __name__ == '__main__':
    main()
