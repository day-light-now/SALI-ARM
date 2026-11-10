"""Offline verification of compensate_plan.py across representative plans.

Checks domain guard, limits, endpoint safety and time preservation; emits
compensated copies of the in-domain demo plans. No hardware access.
"""
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'

spec = importlib.util.spec_from_file_location('compensate_plan', HERE / 'compensate_plan.py')
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def duration(plan):
    return sum(float(w['move_s']) + float(w['dwell_s']) for w in plan['waypoints'])


def check(name, output_name=None, expect_ok=True):
    plan = read(PLANS / name)
    desired = [list(w['degrees']) for w in plan['waypoints']]
    result = cp.compute(desired)
    problems = list(result['problems'])
    compensated = result['compensated']
    limits = cp.limits_deg()
    for index, row in enumerate(compensated):
        for joint, value in enumerate(row):
            joint_name = cp.NAMES[joint]
            if joint_name in limits:
                low, high = limits[joint_name]
                if value < low - 1e-9 or value > high + 1e-9:
                    problems.append('limit@w%d/%s' % (index, joint_name))
    if any(abs(v) > 1e-9 for v in compensated[0]) or any(abs(v) > 1e-9 for v in compensated[-1]):
        problems.append('non-zero endpoint')
    converted = {'schema': 'slaihand_joint_plan/2', 'coordinates': 'absolute_deg',
                 'waypoints': [{'degrees': [float(v) for v in row],
                                'move_s': float(w.get('move_s', 6.0)),
                                'dwell_s': float(w.get('dwell_s', 6.0))}
                               for row, w in zip(compensated, plan['waypoints'])]}
    if duration(converted) != duration(plan):
        problems.append('duration changed')
    ok = not problems
    if ok and output_name:
        (PLANS / output_name).write_text(
            json.dumps(converted, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return {'plan': name, 'waypoints': len(desired),
            'sag_max_deg': round(float(max(abs(float(v)) for row in result['predicted']
                                          for v in row)), 4),
            'expected_conversion': expect_ok,
            'converted': ok,
            'issues': problems,
            'ok': (ok == expect_ok),
            'compensated_plan': output_name if ok else None}


def main():
    cases = [
        ('T1a_sag_20261110.json', 'T1a_sag_comp_20261110.json', True),
        ('T1b_sag_20261110.json', 'T1b_sag_comp_20261110.json', True),
        ('V1_sag_20261110.json', 'V1_sag_comp_20261110.json', True),
        ('V1sub_sag_20261110.json', 'V1sub_sag_comp_20261110.json', True),
        ('SMOKE_7wp_20261110.json', 'SMOKE_7wp_comp_20261110.json', True),
        ('JZ_20261110.json', None, False),
        ('T2_sag_20261110.json', None, False),
        ('T2r_sag_20261110.json', None, False),
        ('V2_sag_20261110.json', None, False),
    ]
    results = []
    for name, output, expect_ok in cases:
        if not (PLANS / name).is_file():
            results.append({'plan': name, 'ok': False, 'issues': ['missing'],
                            'expected_conversion': expect_ok})
            continue
        results.append(check(name, output, expect_ok))
    report = {'schema': 'slaihand_comp_plan_offline_validation/1',
              'results': results,
              'all_ok': all(item.get('ok') for item in results)}
    (HERE / 'offline_comp_plan_validation_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    for item in results:
        status = 'OK' if item.get('ok') else 'FAIL'
        detail = '' if item.get('ok') else ' ' + ','.join(item.get('issues', []))
        print('%-30s wps=%2d sag_max=%6.3f expect=%-5s %s%s' % (
            item['plan'], item.get('waypoints', 0), item.get('sag_max_deg', 0.0),
            str(item.get('expected_conversion')), status, detail))
    print('all_ok', report['all_ok'])
    print('wrote offline_comp_plan_validation_20261110.json')


if __name__ == '__main__':
    main()
