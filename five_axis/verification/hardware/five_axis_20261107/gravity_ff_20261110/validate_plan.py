"""Validate a drag-and-drop joint plan before any motion."""
import json
import sys
from pathlib import Path

LIMITS = [[-80.0, 200.0], [-10.0, 190.0], [-90.0, 90.0], [-50.0, 90.0], [-90.0, 90.0]]


def main():
    path = Path(sys.argv[1])
    try:
        document = json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError) as error:
        print('PLAN_INVALID: 无法读取 JSON: %s' % error)
        return 1
    if document.get('schema') != 'slaihand_joint_plan/2':
        print('PLAN_INVALID: schema 必须是 slaihand_joint_plan/2')
        return 1
    if document.get('coordinates') != 'absolute_deg':
        print('PLAN_INVALID: coordinates 必须是 absolute_deg')
        return 1
    waypoints = document.get('waypoints') or []
    if not (1 <= len(waypoints) <= 12):
        print('PLAN_INVALID: 路点数必须 1..12，收到 %d' % len(waypoints))
        return 1
    for index, waypoint in enumerate(waypoints):
        degrees = waypoint.get('degrees')
        if not isinstance(degrees, list) or len(degrees) != 5:
            print('PLAN_INVALID: 路点 %d 的 degrees 必须是 5 维' % index)
            return 1
        for axis, value in enumerate(degrees):
            lower, upper = LIMITS[axis]
            if not (lower <= float(value) <= upper):
                print('PLAN_INVALID: 路点 %d J%d=%s 超限 [%s, %s]' % (
                    index, axis + 1, value, lower, upper))
                return 1
        if float(waypoint.get('move_s', 0)) <= 0 or float(waypoint.get('dwell_s', 0)) <= 0:
            print('PLAN_INVALID: 路点 %d 的 move_s/dwell_s 必须为正' % index)
            return 1
    first = waypoints[0]['degrees']
    last = waypoints[-1]['degrees']
    if any(abs(float(value)) > 0.01 for value in first):
        print('PLAN_INVALID: 首路点必须是绝对零 %s' % first)
        return 1
    if any(abs(float(value)) > 0.01 for value in last):
        print('PLAN_INVALID: 末路点必须是绝对零 %s' % last)
        return 1
    print('PLAN_OK: %d waypoints' % len(waypoints))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
