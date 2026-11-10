"""Compare previous GC01-GC04 client per-waypoint results. Offline read-only."""
import glob
import json
import sys

base = ('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/hardware/'
        'five_axis_20261107/gravity_compensation_trajectories_20261109/execution_20261109/clients')
for tag in ('gc01f', 'gc02', 'gc03', 'gc04'):
    files = sorted(glob.glob(base + '/' + tag + '/*.json'))
    if not files:
        print(tag, 'NO CLIENT FILE')
        continue
    result = json.load(open(files[0], encoding='utf-8'))
    waypoints = result.get('waypoint_results', [])
    passed = sum(1 for w in waypoints if w.get('passed'))
    print(tag, 'mode', result.get('mode'), 'reason', result.get('reason'),
          'passed', passed, '/', len(waypoints))
    for index, waypoint in enumerate(waypoints):
        print('  w%d passed=%s maxerr=%s' % (
            index, waypoint.get('passed'), [round(v, 2) for v in waypoint.get('max_error_deg', [])]))
