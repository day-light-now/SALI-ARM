"""Summarize one protected-client result JSON. Offline, read-only."""
import glob
import json
import sys

directory = sys.argv[1]
files = sorted(glob.glob(directory + '/*.json'))
if not files:
    print('NO_CLIENT_JSON in', directory)
    sys.exit(1)
for path in files:
    result = json.load(open(path, encoding='utf-8'))
    print('file', path)
    print(' mode', result.get('mode'), 'reason', result.get('reason'))
    print(' trajectory_error', result.get('trajectory_error_code'),
          result.get('trajectory_error_string'))
    print(' jtc_success', result.get('standard_jtc_action_succeeded'))
    print(' final_delta_from_start_deg',
          [round(v, 3) for v in result.get('final_delta_from_start_deg', [])])
    print(' initial_model_rad',
          [round(v, 5) for v in result.get('initial_model_rad', [])])
    waypoints = result.get('waypoint_results', [])
    print(' waypoints', len(waypoints))
    for index, waypoint in enumerate(waypoints):
        passed = waypoint.get('passed')
        errors = [round(v, 2) for v in waypoint.get('max_error_deg', [])]
        print(f'  w{index} arrival={waypoint.get("arrival_s")} passed={passed} '
              f'samples={waypoint.get("samples")} maxerr_deg={errors}')
    last = result.get('last_observed_status', {})
    print(' last motor_state', {name: value.get('motor_state')
                                for name, value in last.items()})
    receipt = result.get('backend_arm_receipt', '')
    if 'audit_file=' in receipt:
        print(' audit_file', receipt.split('audit_file=')[1])
    print()
