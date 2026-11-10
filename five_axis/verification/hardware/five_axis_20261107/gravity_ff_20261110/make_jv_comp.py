"""Generate a JV compensated plan: J1-J4 from the T1a/T1b sag model, J5 from J5 model."""
import json
from pathlib import Path

import numpy as np

from fit_compensate import features as sag_features
from fit_j5 import approach_sign, features as j5_features

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_plan(points, path, move_s=6.0, dwell_s=6.0):
    waypoints = [{'degrees': [float(v) for v in point], 'move_s': move_s, 'dwell_s': dwell_s}
                 for point in points]
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    plan = read(HERE / 'plans' / 'JV_20261110.json')
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    sag_model = read(HERE / 'sag_model_20261110.json')['model']
    j5_model = read(HERE / 'j5_sag_model_20261110.json')
    signs = approach_sign(desired)

    F = sag_features(desired)
    predicted = np.stack([F @ np.array(sag_model[str(joint)]['coef']) for joint in range(5)], axis=1)
    J5 = j5_features(desired, signs) @ np.array(j5_model['coef'])
    predicted[:, 4] = J5
    compensated = desired - predicted
    compensated[0] = desired[0]
    compensated[-1] = desired[-1]
    write_plan(compensated, HERE / 'plans' / 'JVc_20261110.json')
    report = {'schema': 'slaihand_jv_compensation_plan/1',
              'desired_deg': desired.tolist(),
              'approach_sign': signs.tolist(),
              'predicted_sag_deg': np.round(predicted, 4).tolist(),
              'compensated_deg': np.round(compensated, 4).tolist()}
    (HERE / 'jv_compensation_plan_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote plans/JVc_20261110.json')
    for index, (want, sag, cmd, sign) in enumerate(zip(desired, predicted, compensated, signs)):
        print(' w%d q=%s sign=%+.0f pred_sag=%s cmd=%s' % (
            index, np.round(want, 2).tolist(), sign,
            np.round(sag, 3).tolist(), np.round(cmd, 2).tolist()))


if __name__ == '__main__':
    main()
