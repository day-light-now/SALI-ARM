"""Collect the numbers quoted in the delivery report (offline only)."""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from slai_console.j4_control import classify_motion, symmetric_slew, shape_torque
from slai_console.actuation import PD_TIERS

RES = math.degrees(2 * 12.5 / 65535)
for name, path in (
        ('1Nm/0.1', 'tmp/live_trial/20261101_154949_b50554.json'),
        ('1Nm/0.2', 'tmp/live_trial/20261101_155439_df9c6a.json'),
        ('2Nm/0.2', 'tmp/live_trial/20261101_155927_e9ad73.json')):
    result = json.loads((ROOT / path).read_text(encoding='utf-8'))
    metrics = result['motion_metrics']
    codes = metrics['position_codes']
    print(name, 'codes', len(codes), 'span_codes', max(codes) - min(codes),
          'span_deg %.4f' % metrics['total_span_deg'],
          'class', classify_motion(codes, RES),
          'peak %.4f' % metrics['peak_encoded_nm'],
          'trigger/reason', result.get('stop_reason_code'),
          'cap', result['torque_cap_nm'])

control = PD_TIERS['kp32_2nm_brake_candidate'].control
old = 0
value = 1.2
while value > 1e-9:
    value = symmetric_slew(value, 0., .01, 1.)
    old += 1
new = 0
value = 1.2
while value > 1e-9:
    value = shape_torque(requested=0., previous=value, dt=.01, cap=2.,
                         control=control, tracking=True, motion_confirmed=True,
                         model_velocity=.2088, reference_velocity=0.)['torque']
    new += 1
print('release cycles old/new', old, new, 'seconds', old * .01, new * .01)

sys.path.insert(0, str(ROOT / 'tmp' / 'j4_analysis'))
from slai_console.test_j4_control import simulate  # noqa: E402

for label, fs, fk, J in (('high-stiction', 1.2, .8, .15), ('medium', .9, .5, .15)):
    old = simulate(None, fs=fs, fk=fk, legacy_slew=1., J=J)
    new = simulate(control, fs=fs, fk=fk, J=J)
    print(label, 'old', {k: (round(v, 4) if isinstance(v, float) else v)
                         for k, v in old.items() if k != 'history'})
    print(label, 'new', {k: (round(v, 4) if isinstance(v, float) else v)
                         for k, v in new.items() if k != 'history'})
