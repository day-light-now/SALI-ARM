"""Print per-waypoint residual tables for JV baseline and compensated JVc2."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / 'j5_compensation_result_20261110.json').read_text(encoding='utf-8'))
base = {row['index']: row for row in data['baseline_windows']}
comp = {row['index']: row for row in data['compensated_windows']}
indices = sorted(set(base) | set(comp))
print('idx  desired(J3,J5)       baseline_res(J3,J5)   comp_res(J3,J5)')
for index in indices:
    b = base.get(index)
    c = comp.get(index)
    want = (b or c)['desired']
    b_res = b['residual'] if b else None
    c_res = c['residual'] if c else None
    def fmt(values):
        return '[%7.3f %7.3f]' % (values[2], values[4]) if values else '        n/a        '
    print('%2d  [%6.2f %7.2f] %s %s' % (index, want[2], want[4], fmt(b_res), fmt(c_res)))
