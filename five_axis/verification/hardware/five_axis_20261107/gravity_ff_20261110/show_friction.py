"""Print the per-joint friction summary from friction_estimates_20261110.json."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / 'friction_estimates_20261110.json').read_text(encoding='utf-8'))
for joint, entry in data['joints'].items():
    print('%s bins=%d mean=%.3f std=%.3f range=[%.3f, %.3f] Nm q=[%.1f, %.1f] deg' % (
        joint, len(entry['bins']), entry['friction_mean_nm'], entry['friction_std_nm'],
        entry['friction_min_nm'], entry['friction_max_nm'],
        entry['q_range_deg'][0], entry['q_range_deg'][1]))
print('missing:', [f'J{i}' for i in range(1, 6) if f'J{i}' not in data['joints']])
