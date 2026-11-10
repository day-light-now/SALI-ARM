"""Exploratory probe of the 2026-11-09 recordings. Offline only, no hardware."""
import gzip
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
ACQ = ROOT / 'verification/hardware/five_axis_20261107/acquisition_20261109'

for name in ('j5_excitation_20261109', 'ident_c_20261109'):
    path = ACQ / 'raw' / (name + '.jsonl.gz')
    kinds = Counter()
    first = {}
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            r = json.loads(line)
            kinds[r.get('kind')] += 1
            first.setdefault(r.get('kind'), r)
    print('=' * 70)
    print(name, dict(kinds))
    for kind, row in first.items():
        text = json.dumps(row, ensure_ascii=False)
        print('--', kind, '--')
        print(text[:1800])
