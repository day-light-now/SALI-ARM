"""Check that a run produced enough settled static windows."""
import json
import sys
from pathlib import Path

here = Path(__file__).resolve().parent
tag = sys.argv[1]
minimum = int(sys.argv[2]) if len(sys.argv) > 2 else 6
path = here / f'windows_{tag}.json'
if not path.is_file():
    print('WINDOWS_MISSING', path.name)
    raise SystemExit(1)
data = json.loads(path.read_text(encoding='utf-8'))
usable = [w for w in data['windows'] if 'sag_deg' in w and w.get('samples', 0) >= 150]
print('WINDOWS=%d USABLE=%d' % (len(data['windows']), len(usable)))
raise SystemExit(0 if len(usable) >= minimum else 1)
