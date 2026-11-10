"""Recorder duration for a plan: total motion + dwell + margin."""
import json
import math
import sys
from pathlib import Path

document = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8-sig'))
total = sum(float(w.get('move_s', 0)) + float(w.get('dwell_s', 0)) for w in document['waypoints'])
print(int(math.ceil(total)) + 20)
