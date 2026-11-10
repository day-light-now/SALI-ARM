"""Read-only analysis of the field J5 audit; never opens a serial port."""
import json
import os
from pathlib import Path

path = Path(os.path.expanduser("~/.ros/slaihand/j5_readonly_20261107/readonly.jsonl"))
rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
print("rows", len(rows))
print("first", rows[0])
print("last", rows[-1])
sessions = []
current = None
for row in rows:
    if row["direction"] == "open":
        current = {"open": row, "rows": []}
        sessions.append(current)
    if current is not None:
        current["rows"].append(row)
print("sessions", len(sessions))
for number, session in enumerate(sessions, 1):
    events = session["rows"]
    tx = [event for event in events if event["direction"] == "tx"]
    rx = [event for event in events if event["direction"] == "rx"]
    faults = [event for event in events if event["direction"] in ("fault", "open", "close")]
    gaps = []
    for left, right in zip(events, events[1:]):
        gaps.append((right["at"] - left["at"], left, right))
    gaps.sort(reverse=True, key=lambda item: item[0])
    print("SESSION", number, session["open"].get("payload"))
    print("  events", len(events), "tx", len(tx), "rx", len(rx),
          "faults", faults)
    print("  wall", events[0]["wall"], "..", events[-1]["wall"],
          "span_s", round(events[-1]["wall"] - events[0]["wall"], 3))
    print("  last tx", tx[-5:])
    print("  last rx", rx[-5:])
    print("  largest gaps")
    for gap, left, right in gaps[:8]:
        print("   ", round(gap, 4), left["direction"], left["payload"],
              "->", right["direction"], right["payload"], "wall", right["wall"])
    # 1-second bins: tx/rx density, to see starvation before a stop.
    if events:
        start = events[0]["wall"]
        bins = {}
        for event in events:
            bucket = int(event["wall"] - start)
            counts = bins.setdefault(bucket, {"tx": 0, "rx": 0})
            if event["direction"] in ("tx", "rx"):
                counts[event["direction"]] += 1
        print("  last bins", sorted(bins.items())[-8:])
