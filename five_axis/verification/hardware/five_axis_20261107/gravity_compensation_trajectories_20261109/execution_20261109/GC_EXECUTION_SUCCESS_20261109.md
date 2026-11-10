# GC trajectory execution success (2026-11-09)

All four Codex trajectories completed from a fresh admission.

## Fix applied before the successful runs

The real failing exception was:

```text
SERIAL_OR_AUDIT_WRITE_FAILURE action=MOTOR: CANUSB incomplete serial packet write
```

with a write phase of about 15.7 ms. This was a real USB-IP serial write stall,
not only the audit queue. The audit queue batching fix was also kept.

Final transport setting used for these runs:

```text
serial write deadline: 20 ms
audit queue: 8192, batched 256 KiB writes
```

This is an experimental transport setting; it does not change the 4° guard,
STOP path, limits, or freshness checking.

## Results

| trajectory | waypoint windows | final absolute zero | notes |
|---|---:|---|---|
| GC01 J1/J2 paired | 9/9 | passed | intermediate 10°/20° windows still have >0.5° errors |
| GC02 J3/J4 paired | 9/9 | passed | intermediate J4 windows show >0.5° errors |
| GC03 mixed holdout | 10/10 | passed | intermediate mixed poses show >0.5° errors |
| GC04 staged 40° | 9/9 | passed | intermediate 40° region has 2–2.6° errors |

Client result: final zero target `[0,0,0,0,0]` passed in every trajectory.
No STOP/UNKNOWN/global stop occurred during the successful runs.

## Important limits

- No gravity feedforward was enabled; these are baseline trajectories.
- `effort` remains an estimate, not measured current.
- Intermediate waypoints are not all within 0.5°; only the final zero windows
  passed the client check.
- No thermal, saturation, or 1.5× conclusion.
- Driver is stopped after the runs.

Raw state JSONL, clients, and earlier failed attempts are in
`execution_20261109/`.
