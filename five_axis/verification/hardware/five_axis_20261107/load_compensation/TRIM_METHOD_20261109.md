# Pose-trim test result (2026-11-09)

## Full-sequence trim plan

A one-shot correction was generated from the prior 40° mixed-pose errors and
applied to the same 7-waypoint sequence.

The client rejected it **before motion**:

```text
waypoint exceeds full URDF limit
```

The arm was already near the J3 upper region, so adding the relative 40° target
would exceed the URDF limit. No motion occurred for this plan.

## Small J1 step test

From the then-current pose, a 3-waypoint test commanded:

```text
hold current
J1 +20° (10 s move, 8 s dwell)
J1 +21.4° (3 s move, 8 s dwell)  # +1.4° trim from prior J1 error
```

Results (client 0.3 s windows):

| pose | J1 max_abs | J4 max_abs |
|---|---:|---:|
| J1 +20° | 1.968° | 0.197° |
| J1 +21.4° | 1.925° | 0.503° |

- J1 improved only by about 0.043°;
- J4 got worse;
- JTC aborted on goal-time after the final hold.

## Verdict

The one-shot pose trim does **not** produce a meaningful reduction. A +1.4°
target correction moved the steady error by only ~0.04°, so a simple target
offset is not an effective compensation path here.

Per the "不行换下一个" rule, stop this method.

## Next candidates

1. **Physics-based static gravity identification** with many independent poses
   and identified link/payload parameters (not a simple linear offset).
2. **External per-axis current** for a real actuator-load model and electrical
   evidence.
3. A different control-level approach only after offline validation; do not
   just keep raising Kp.

No current, thermal, saturation, or 1.5× claim. Driver stopped; no panel.
