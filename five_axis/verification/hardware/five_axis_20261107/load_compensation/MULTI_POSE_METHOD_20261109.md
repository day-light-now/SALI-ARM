# Multi-pose torque-proxy experiment (2026-11-09)

Both power/CAN groups were exercised together. No collision. Driver is stopped.

## Runs

- 20 deg: 12 waypoints including single-axis and mixed poses; all sampled;
  JTC goal-time abort at the end.
- 40 deg: 7 waypoints including single-axis and mixed poses; all sampled;
  JTC goal-time abort at the end.
- Max supply current reported by operator: **0.65 A** (total supply current;
  still no per-axis current).

## Per-joint torque estimates at 40 deg (Nm)

| pose (relative) | J1 | J2 | J3 | J4 | J5 |
|---|---:|---:|---:|---:|---:|
| J1 +40 | 11.60 | 0.08 | −0.13 | 2.81 | −0.05 |
| J2 +40 | −0.22 | 11.63 | 0.06 | 2.30 | −0.05 |
| J3 +40 | −0.18 | −1.04 | 0.57 | 1.01 | −0.04 |
| J4 +40 | 2.87 | 1.91 | −0.83 | 5.16 | 0.17 |
| J5 +40 | 2.09 | 1.64 | −0.27 | −0.47 | 0.64 |
| all +40 | 8.63 | 9.43 | −1.90 | 6.76 | 0.17 |

The torque estimate responds per axis, but cross-coupling is large.

## Coupled linear model test

Fit on 14 poses (20 and 40 deg combined), leave-one-out:

| axis | max LOO error | mean LOO error |
|---|---:|---:|
| J1 | 3.89 Nm | 1.20 Nm |
| J2 | 3.36 Nm | 1.17 Nm |
| J3 | 1.79 Nm | 0.46 Nm |
| J4 | 3.55 Nm | 1.03 Nm |
| J5 | 1.03 Nm | 0.28 Nm |

The model is not accurate enough for a deployable feedforward: J1/J4 errors
are comparable to a large fraction of their holding torque.

## Verdict

- The torque-estimate acquisition method is **usable as a diagnostic**.
- The simple linear pose model is **not usable as FF**.
- Per Codex/operator "不行换下一个": switch away from this linear torque model.
- Real per-axis current is still unavailable; total supply current cannot
  identify axis load.
- No thermal, saturation, or 1.5x claim.

## Candidate next methods

1. **Static pose-trim / integral trim** — directly reduce the observed position
   error without current; needs bounded correction and independent holdout.
2. **Nonlinear physics/gravity identification** — many more poses and payload
   parameters; heavier but potentially general.
3. External per-axis current sensors — strongest evidence, but hardware work.

Recommended next: bounded pose-trim first, since it directly targets the 0.5 deg
miss without pretending the torque proxy is current.
