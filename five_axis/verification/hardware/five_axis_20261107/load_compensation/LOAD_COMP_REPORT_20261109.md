# Path-local load proxy (2026-11-09)

This is **not** a gravity model, not measured torque, and not deployable FF.

## Source

Corrected 0–40 synchronized baseline (`corrected_sweep160_summary_20261109.json`),
window rule from the client target list, whole-window reference span, feedback
speed limit.

`pd_request_estimate = -Kp*(feedback-reference)` with Kp=160 for every axis.

## Quadratic fits (relative path angle, degrees)

| Axis | a0 | a1 | a2 | validity | LOO max error |
|---|---:|---:|---:|---|---:|
| J1 | 0.142 | 0.558 | -0.0079 | 0.11–35.11° | 0.381 Nm |
| J2 | 0.423 | 0.392 | -0.0039 | 0.11–35.11° | 1.038 Nm |
| J3 | 0.278 | 0.078 | -0.0023 | 0.11–35.11° | 0.818 Nm |
| J4 | 0.277 | 0.268 | -0.0023 | 0.11–35.11° | 0.477 Nm |
| J5 | 0.409 | 0.039 | -0.0014 | 0.11–35.11° | 0.849 Nm |

## What this does and does not mean

- It is a **path-local PD request proxy**, not measured load.
- It is fit on one synchronized path; it cannot describe multi-axis coupling.
- J2/J3/J5 variation is not independent enough for a five-axis gravity model.
- J2's LOO error is above 1 Nm; the shape is not reliable for control.
- No coefficients are written into any driver/policy, and no FF is enabled.
- No current/bus/thermal data, so no saturation or 1.5× margin conclusion.

## Next

1. Add real bus voltage/current measurement and synchronize it with position.
2. Collect independent single-axis and mixed poses.
3. Fit a model from signed, multi-pose, current-aware data.
4. Only then implement bounded FF with limits, slew, validity and STOP cleanup,
   and first validate it in software/PTY.

Files: `fit_static_load.py`, `test_fit_static_load.py`, `path_load_proxy_20261109.json`.
