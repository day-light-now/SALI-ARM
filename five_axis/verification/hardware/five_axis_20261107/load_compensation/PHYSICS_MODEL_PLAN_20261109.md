# Physics gravity model plan (2026-11-09)

## What the URDF provides

- `slaihand_four_axis.urdf` has **visual meshes only** — no `<inertial>`, mass,
  COM, or inertia tensors.
- **J5 is not a kinematic joint in the physical model.** It is a fixed mass (the
  end payload/tool) carried by J4; only J1–J4 contribute gravity degrees of
  freedom. J5's own torque estimate must not be interpreted as a gravity-joint
  torque.
- Therefore a CAD-derived gravity model is not available; J5 must be modeled as
  a fixed payload mass/COM on the J4 chain.

## Offline feasibility result

Tried a four-point-mass model at the J1-J4 link origins using the 14 multi-pose
points, with J5 excluded as a joint. The normal-equation system is
**singular/rank deficient**:

```json
{
  "sample_count": 14,
  "singular": true,
  "error": "singular",
  "deployable": false
}
```

So the link/payload parameters cannot be identified from the existing data with
this reduced model. Adding an independent J5 payload parameter alone does not
remove the rank deficiency because the payload acts at the J4 end.


## Required identification data

- 30–40 independent safe poses, not a single path;
- mixed single-axis and multi-axis poses;
- cover J1–J4 ranges while avoiding the J3 upper-limit region and rear/mechanical
  obstructions;
- slow moves, static dwell, per-joint `effort` torque estimate, position/velocity,
  state/validity;
- no current sensor is required for the torque-estimate version, but the result
  remains an **estimated-torque model**, not measured current.

Suggested session layout:

1. Session A: J1/J2 excitation, J3/J4 fixed; ~10 poses.
2. Session B: J3/J4 excitation, J1/J2 fixed; ~10 poses.
3. Session C: mixed independent poses; ~10–15 poses.
4. Session D: held-out validation poses; do not reuse for fitting.

Each session stays under the client/session limit and ends with the arm in a
safe supported pose.

## Model and validation

- Fit a **full base-parameter gravity regressor** (or at minimum a constrained
  point-mass/first-moment model) from the torque estimates.
- Check rank/condition before fitting; reject underdetermined datasets.
- Leave-one-session-out validation, not only same-path LOO.
- Do not enable FF unless held-out error is materially better than the current
  static error and limits/slew/STOP behavior are defined.
- Keep all current 4°/STOP/timing guards unchanged.

## Admission blocker

The current artifact/evidence set expired while this offline work was done.
A new capture requires a **J5 power cycle** and fresh TIMEOUT -> evidence ->
artifact chain. No driver/panel/motion is running now.
