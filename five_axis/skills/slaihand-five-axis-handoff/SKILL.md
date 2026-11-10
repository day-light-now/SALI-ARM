---
name: slaihand-five-axis-handoff
description: Resume the SLAIHand five-axis robot work from the 2026-11-09 handoff: current repo/commit, J5 output-load geometry, no-power-cycle TIMEOUT verification, serial/audit fixes, acquired data, and next gravity-compensation steps.
---

# SLAIHand five-axis handoff skill

Use this skill when resuming the five-axis SLIHand robot work after context
handoff, or before changing its controller, gravity model, admission path, or
trajectory execution.

## First action

Read the full handoff:

```text
D:/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/HANDOFF_20261109.md
```

Do not act on older four-axis assumptions without reconciling them with that
document.

## Current hard facts

- Worktree: `slaihand_five_axis_ws_gain`, branch
  `feature/five-axis-gain-test-20261108`, HEAD `5cebc3f`.
- Main worktree `slaihand_five_axis_ws` is dirty; do not overwrite it.
- All drivers are stopped at handoff. Check again before any command:
  `pgrep -af ros2_control_node` and `fuser -v /dev/ttyACM0`.
- Five-axis machine.
- J5 motor body is fixed to J4 link.
- The 3.5 kg hand-motor box is mounted on the J5 output and rotates with J5.
- J5 is therefore a real gravity joint; the old fixed-box-on-link4 model and its
  COM candidate are invalid.
- The existing load profile still says gravity FF is not allowed; the driver
  still rejects five-axis gravity FF.
- `effort` is a torque estimate, not measured current.
- No per-axis current sensor; no thermal/1.5× evidence.

## Do not

- do not disable the J5 TIMEOUT watchdog or set it to 0;
- do not bypass artifact/evidence checks;
- do not reuse old four-axis gravity/model assumptions;
- do not deploy the Codex fixed-box candidate;
- do not enable full gravity FF before J5-aware held-out validation;
- do not lower 4°/STOP/limit/freshness guards;
- do not use `effort` or request estimates as measured current.

## Current infrastructure

- Readback-only admission avoids a power cycle when TIMEOUT is already 2000:
  `j5_timeout_setup.py --verify-existing` (no parameter write).
- Serial/audit stability fixes: 20 ms serial write deadline, 8192 audit queue,
  256 KiB batched audit writes.
- Four GC baseline trajectories completed after the fix; all final absolute-zero
  windows passed, intermediate errors remain.
- J5 excitation data (`J5_EXCITATION_ZERO.json`) now exists and passed 6/6
  windows.

## Next work order

1. Correct the five-axis gravity model for the J5 output load.
2. Re-fit with J5 excitation and existing poses; do not reuse fixed-link4 COM.
3. Validate on held-out poses.
4. Only then implement bounded five-axis FF (shadow/alpha 0, then small scale).
5. Keep limits, slew, STOP-zero, and pose-domain checks.
6. Add the box visualization later only if requested.

## Latest evidence (2026-11-09 evening, read this before new motion)

- Full handoff update: `HANDOFF_20261109.md` section 11.
- J5 excitation data does NOT identify a J5 output-load gravity term: upstream
  J1-J3 torque stays flat while q5 sweeps +/-20 deg, the J5 effort curve is not
  a gravity sinusoid, and a stiction slip releases ~0.75/0.86 Nm on J1/J4.
  A payload rotating with J5 at the old fixed-box COM would swing J2/J4 by
  2.21 Nm; observed <=0.02 Nm. Evidence:
  `verification/hardware/five_axis_20261107/five_axis_j5_model_20261109/README_J5激励数据复核_20261109.md`.
- A parallel 73-window five-DOF energy model found the same 1.60 Nm J5 approach
  dependence and also selected no deployable parameters:
  `verification/hardware/five_axis_20261107/five_dof_identification_20261109/README_五轴模型修正.md`.
- Next data must be paired-approach static poses (GC05-GC08 plan library in
  `gravity_sampling_v2_20261109/`, imported into the panel) plus independent
  J5 axis/pivot geometry. Do not enable FF before paired-data refit and
  held-out validation below the friction floor.
- GC05-GC08 were EXECUTED on the real arm on 2026-11-09 evening (no FF, all
  guards unchanged). 4/4 trajectories reached goal; GC06-GC08 carry the same
  client `WAYPOINT_FEEDBACK_CRITERION_NOT_MET` as GC01-GC04 because of static
  gravity sag, not a safety fault. 42 static windows were extracted; J5
  up/down friction half-differences are 0.49-1.01 Nm and the paired-mean J5
  torque is ~0 at the zero base pose and ~-0.15 Nm at shifted poses. Record:
  `verification/hardware/five_axis_20261107/gravity_sampling_v2_20261109/execution_20261109/README_GC05-08采集执行_20261109.md`.
  Next: refit the 73 old + 42 new windows with approach-direction nuisance
  terms (GC05-GC07 train, GC08 holdout); no FF until that passes.
- GC09-GC12 (J1/J4/J2/J3 direction+repeatability) were collected on 2026-11-09.
  Per-plan pair half-differences 0.64-1.10 Nm and pair means 2.22/1.05/2.35/
  -0.08 Nm for J1/J4/J2/J3; within-session repeatability <=0.013 Nm, but a
  0.15 Nm session-to-session drift was seen on GC11. GC13 is NOT collected yet
  (frozen until the model/thresholds are locked).
- Two hardware incidents that session: (a) a `SERIAL_OR_AUDIT_WRITE_FAILURE`
  aborted the first GC11 attempt (safe STOP, clean retry succeeded); (b) the
  driver process crashed after an enable/ramp on the first GC12 attempt with
  the motors still enabled, so the operator cut power; after re-power the
  TIMEOUT had reverted to 0 and a write-once setup + fresh artifact were used.
  GC12 retried successfully; the crash root cause is NOT determined. Before
  routine motion, diagnose the control-loop throw/audit/serial paths and the
  USB link, run with unbuffered logs + core dumps, and re-verify admission.
  Full report:
  `verification/hardware/five_axis_20261107/five_dof_refit_gc05_08_20261109/execution_20261109/README_GC09-13采集与GC12事故_20261109.md`
- 2026-11-10: **static position-feedforward gravity-sag compensation validated**
  without changing any FF gate. Auto-collected 20 training static windows,
  fitted a per-joint sag model, then compensated waypoint targets on 8 unseen
  mixed poses: steady-state residual vs desired RMS 0.550 -> 0.205 deg
  (max 1.531 -> 0.443 deg); J1/J2/J4 max residual reduced 68-76%, J5 needs
  dedicated data (back-lash/hysteresis, sign can flip). Torque FF alpha stays 0
  because candidate admission still blocks feedforward for the provisional load;
  this is position compensation only. Evidence:
  `verification/hardware/five_axis_20261107/gravity_ff_20261110/README_静态重力补偿_20261110.md`
- 2026-11-10 J5 follow-up: J5 static error is direction-dependent **stiction**
  (~0.28-0.38 deg, sign flips with motion direction), not a gravity term.
  Direction-aware J5 model trained on fixed-base bidirectional sweeps; novel-pose
  validation: overall residual RMS 0.689 -> **0.171 deg**, max 1.028 -> 0.361,
  J5 0.382 -> 0.266 (-30%), J1/J2/J4 -59..-72%. `SERIAL_OR_AUDIT_WRITE_FAILURE`
  interrupted two runs (safe STOP, fresh disable); a usbipd detach/attach reset
  the link and the full run completed. Plan-level position compensation only;
  FF alpha 0, GC13 still frozen. Evidence: same `gravity_ff_20261110/` directory
  (`j5_sag_model_20261110.json`, `j5_compensation_result_20261110.json`).
- 2026-11-10 iterative static compensation: per-pose measure-correct in two
  passes on a never-trained new pose reduced overall residual RMS 0.632 ->
  **0.0775 deg** (max 1.017 -> 0.164 deg); J3 -46%, J5 -54%, J1/J2/J4 93-98%.
  One baseline run per new pose/path is needed before generating the
  compensated plan; plans keep absolute-zero endpoints; position feedforward
  only (no torque FF), no gate change, FF alpha 0, GC13 frozen.
- 2026-11-10 one-click static compensation: drag a plan JSON onto
  `run_compensation.cmd` (dry-run variant `run_compensation_dry.cmd`) in
  `gravity_ff_20261110/`; it runs admission -> artifact -> driver -> baseline ->
  two measured-residual iterations -> report and asks the operator to type RUN.
  Dry-run passed; full real smoke was blocked by the recurring
  `SERIAL_OR_AUDIT_WRITE_FAILURE` (wrapper aborted safely each time). Retry only
  after checking the USB link/power; FF gates unchanged. Usage:
  `gravity_ff_20261110/README_一键补偿.md`.
- 2026-11-10 J5 coupling test (box confirmed on J5 flange): J5 +/-30 deg at two
  tilted poses gives J2 effort amplitude only 0.0055/0.015 Nm (code 0.098 Nm);
  J1/J3 apparent amplitudes are stick-slip steps, not gravity; J5 effort is
  direction-dependent friction. => box COM nearly on J5 axis, J5 gravity term
  <~0.1 Nm, so do NOT calibrate the J5 axis for gravity FF. **Effort absolute
  calibration remains the hard blocker** (no known weights/force gauge on site),
  so torque FF and free-drag cannot be certified; FF alpha 0, admission
  unchanged. Cheap calibration path: known-volume water bottles as known masses
  on a measured lever. Details:
  `gravity_ff_20261110/README_J5耦合标定_20261110.md`.
- 2026-11-10 friction sweeps F1-F5 (single-joint bidirectional slow moves):
  sliding friction J1 1.09, J2 0.98, J3 0.55, J4 0.92, J5 0.94 Nm. Adding
  q-dependent friction columns to the no-prior gravity fit did NOT improve
  cross-validation (0.656 -> 0.659 Nm; inconsistent scales), and pose design
  showed more static data gives only 0.661 -> 0.657 Nm. Static gravity model
  floor is ~0.65 Nm under the current measurement chain. Torque FF/free-drag
  stay blocked until mechanical friction reduction (planned motor-box change)
  and/or a known torque/force reference; position-feedforward compensation
  remains the usable capability. Details:
  `gravity_ff_20261110/README_摩擦辨识_20261110.md`.
- Offline model-compensated plan converter (2026-11-10 evening):
  `gravity_ff_20261110/compensate_plan.py` turns an in-domain plan into
  `command = desired - predicted_sag` using `sag_model_20261110.json`, with
  training-domain/sag-envelope/limit/endpoint guards; Windows offline entry
  `make_comp_plan.cmd` (no hardware). Offline evidence: model sag prediction
  error on V1 RMS 0.23 deg, measured V1subc residual after model compensation
  0.205 deg (vs per-pose iterative 0.0775 deg); out-of-domain T2/V2/JZ plans
  are rejected on purpose. Position feedforward only; FF alpha 0, GC13 frozen,
  hardware validation after box reinstall pending. Details:
  `gravity_ff_20261110/README_模型补偿_20261110.md`.

## Key evidence locations

- `verification/hardware/five_axis_20261107/acquisition_20261109/`
- `verification/hardware/five_axis_20261107/gravity_compensation_trajectories_20261109/execution_20261109/`
- `verification/hardware/five_axis_20261107/identification_codex_20261109/`

The handoff document is authoritative if this skill and the repository disagree.
