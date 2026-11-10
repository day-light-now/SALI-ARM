# Five-axis unified candidate — Codex review fixes (2026-11-08)

Branch: `feature/five-axis-unified-20261107`  
Base: `b6a1817`; prior C++ `dc57679`, integration `c0e9104`, final `bd7e4ed`
Five-axis optimization/diagnostics candidate: **`4228e35`**  
J5 field execution/evidence: **`4b88197`**  
J5 preparation chain: **`99615f2`**  
J5 adopted-zero config/provenance: **`65c930b`**  
Round-two review: **`d745f4d`** (`五轴第二轮复验_0d958f9_20261108.md`)
Prior review fixes: **`286aa3e`** (`五轴复验_bd7e4ed_20261108.md`)

Earlier PTY-only evidence remains simulation. On 2026-11-08 the operator directed
DSH to execute the real J5 path: the old read-only owner was stopped, the
exclusive TIMEOUT tool and a real candidate trial were run on domain183. The
first trial result is recorded separately below; domain0 and the other CAN
adapter were not used.

## J5 zero/sign adoption (2026-11-08)

The user instructed that the previously recorded J5 zero and manual positive
direction be adopted. Codex updated only
`src/slaihand_bringup/config/hardware_mapping.yaml` and added
`j5_zero_direction_adopted_20261108.json`:

- `hardware_sign: +1`
- `zero_offset_rad: 2.5389105058365757` = 145.46885654586075°
- historical manual positive move: +23.51806620739934° from the zero pose
- two source snapshots with SHA256 provenance
- no motor zero write, no motor command, `formal_full_calibration: false`
- `limits_rad: [-π/2, +π/2]` = ±90° model-frame relative to the adopted zero
- TIMEOUT was later set to 2000 by the exclusive tool and read back; first
  +10°/return trial completed (see execution record)
- read-only page relative J5 angle was approximately -1.246°

The user also supplied ±90° travel and authorized J5 motion. The limit is
imported into `hardware_mapping.yaml`. On 2026-11-08 the operator directed DSH to
stop the old read-only owner and execute the one-shot path. The resulting TIMEOUT
evidence reads back **2000**; the candidate started on domain183 with
`j5_motion_qualified=true` and the first J5 +10°/return trial completed.

See `j5_first_motion_execution_20261108.json` and the evidence/artifact files.
This is adopted historical data plus an operator limit and a first finite trial,
not a formal calibration.

## J5 motion preparation chain (executed 2026-11-08)

The source files are the committed candidate chain; the real evidence is now
present and the mapping TIMEOUT is 2000. TIMEOUT is a RAM parameter (not flash),
so it must be rechecked after a power cycle.

- `scripts/j5_timeout_setup.py`: one-shot bounded setup tool. Exclusive serial,
  all J1–J5 fresh `motor_state=0` preflight, J5 ID5/master0/mode1/profile check,
  exactly one official RID9 uint32 write (`05 00 55 09 d0 07 00 00`), bounded
  readback 2000, no flash/ID/zero/enable/MIT, no retry. PTY tests: 25.
- `config/load_3p5kg_provisional.yaml`: position-only identity declaration;
  mass 3.5 kg is a user estimate, geometry/COM/mount unknown. Gravity status
  PROVISIONAL, `formal_gravity_allowed`/`gravity_fit_allowed`/`float_enabled`
  false, alpha 0. The old `no_end_load` id is explicitly rejected.
- `scripts/j5_motion_artifact.py` + `slaihand_canusb_five_axis_candidate.launch.py`:
  derive the actual load-declaration SHA256, bind mapping/model/policy/load and
  timeout-evidence hashes, verify J5 sign/zero/limits/profile, freshness/device,
  and accept only a canonical artifact SHA. A raw 64-hex value is never accepted.
  Without artifact the candidate starts J5-unqualified feedback-only.
- `j5_first_motion_10deg.json`: relative-start plan; only J5 +10° then back to
  0°, all J1–J4 relative 0°, move 2 s, bounded. Executed once on 2026-11-08
  with the protected client; result `finite_trial_complete`, JTC error 0, all
  three waypoints passed, final five-axis feedback disabled/state0.

## Five-axis optimization and 1.5× margin (offline candidate)

See `CODEX_WIDE_FOLLOWING_ANALYSIS_20261108.md` and
`DSH_FIVE_AXIS_OPTIMIZATION_1P5_20261108.md` for the reviewed correction: the
first failure is driver-layer J2 exceeding the unchanged 4° guard, not J1
exceeding the later JTC 10° error. The optimization candidate adds:

- optional five-axis `kp_axis`/`kd_axis` lists in the policy path, scalar
  fallback 160/5, caps unchanged, per-axis values used in ramp/HOLD/FF
  request-estimate/final MIT; runtime identity exposes applied arrays/source;
- frozen driver-first `first_fault` snapshot before STOPPING with axis, error,
  target/actual, age, encoded Kp/Kd/tau_ff, command/cycle/boot/last complete
  write; post-stop request estimates are `LAST_ACTIVE_STALE`, never current;
- `/api/diagnostics` and a panel card showing driver 4° first cause versus JTC
  10° later error; display-only 4° JTC restoration recommendation (not deployed);
- `/api/margin` and a five-axis margin table with PASS only for verified
  allowed/demand ≥1.5 and matching duration/condition. Without measured
  capability/current/bus data every axis is `UNKNOWN`/`NOT_QUALIFIED`, which
  grants no tuning or amplitude extension; TMAX 200/54/28 is never treated as
  capability;
- ten local single-axis ±2° diagnostic plans under `five_axis_2deg_diagnostics/`
  (8 s moves, 3–5 s dwells, relative start, not auto-run).

Important: **the standard arm is not J5-only.** The protected client calls
`arm_supported_current_pose`; the driver enables all five active axes (J1–J5),
then ramps. J1–J4 are therefore enabled and hold their current positions while
J5 moves. There is no J5-only arm service.

Exact timeout/evidence/admission commands are in
`verification/hardware/five_axis_20261107/j5_timeout_setup_README.md` and
`verification/hardware/five_axis_20261107/J5运动准入与3.5kg暂定负载_使用说明.md`.
The executed sequence was: stop read-only owner -> exclusive timeout setup and
readback -> mapping TIMEOUT 2000 -> rebuild -> real artifact build/validate ->
candidate launch on domain183 -> first protected trial. No deployment and no
gravity/FF was enabled.

## Gravity-identification trajectory candidates

`gravity_trajectories/` contains fifteen offline `slaihand_joint_plan/2` static-pose
excitation groups for gravity/load identification:

- five early J1–J4 groups: `GRAV_A_J1_J2`, `GRAV_B_J2_J3`, `GRAV_C_J3_J4`,
  `GRAV_D_J1_J4`, `GRAV_E_MIXED` (J5 fixed at relative 0°)
- four moderate J5-inclusive groups: `J5_GRAV_A` (+10°), `J5_GRAV_B` (±10°),
  `J5_GRAV_C` (+15°), `J5_GRAV_D` (−15°)
- six wide J5-inclusive groups: `WIDE_J5_60` (±60°), `WIDE_J5_80` (±80°),
  `WIDE_J1_J2_J5`, `WIDE_J3_J4_J5`, `WIDE_MIXED_GRID`, `WIDE_J5_COMBINED`
- all use 6 s static dwells and >0.5 s moves below the 0.5 rad/s reference;
  wide groups stay within the real J5 ±90° limit
- not executed: the only finite field trial was the separate J5 +10°/return plan
- these are static sampling candidates, not gravity feedforward/float, and not a
  formal gravity fit; geometry/COM/mass are still provisional.

## Round-two counterexamples fixed

1. **Preview identity freeze** — a five-axis preview ticket records the full
   runtime identity fingerprint. `execute` rejects if boot/policy/mapping/model
   changed; `_launch` passes `--expected-identity-sha256`, and the protected
   client re-checks it immediately before arm. Failure does not launch or arm.
   Four-axis legacy preview behavior is unchanged.
2. **Actual motor disabled state** — calibration start/capture/each commit now
   require every controlled axis to have finite `motor_state == 0`, not merely
   host `stage=0`; missing/NaN/1/fault state rejects and invalidates the sequence.
3. **Atomic calibration commit** — feedback callbacks and the final commit now
   share `_calibration_commit_lock`; `_state_revision` increases on every
   joint/dynamic/identity store. Sessions carry a revision and terminal state.
   The final commit atomically rechecks active state, revision, expected phase,
   stop epoch, fresh identity, DISABLED feedback and an unchanged state revision.
   A concurrent abort/stop/identity change or a `motor_state=1` feedback update
   makes a late upper reading fail with `CAL_INVALIDATED`/`AXIS_FEEDBACK`; no
   `aborted → capture → complete` transition is possible. Regression tests use
   an actual feedback-update thread and verify a callback blocks on the commit
   lock rather than updating underneath it.
4. **Five-axis float config in the client** — `float_settings_confirmed` now
   reuses the production FF parser instead of row 21/22/23. Legal 43-field
   five-axis flag/Kd/seconds at 25/26/27 is accepted; four-axis 21/22/23 remains
   compatible; wrong axis/protocol is rejected.
5. **Strict explicit FF metadata** — explicit `schema=.../banana`, explicit
   float dimension size 2, or explicit axes≠current joint count is rejected with
   `FF_SCHEMA`/`FF_PROTOCOL_LAYOUT`/`FF_AXIS_MISMATCH`. Only messages with no
   metadata may use length fallback. A legacy four-axis payload in a five-axis
   panel is retained only as `diagnostic_only` and is never control-connected.

Repro evidence: `repro_round2_fixed.json` (all five original cases now
rejected/fixed, no ROS/real port).

## Review fixes

### P1 — real five-axis protected client, hold, teaching/replay

- `commission_supported_trial.py` now has explicit `--joint-count {4,5}` (default
  4 for old callers). Five-axis requires explicit `--j5-limits-deg LOWER UPPER`;
  J1–J4 still come only from canonical `slaihand_four_axis.urdf`; J5 is never
  inferred. Joint names and every plan/hold/JTC/result array are dynamic; wrong
  dimensions are refused, not padded.
- `drag_teaching.py` has `configure_axes(4|5)`: 4 stays `/1`, 5 switches
  capture/teaching/playback to `/2`; all q/dq/age/motor/limits widths follow it.
  A `/1` record is rejected in five-axis mode and vice versa.
- Panel calls `configure_axes(self.joint_count)` once, adds
  `--joint-count`/`--j5-limits-deg` to every protected-client launch, and detects
  five-axis client capability from the explicit CLI contract (not a literal
  joint name). The old `CLIENT_AXES` gate is retained for genuinely old clients;
  it is no longer permanently true.
- **End-to-end PTY/JTC positive path PASS**: one fake PTY owner, qualified J5
  simulation (`calibration_qualified=true`, finite ±30°, 2000 watchdog,
  simulated artifact hash), real installed protected client, standard JTC:
  - five-axis hold → `finite_trial_complete`
  - five-axis planned execution → J1–J4 ≈1.0°, J5 ≈1.99° excursion
  - five-axis teaching/replay `/2` → J1–J5 tracking, final errors <0.02°
  - J5 FC/FD/MIT observed, no physical port.
  Evidence: `five_axis_client_jtc_result.json`.

### P2 — `gravity_ff_state` five-axis protocol

- C++ publisher now labels `layout.dim`: `slaihand_gravity_ff_state/2`,
  `size=axis_count`, second dim `float=0/1`.
- `control_panel_model.parse_gravity_ff_state` resolves explicit metadata and
  falls back by length; it accepts 4-axis 21/22/24/34/37 and 5-axis 40/43,
  exposes exactly N requested/encoded/applied/last-written + N hold target/error,
  and returns a stable error for every wrong dimension/version/axis mismatch.
- `_ff_callback` stores one state or one explicit rejection per message; nothing
  is silently discarded. The four-axis gravity shadow gets only its four-axis
  slice and is labelled diagnostic.
- Repro `37→1, 40→1, 43→1`; production dim-label path and wrong-shape cases are
  pinned by `test_five_axis_ff_callback.py` (13 tests).

### P2 — unified per-axis calibration workflow

- New UI card in the existing `示教 / 采样` workspace: select J1–J5, then
  `零位读数 → 正方向移动 → 下限端点 → 上限端点`, with two explicit support/
  clearance acknowledgements and a visible “候选，非正式” result.
- Backend/session enforces: same identity/boot/mapping/model/policy, same
  selected joint, fresh valid feedback, all controlled axes DISABLED, stop epoch
  stable, support acknowledgement every capture, strict phase order, minimum
  1° positive move and 90° single-turn guard, finite ordered limits.
- Stop/UNKNOWN/identity change/feedback loss/panel close/abort invalidates the
  sequence. Every transition is written to non-formal
  `<archive>/axis_calibration/<session>.json` with
  `formal:false, motor_command_sent:false`; no motor zero, mapping or URDF write.
- Routes: `GET /api/calibration/axis`, `POST /api/calibration/axis/{start,
  capture,abort}` under the existing token/host/origin/CSP surface;
  `/api/status` exposes `axis_calibration`. 27 workflow/http/UI tests.

## Communication / UI retained

- Five sticky workspaces, J1–J5 overview cards, single collapsible detail table,
  one J5 qualification row, panel latency chip, single global stop.
- One read request per control cycle (round-robin J1–J5); the 5 ms write
  deadline, 3-EAGAIN exit, 250 ms feedback gate and all-axis stop are unchanged.
- Tests use isolated ROS domains; stale `robot_state_publisher` publishers from
  old launches must not feed an old transient-local `robot_description` to a new
  controller manager.

## Reproduce

```bash
# build (persistent cache path avoids WSL /tmp loss)
cd /mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws
source /opt/ros/jazzy/setup.bash
colcon build --build-base /home/xuyu/.cache/dsh_five/build \
  --install-base /home/xuyu/.cache/dsh_five/install \
  --packages-up-to slaihand_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source /home/xuyu/.cache/dsh_five/install/setup.bash

colcon test --build-base /home/xuyu/.cache/dsh_five/build \
  --install-base /home/xuyu/.cache/dsh_five/install \
  --packages-select slaihand_transport slaihand_hardware \
  --event-handlers console_direct+ --return-code-on-test-failure

python3 -m unittest \
  verification.hardware.five_axis_20261107.test_five_axis_plan \
  verification.hardware.five_axis_20261107.test_five_axis_panel_http \
  verification.hardware.five_axis_20261107.test_five_axis_panel_backend \
  verification.hardware.five_axis_20261107.test_five_axis_ff_callback \
  verification.hardware.five_axis_20261107.test_five_axis_calibration \
  verification.hardware.five_axis_20261107.test_five_axis_client_teaching \
  verification.hardware.five_axis_20261107.test_five_axis_client_float \
  verification.hardware.five_axis_20261107.test_gravity_trajectory_library \
  verification.hardware.five_axis_20261107.test_j5_timeout_setup \
  verification.hardware.five_axis_20261107.test_j5_motion_admission \
  verification.hardware.five_axis_20261107.test_j5_first_motion_plan \
  verification.hardware.five_axis_20261107.test_five_axis_2deg_diagnostics \
  verification.hardware.five_axis_20261107.test_five_axis_gain_fault \
  verification.hardware.five_axis_20261107.test_five_axis_margin

# round-two independent counterexamples (no ROS/real port)
python3 verification/hardware/five_axis_20261107/repro_round2_fixed.py

# five-axis client -> real driver -> JTC; generated PTY, simulated qualification
python3 verification/hardware/five_axis_20261107/test_five_axis_client_jtc_pty.py

# unqualified driver feedback/stop gate
python3 verification/hardware/five_axis_20261107/test_five_axis_driver_pty.py

# browser UI (fake transport-only backend)
python3 verification/hardware/five_axis_20261107/five_axis_browser_fixture.py  # WSL
python  verification/hardware/five_axis_20261107/five_axis_browser.py          # Windows Chrome
```

## Results

- Python/web: **261 tests OK** across the expanded offline/API suites,
  including 35 gain/fault and 41 margin tests plus the ten 2° plans.
- C++: **1/1 transport + 7/7 hardware** tests passed, including per-axis gain
  encode and frozen J2-first-cause/stale-estimate cases.
- Offline PTY: driver feedback/stop and five-axis client/JTC PASS on
  `ROS_DOMAIN_ID=180`; this remains generated-PTY evidence, not hardware.
- Legacy planner: **14 tests OK**.
- C++ `colcon test`: **7/7 passed**.
- Five-axis client/JTC PTY: **PASS** hold, execute, teaching/replay; 5 axes.
- Unqualified driver PTY: **PASS** J5 feedback/TIMEOUT0, arm refused, 5×FD stop,
  FF layout `axes=5/float=0` with 40 fields.
- Browser DOM: **PASS** five tabs, five rows/cards, calibration card, no J5 card.
- **Real field first trial (2026-11-08)**: timeout 0→2000 readback; real artifact
  admitted; candidate domain183 `j5_motion_qualified=true`; protected client
  executed J5 +10° then returned, JTC error 0, all three waypoints passed, final
  feedback all `motor_state=0`/`timeout=2000`/disarmed. See
  `j5_first_motion_execution_20261108.json` and
  `j5_first_motion_20261108_142736.json`.
- **Trajectory tolerance (operator request 2026-11-08)**: after J1
  following-error aborts at 4.097°, then 8.81°/9.08° vs 8°, the five-axis JTC
  `trajectory` tolerance was widened to **10°** for J1–J5; the 1° goal tolerance
  and all stop/feedback limits are unchanged. This is a following-error abort
  limit, not a torque limit. A slower WIDE plan is still preferable for data
  quality.
- Evidence: `five_axis_client_jtc_result.json`, `five_axis_driver_result.json`,
  `evidence/five_axis_single_table.png`, `evidence/five_axis_calibration_workflow.png`,
  `evidence/j5_timeout_setup_evidence.json`, `j5_motion_calibration_artifact.json`.

## Real measurement gaps (unchanged; simulation does not fill these)

1. J5 zero/sign are now the user-adopted historical values above; this is not a
   fresh formal calibration session and no motor zero was written.
2. J5 ±90° travel is imported into the mapping; the real candidate used the
   same interval and passed the first +10° trial.
3. J5 TIMEOUT is now **2000** only as a RAM parameter, read back on 2026-11-08;
   it is not saved to Flash and may revert after a power cycle. The artifact is
   session-bound and expires (30 min). MIT mode=1/PMAX/VMAX/TMAX are observed;
   DM-J4340-2EC remains project-declared. `mass_profile_sha256` is now the actual
   provisional declaration file hash, but the mass/geometry/COM are still not
   measured and formal gravity/FF remains forbidden.
4. J5 geometry/flange/mount/COM and 3.5 kg assembly mass/COM.
5. Real USB/WSL link acceptance is not proven by one first trial; the first
   +10° trial is a limited field observation, not a reliability acceptance.
6. Five-axis static gravity capture/calibration groups stay 4-axis legacy and are
   explicitly refused by the five-axis client; this review’s calibration workflow
   is manual per-axis, non-formal, and does not perform formal gravity fitting.
7. Per-axis Kp/Kd is now a supported candidate path, but the deployed policy
   still uses scalar 160/5 and no gain value has been physically tuned. The
   driver 4° guard and JTC 10° path remain intentionally different; restoring
   JTC 4° is only a display recommendation and was **not deployed**.
8. Five-axis 1.5× margin is `UNKNOWN/NOT_QUALIFIED`: no verified per-axis
   allowed output, measured current/bus/torque demand, or matching operating
   condition exists. TMAX 200/54/28 is protocol range, not capability. No
   margin PASS, no tuning grant, no amplitude extension, and no physical
   optimization claim is made.

## Boundary

The first J5 +10°/return trial is a real, limited field result, not a formal
calibration, position-accuracy acceptance, or gravity-compensation qualification.
The production launch default remains `j5_qualified=false` unless a validated
artifact is supplied. TIMEOUT2000 is RAM-only. Do not reuse this result after a
power cycle or after changing mapping/model/policy/load/limits without new
evidence and a new artifact.
