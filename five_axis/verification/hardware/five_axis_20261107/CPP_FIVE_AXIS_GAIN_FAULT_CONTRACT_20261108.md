# C++ five-axis per-axis gains + first-fault diagnostics (candidate)

Date: 2026-11-08. Owner: `cpp-gain-diagnostics` (C++ side only).
Worktree: `D:/CodexData/Projects/机械臂/slaihand_five_axis_ws`,
branch `feature/five-axis-unified-20261107`, HEAD `5941530` when this work was
done (its parent is `9310b19`; nothing was committed or reset).

Status: **offline candidate implementation only**. No real serial/CAN/USB, no
ROS domain 183/0, no arm/enable/motion/field process, no physical
qualification claim, no gain change deployed. The adopted policy file is left
unchanged (SHA256 still
`f8ae7e83a2f898aad14096f66b0cb90238bb507428e4373b3c8c3810f08897e4`, still
160/5 and 4 deg) and the four-axis behaviour/config is unchanged.

## 1. Files

| File | Change |
|---|---|
| `src/slaihand_hardware/include/slaihand_hardware/motion_supervisor.hpp` | `MotionPolicy::kp_axis/kd_axis`, `parse_gain_axis_parameter`, `FaultSnapshot`, effective gain accessors/JSON, first-fault/stale-estimate accessors, private capture helpers |
| `src/slaihand_hardware/src/motion_supervisor.cpp` | list parsing/validation, per-axis effective gains in ramp/HOLD/gravity-hold lock+resume and packet path, first-fault capture before `stop()`, JSON serialization, request-estimate state |
| `src/slaihand_hardware/include/slaihand_hardware/motion_audit.hpp` | `MotionRecord.boot_id/request_estimate_state/request_estimate_at/first_fault` |
| `src/slaihand_hardware/src/motion_audit.cpp` | STAGE serializes `boot_id`/`first_fault`/`request_estimate_state`; CONTROL_CYCLE serializes `request_estimate_state`/`request_estimate_at` |
| `src/slaihand_hardware/src/canusb_system.cpp` | parses optional `kp_axis`/`kd_axis` hardware params, identity/metadata gain fields, first-fault fields in identity, latched detection cycle, identity republish on stage change |
| `src/slaihand_hardware/include/slaihand_hardware/canusb_system.hpp` | latched first-fault detection cycle members |
| `src/slaihand_description/urdf/slaihand_canusb_five_axis.urdf.xacro` | passes optional `kp_axis`/`kd_axis` lists from the five-axis policy YAML; the four-axis xacro is untouched |
| `src/slaihand_hardware/test/motion_supervisor_test.cpp` | new per-axis gain/validation/ramp/HOLD/encoding/identity tests + frozen J2-first-fault tests |
| `src/slaihand_hardware/test/motion_audit_test.cpp` | audit JSON contract for boot/first-fault/stale-estimate fields |

Not touched: driver 4 deg FOLLOWING_ERROR gate, STOP/DISABLE flow, 5 ms write
deadline, 3-EAGAIN exit, 60 ms feedback age, 50 ms cycle gap, limits/temperature
gates, native codec/transport, `canusb_five_axis_motion_policy.yaml`,
`canusb_five_axis_controllers.yaml`, panel/web/Python code.

## 2. Configuration contract (A: independent Kp/Kd)

Five-axis policy YAML may add two optional lists to
`canusb_five_axis_motion_policy.yaml` (independently optional):

```yaml
kp: 160.0                      # required legacy scalar, kept as fallback
kd: 5.0
kp_axis: [150.0, 145.0, 140.0, 135.0, 130.0]   # J1..J5, exactly five
kd_axis: [4.5, 4.0, 3.5, 3.0, 2.5]
```

- Legacy scalar-only policy files remain valid: empty/absent list = scalar
  `kp`/`kd` applied to every active axis (default 160/5, unchanged).
- Caps are **not raised**: each Kp entry must be finite and in `(0, 160]`, each
  Kd entry finite and in `(0, 5]`. NaN/Inf/wrong length/out-of-range are
  rejected; nothing is clamped or padded.
- Lists are accepted only for a five-axis active set (exactly `active_axes`
  entries). A four-axis session must keep the scalar policy; per-axis lists are
  rejected there.
- The generated five-axis URDF passes the lists as
  `<param name="kp_axis">150.0 145.0 ...</param>` (space-separated); the driver
  parser also accepts comma and `[...]` forms for tests/tools. Empty text means
  "not supplied".
- `policy_sha256` is the launch-computed hash of the policy YAML, so adding or
  changing either list changes the runtime identity's policy hash. Existing
  ticket/identity machinery that binds `policy_sha256` + `boot_id` (preview
  identity freeze, protected-client `--expected-identity-sha256`, drag-teaching
  records) therefore rejects old tickets after a gain-policy change. This
  implementation adds no parallel ticket mechanism.
- Four-axis behaviour: no arrays parsed/passed by the four-axis launch; the
  supervisor still normalizes scalar kp/kd to its effective per-axis vector, so
  packets and gains are bit-for-bit the legacy path (covered by the existing
  four-axis tests, which all still pass).

### Application path

`MotionSupervisor` normalizes `kp_axis_`/`kd_axis_` once (scalar fallback) and
uses these effective values everywhere:

- RAMP/TRACK/HOLD position gain: `kp = ramp_scale * kp_axis[i]`,
  `kd = ramp_scale * kd_axis[i]` (same independent per-axis ramp time).
- Gravity-hold LOCK/RESUME (four-axis gravity feature, unchanged availability):
  `kp = hold_scale * kp_axis[i]`,
  `kd = float_kd + hold_scale * (kd_axis[i] - float_kd)`; release starts from the
  actually reached `hold_gain_scale`.
- Gravity-FF request-estimate/headroom path and the final MIT encode: the
  decoded packet's `kp/kd/q/dq/tau` are used for `request_estimate_nm`, so the
  estimate always matches the per-axis values actually encoded.
- Because five-axis gravity feedforward is refused by an existing policy (the
  only qualified model is four-axis), the LOCK/RESUME path cannot be armed on a
  five-axis session today; it is nevertheless per-axis-consistent for a future
  qualified five-axis model, and the four-axis scalar lock/resume tests still
  pass unchanged.

### Runtime identity fields (A)

`/slaihand/runtime_identity` (std_msgs/String, transient_local reliable,
schema `slaihand_runtime_identity/1`) gains (additive):

| Field | Type | Meaning |
|---|---|---|
| `kp_axis` | `[number]` | Effective per-axis Kp the driver accepted and uses at full ramp; length == `active_axes` (J1..Jn order) |
| `kd_axis` | `[number]` | Same for Kd |
| `kp_axis_source` | `"per_axis" \| "scalar_legacy"` | Whether the list came from the policy or was filled from scalar `kp` |
| `kd_axis_source` | same | Same for Kd |
| `following_error_deg` | number | Current driver FOLLOWING_ERROR threshold (display only; still 4) |

Legacy `kp`/`kd` scalars stay for compatibility but must not be presented as
the applied per-axis value when the arrays are present. Identity is republished
on stage changes and every 2 s; arrays are the full-ramp values, the per-cycle
ramp scale is applied monotonically on top.

The audit SESSION metadata line in `canusb_motion_*.jsonl` carries the same
`kp_axis`/`kd_axis`/`*_source`/`following_error_deg` fields.

## 3. First-fault snapshot contract (B)

Captured in `MotionSupervisor::check_observations` **before**
`stop()`/STOPPING can overwrite stage state, for the first unsafe observation of
a session. Later OPERATOR_GLOBAL_STOP or downstream JTC errors keep the same
frozen object; the global stop still has stop priority and still drives the
normal STOPPING/DISABLED flow. A new `arm()`/`reset()` clears the snapshot only
for the next session.

Captured codes: `FOLLOWING_ERROR` (full detail), plus axis-specific
`FEEDBACK_SPEED`, `TEMPERATURE`, `MOTOR_FAULT`, `UNEXPECTED_AXIS_DISABLED`,
`ACTUAL_OUTSIDE_OPERATING_LIMITS`, `REQUIRE_DISABLED_STATIONARY`,
`REQUIRE_MIT_TIMEOUT2000_IDS`, `NATIVE_PROFILE_MISMATCH`,
`AXIS_CALIBRATION_PENDING`, `FEEDBACK_ESTIMATE_GUARD_NOT_HARD_CLAMP`, and
non-axis-specific `ACTIVE_AXIS_COUNT_MISMATCH`,
`FEEDBACK_STALE_OR_CONFLICTED`, `NONFINITE_CLOCK`, and generic supervision
failures (cycle gap/session/reference errors). Only the first one is kept.

`first_fault` object (null before the first fault; frozen afterwards):

```json
{
  "schema": "slaihand_first_fault/1",
  "layer": "DRIVER",
  "axis": 2,
  "joint": "slaihand_joint2",
  "code": "FOLLOWING_ERROR",
  "detected_at": 15324.007779907,
  "error_rad": 0.0698951, "limit_rad": 0.0698132,
  "error_deg": 4.0047,   "limit_deg": 4.0,
  "target_rad": 0.70162, "actual_rad": 0.63172,
  "target_deg": 40.1998, "actual_deg": 36.1951,
  "feedback_age_s": 0.012, "feedback_valid": true,
  "encoded_kp": [159.9512, 159.9512, 159.9512, 159.9512, 159.9512],
  "encoded_kd": [5.0, 5.0, 5.0, 5.0, 5.0],
  "encoded_tau_ff_nm": [-0.04884, -0.04884, -0.01319, -0.01319, -0.00684],
  "request_estimate_nm": [6.724, 11.117, 0.0, 0.0, 0.0],
  "request_estimate_at": 15324.0075,
  "request_estimate_state": "LAST_ACTIVE",
  "command_id": 14, "cycle_id": 253166,
  "boot_id": "<driver boot id>", "last_complete_write_at": 15324.0075,
  "frozen": true
}
```

Semantics:
- `error_rad/limit_rad` are the observed gate error and the configured gate
  (`following_error_deg`, still 4 deg). Values may be null for non-numeric
  faults.
- `target_rad` is the active commanded target only for output stages; null is
  possible for preflight/non-axis faults. `actual_rad` is the feedback mapped
  into the model frame; `feedback_age_s/feedback_valid` use the same 60 ms age
  rule as the gate.
- `encoded_kp/kd/tau_ff` are decoded from the last MIT packets that actually
  reached the serial write path for every axis (host write receipt, not a drive
  execution acknowledgement). `last_complete_write_at` is the steady-clock time
  of the last batch that wrote every active axis (`null` if none yet).
- `request_estimate_nm` is the last completed controls() estimate (pre-fault);
  it is last-active evidence, not a current powered-output claim. Its internal
  state is `LAST_ACTIVE`.
- `cycle_id` is the driver read-cycle counter of the detection cycle (latched,
  not the republish cycle); `boot_id` binds it to the driver session.

`request_estimate_state` (top level of runtime identity and in audit records):

- `UNAVAILABLE` - no controls() output yet in this session;
- `CURRENT` - the retained `request_estimate_nm` belongs to the current output
  cycle (only PRELOAD/ENABLING/RAMP/HOLD/TRACK);
- `LAST_ACTIVE_STALE` - after STOPPING/DISABLED/UNKNOWN/DISARMED; the retained
  numbers must be displayed as "last active estimate", never as current output.

### Audit JSONL

- STAGE records add `"boot_id"`, `"first_fault"` (object or `null`) and
  `"request_estimate_state"`.
- CONTROL_CYCLE records keep `"request_estimate_nm"` and add
  `"request_estimate_state"` and `"request_estimate_at"` (steady seconds or
  null). After a stop the array remains from the last active cycle, but the
  state is `LAST_ACTIVE_STALE`.

## 4. Tests / build (source only)

```bash
cd "/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws"
source /opt/ros/jazzy/setup.bash
colcon build --build-base /home/xuyu/.cache/dsh_five/build \
  --install-base /home/xuyu/.cache/dsh_five/install \
  --packages-up-to slaihand_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --build-base /home/xuyu/.cache/dsh_five/build \
  --install-base /home/xuyu/.cache/dsh_five/install \
  --packages-select slaihand_transport slaihand_hardware \
  --event-handlers console_direct+ --return-code-on-test-failure
```

Result (final): build OK, no compiler warnings from the package
(`-Wall -Wextra -Wpedantic`), `100% tests passed` for
`slaihand_transport` (1/1) and `slaihand_hardware` (7/7), including the extended
`four_axis_motion_supervision` and `five_axis_motion_audit` cases. C++ tests
prove:

- valid five-axis `kp_axis/kd_axis` accepted; wrong length, NaN, Inf, zero,
  >160 Kp, >5 Kd, six entries and four-axis arrays rejected; malformed list
  text rejected; legacy scalar policy still encodes 160/5 for every axis and is
  reported as `scalar_legacy`.
- five different per-axis Kp/Kd reach one shared ramp scale, full-ramp HOLD and
  the final MIT packet (decoded), and the request estimate recomputes exactly
  from the same encoded packet fields.
- J2 FOLLOWING_ERROR (6 deg vs the unchanged 4 deg gate) freezes J2 as first
  fault with axis/joint/code/error/limit/target/actual/age/applied encoded
  gains/request estimate/command id; a later `OPERATOR_GLOBAL_STOP` and a later
  `JTC_TRAJECTORY_ERROR` do not replace it; after STOPPING/DISABLED the state is
  `LAST_ACTIVE_STALE`; the JSON contains schema/layer/boot/cycle/frozen fields;
  operator stop without a fault invents no first fault.
- audit STAGE/CONTROL_CYCLE records carry the frozen first-cause/stale fields
  and five-element arrays without padding.

Offline PTY identity verification (temporary harness under the gitignored
`tmp/xacro_probe/`, using the existing driver PTY plant; domain 180 isolated,
fake `/dev/pts`, no physical port; only `stop_all`, no arm/enable/MIT):

- scalar-legacy policy: identity `kp_axis=[160,160,160,160,160]`,
  `kd_axis=[5,5,5,5,5]`, sources `scalar_legacy`, `following_error_deg=4`,
  `first_fault=null`; SESSION audit carries the gain arrays and STAGE records
  bind `boot_id`; dynamic state still has 17 interfaces/axis; five-axis FD
  observed, zero FC/MIT frames.
- per-axis policy (installed build-cache policy temporarily swapped, source
  YAML untouched and restored): identity `kp_axis=[150,145,140,135,130]`,
  `kd_axis=[4.5,4,3.5,3,2.5]`, sources `per_axis`, new `policy_sha256`
  (`611905428c98ae2c9299b55db380fc208c9c63bed77b243912a2763c3e42c2f6`), same
  audit/state checks. The source policy hash remains
  `f8ae7e83a2f898aad14096f66b0cb90238bb507428e4373b3c8c3810f08897e4`.

## 5. Explicit limitations / not done

- No real motor, CAN, USB, field process or panel was touched; no 5-axis
  physical gain tuning, no 1.5x margin qualification, no gravity compensation.
- Five-axis gravity FF/lock/resume remains unavailable by existing policy;
  per-axis gain code there is unit-consistent but not exercised on five axes.
- The adopted policy file was not modified (no automatic gain change); using
  per-axis values is a future operator/field decision that changes
  `policy_sha256` and invalidates old tickets via the existing machinery.
- No commit was made; another track owns panel/web/Python changes.
