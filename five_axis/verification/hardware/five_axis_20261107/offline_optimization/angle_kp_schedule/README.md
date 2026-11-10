# Angle-scheduled Kp implementation (offline candidate)

This is a source-level candidate. It does **not** start ROS, driver, panel, serial,
CAN, or motion. No hardware was used to build or test it.

## Policy schema

Optional five-axis schedule, one semicolon-separated list per J1..J5. Each point
is `angle_deg:kp`; angles must strictly increase:

```yaml
# example only; not enabled in any live policy
kp_schedule: "0:160,20:220,40:280;0:160,20:240,40:280;0:160,20:180,40:200;0:160,20:200,40:220;0:160,20:180,40:200"
kp_schedule_slew_kp_per_s: 80.0
```

The same values are passed through the five-axis URDF as `kp_schedule` and
`kp_schedule_slew_kp_per_s`.

## Behaviour

- input angle: measured model-frame joint position, converted rad -> deg;
- linear interpolation between points;
- clamp below the first and above the last point;
- invalid/unsorted/duplicate/non-finite points are rejected;
- 1..32 points per axis; schedule count must equal active axes;
- Kp bound is the explicit policy `kp_max` (default 160; this experimental
  branch sets `kp_max: 280.0`; the wire protocol 500 is not a capability cap);
- rate-limited gain transition: `|delta Kp| <= slew * dt`, with `dt` capped at
  50 ms so a long gap cannot make a gain jump;
- first control cycle initializes the effective gain at the measured pose, then
  later cycles slew toward the interpolated target;
- Kd remains per-axis constant; floating/hold modes keep their existing scaling
  (scheduled Kp is multiplied by the same hold/ramp factors).

## Audit and identity

`gain_axis_json()` now includes:

- `kp_axis` (nominal configured values);
- `kp_effective` (what the next MIT packet uses);
- `kp_schedule_target` (interpolated target before slew);
- `kp_schedule_segments` (current segment per axis);
- `kp_schedule_source` (`angle_schedule` or `none`);
- `kp_schedule_slew_kp_per_s` and the configured points.

Audit `CONTROL_CYCLE` and `STAGE` records carry `kp_effective`,
`kp_schedule_target`, and `kp_schedule_segment` per active axis. The session
metadata carries the schedule points, so a log reader can reproduce the input
without re-reading the policy file.

## Tests

```bash
cd /mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain
source /home/xuyu/.cache/dsh_sched/install/setup.bash
/home/xuyu/.cache/dsh_sched/build/slaihand_hardware/motion_supervisor_test
colcon test --build-base /home/xuyu/.cache/dsh_sched/build \
  --install-base /home/xuyu/.cache/dsh_sched/install \
  --packages-select slaihand_hardware --event-handlers console_direct+
```

The supervisor test covers parser bounds, endpoint clamp, interpolation,
rate limiting, invalid schedule rejection, policy bounds, activation, and audit
JSON fields. Full hardware package tests are 7/7.

## Not done / not claimed

- No live driver/panel run with a schedule;
- no PTY closed-loop dynamics yet;
- no hardware acceptance, capability, or 1.5x margin;
- no schedule enabled in the current policy;
- no backward/obstruction motion.
