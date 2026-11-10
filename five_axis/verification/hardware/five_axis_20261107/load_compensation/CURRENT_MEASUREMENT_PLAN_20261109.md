# Current/bus measurement plan (2026-11-09)

The current software chain has no measured current or bus voltage:

- `dynamic_joint_states` has position/velocity/effort/state/temperature only;
- no `bus_current`, `bus_voltage`, or external current topic;
- `effort` and PD request are estimates, not current.

## Required hardware

- bus current/voltage sensor (shunt + INA226/INA219 or a power analyzer) on the
  motor supply, with a known range and polarity;
- optional per-phase or per-drive current if available;
- sampling synchronized to the ROS state stream (same host clock or trigger);
- recorded units, calibration date, gain/offset, sample rate, range and validity.

## Required capture bundle

- J1–J5 target, actual, velocity, reference velocity, feedback validity/age;
- MIT encoded Kp, Kd, tau_ff and last-written timestamp (now in audit);
- bus voltage/current samples with timestamps;
- driver stage, command_id, first-fault/stale state;
- artifact/evidence hashes, policy hash, load id, git HEAD.

## Experiment sequence

1. Static holds at independent single-axis and mixed poses.
2. Positive/negative approach to the same pose.
3. Repeat holds and independent runs.
4. Slow dynamic moves and, separately, faster moves; record peak/overshoot,
   current/voltage, temperature and stopping behavior.
5. Validate the model on held-out runs before any FF implementation.

## Constraints

- Do not use `Kp*error`, effort, or protocol TMAX as measured current;
- do not loosen the 4°/STOP/timing/serial limits;
- no 1.5× margin claim until real allowed output, duration, temperature/power
  and demand evidence exist;
- current work remains offline; no driver/panel/motion is started by this plan.
