# Alternative to per-axis current: per-joint torque estimate (2026-11-09)

Per-axis current is not available from the current USB2CAN protocol/feedback.
A practical alternative exists: the DAMIAO MIT feedback already provides a
per-joint **torque estimate** (`effort`), updated with the 100 Hz dynamic state.

## Evidence from the existing large-angle run

At the corrected +40° window:

```text
J1 effort estimate ≈ 11.673 Nm
J4 effort estimate ≈  2.870 Nm
```

Across the staged forward run:

| relative J1 | J1 torque estimate | J4 torque estimate |
|---:|---:|---:|
| 0.11° | 0.088 Nm | 0.091 Nm |
| 10.11° | 5.275 Nm | 0.142 Nm |
| 20.11° | 7.762 Nm | 1.013 Nm |
| 30.11° | 9.429 Nm | 2.402 Nm |
| 40.00° | 11.673 Nm | 2.870 Nm |

This is a per-axis torque estimate, not measured current. It is much more
axis-resolved than the total bus current.

## Recommended approach

1. Keep the existing per-joint `effort` in the recorder.
2. Add a **bus voltage/current** source only as an aggregate sanity check:
   - WANPTEK supply if it has a real data interface, or
   - an INA226 on the 24 V output.
3. Collect independent static poses with:
   - controller target/actual/velocity;
   - per-joint effort/torque estimate;
   - bus voltage/current;
   - feedback validity and state.
4. Fit a path-local torque-estimate proxy per axis, with leave-one-out and
   independent-pose validation.
5. Implement a bounded FF only after that validation; validate in software/PTY
   before any field test.
6. Do not call the torque estimate "current" or use it for a 1.5× current/thermal
   margin. Real current still requires a sensor or vendor current telemetry.

## Why this is acceptable for the next step

- It provides **per-axis** information, unlike total bus current.
- It can separate J1/J4 trends and support a static gravity/load proxy.
- It does not require opening the motor wiring or adding five current sensors.
- Its limitations are explicit: estimate, not measurement; path-local; no
  multi-pose coupling yet; no saturation/thermal proof.

## What it cannot do

- Prove motor current limit or bus current demand;
- prove 1.5× capability;
- replace per-axis current for a formal electrical acceptance.

Tool: `signed_acquisition/analyze_static_torque.py` joins controller windows with
the recorder's dynamic effort; no ROS/hardware is used by the tool.

Current status: driver stopped; no panel; no motion.
