# Backward J1 test and power/CAN chain note (2026-11-09)

## Operator setup confirmation

- rear obstruction cleared;
- J1-J2-J3 are one power/CAN series group;
- J4-J5 are another power/CAN series group;
- recent motion is authorized in both directions, but this run still tests backward
  only after a fresh admission.

The supply/CAN grouping does not change the per-joint control chain, but it means
a single bus current sensor can give at most a **group total**, never per-axis
current. The per-joint `effort` torque estimate remains the only current axis-
resolved proxy.

## Admission

- J5 TIMEOUT read 0, written 2000, read back 2000;
- fresh evidence `j5_timeout_setup_evidence_20261109_d.json`;
- fresh artifact generated under default freshness;
- no driver/panel left running.

## Backward J1 test

`BACKWARD_J1_10_SLOW.json`, 15 s move / 8 s dwell, no return.

| window | relative J1 | J1 mean | J1 max_abs | Kp | J1 torque estimate |
|---|---:|---:|---:|---:|---:|
| start | -0.11° | +0.088° | 0.113° | 160 | -0.436 Nm |
| backward | -10.00° | +1.126° | **1.126°** | 160 | **-3.315 Nm** |

- Backward motion completed and was sampled; no obstruction contact;
- JTC then aborted on `goal_time_tolerance` (~2.0 s);
- at negative model angle the schedule clamps to 160, so this is not an
  angle-schedule gain test;
- still above 0.5° precision; no current/thermal/dynamic acceptance.

## Current status

- driver stopped;
- no panel, no active motion;
- no measured current; power/CAN groups only allow aggregate current;
- no 1.5x margin claim.
