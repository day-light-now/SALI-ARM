# GC trajectory execution aborted: serial/USB-IP failure (2026-11-09)

The Codex trajectories were submitted. **None completed.**

## GC01 — attempt 1

- `GC01_J1_J2_PAIRED_ZERO.json`
- First zero-hold window sampled;
- then supervisor stopped with `SERIAL_OR_AUDIT_WRITE_FAILURE`
  (`control_state 7 -> 8`, output marked stale/disabled);
- only **1/9** waypoints sampled;
- no 10°/20° J1/J2 motion completed.

## GC01 — attempt 2 after driver restart

- Fresh driver and reset;
- three windows sampled (zero, J1+10°, J1+20°);
- stopped again with the same serial/audit failure;
- only **3/9** waypoints sampled;
- J1 had reached about 20°, then the driver entered the safe stop path.

No collision. No manual continuation after the stop.

## Root cause evidence

- failure stage reason: `SERIAL_OR_AUDIT_WRITE_FAILURE`;
- kernel log shows `vhci_hcd` URB status `-104` (connection reset) around the
  same time;
- this is the same USB/IP long-tail/reset class seen earlier, not a gravity
  feedforward or trajectory-file error.

## Not executed

- GC02 J3/J4 paired
- GC03 mixed holdout
- GC04 staged 40°

They were **not** started; the run was stopped after the second GC01 failure.

## Current state

- driver stopped;
- no gravity feedforward enabled;
- no panel motion;
- no current/thermal/1.5x claim;
- trajectory files themselves remain valid; execution blocked by the USB link.

## Next

Stabilize the USB/USB-IP path before any further trajectory execution:

1. try a different USB port/cable/hub;
2. avoid Windows USB resets while ROS is running;
3. consider a native/non-USB-IP serial bridge;
4. verify a long idle + short motion session without `vhci_hcd -104` and without
   `SERIAL_OR_AUDIT_WRITE_FAILURE`;
5. only then re-run GC01 from a known zero pose.

Full raw attempts and clients are under `execution_20261109/`.
