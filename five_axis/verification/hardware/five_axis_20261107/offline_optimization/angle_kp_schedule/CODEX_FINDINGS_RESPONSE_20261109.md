# Response to Codex angle-schedule findings (2026-11-09)

Repo: `slaihand_five_axis_ws_gain` / `feature/five-axis-gain-test-20261108`.
No driver/panel/hardware action was performed for these fixes.

## 1. Arm/reset schedule lifecycle — fixed

- `reset_kp_schedule_state()` clears initialized/last-at/target/effective/segment.
- Called on every accepted arm and every accepted reset.
- Next arm initializes from the fresh preflight pose before PRELOAD.
- Added integration test: hold, stop, disconnect, set simulated pose 40°, reset,
  arm; the new scheduled target is 280, not the inherited 215.571.
- PRELOAD still encodes Kp=0 until the existing enable ramp; no full-gain jump
  is claimed.

## 2. Identity/audit effective semantics — fixed

- `gain_axis_json()` is now static configuration only: configured/per-axis Kp,
  `kp_max`, schedule source/points/slew. Dynamic effective/target/segment are no
  longer cached into runtime identity.
- Audit CYCLE/STAGE now has separate fields:
  `kp_scheduled`, `kp_schedule_target`, `kp_schedule_segment`,
  `kp_scaled`, `kp_encoded`, `kp_last_written_at`, `kp_output_state`.
- `kp_scaled` is the post-ramp value used to build the latest MIT packet;
  `kp_encoded` is decoded from a packet that reached the write path;
  `kp_last_written_at` gives the per-axis write time.
- `kp_output_state` distinguishes `WRITTEN_CURRENT`, `NOT_YET_WRITTEN`,
  `STALE_DISABLED`, `NO_WRITE`, and `NOT_ARMED`.
- The existing enable/hold ramp remains in `kp_scaled`; the scheduled value is
  not presented as the actual output.

## 3. Analyzer Kp assumption — fixed

`analyze_signed.py` now accepts `--kp-axis` or `--kp-schedule` and computes:

```text
pd_request_estimate = -effective_kp * (feedback - reference)
```

It is labeled a PD request estimate, not measured torque/current. For the 40°
schedule point J1, the corrected estimate is about **11.865 Nm** instead of the
old fixed-160 estimate of 6.78 Nm.

## 4. Static windows/origin/labels — fixed

- Added `--client-result`: windows are selected by the client target list.
- The selected window must have whole-window reference span <= 0.0015 rad and
  feedback speed <= 0.05 rad/s; adjacent-sample closeness is no longer accepted.
- Origin is the client initial model pose, not the first moving group.
- Corrected large-angle values (0.3 s windows, scheduled Kp):
  - +10°: J1 max 1.633°, mean -1.608°, Kp≈194.95
  - +20°: J1 max 2.067°, mean -2.046°, Kp≈223.63
  - +30°: J1 max 2.225°, mean -2.193°, Kp≈253.19
  - +40°: J1 max 2.428°, mean -2.428°, Kp=280
- +40 window reference span is 0.0° and matches the client result.
- Earlier 1.8506/2.1916/2.5119 values are superseded; those windows mixed
  motion/settling.
- The +10/+20/+30 corrected values remain above 0.5°; no performance win is
  claimed.

## 5. Serial low-latency observation

- `TIOCGSERIAL`, `TIOCSSERIAL`, errno, and readback state are now retained and
  logged at startup (`SERIAL_LOW_LATENCY get=... set=... readback=... errno=...`).
- This remains best-effort and does not alter protocol/deadline/safety limits.
- A single no-gap run is still not proof that USB-IP long-tail latency is fixed.

## Tests

```text
C++ hardware/transport: 7/7 passed
Signed analyzer: 4/4 passed
```

No hardware stability, current, thermal, or 1.5x acceptance is claimed.
