# Serial/timing investigation and low-latency change (2026-11-09)

## Symptom

The schedule A/B runs were intermittently stopped by:

```text
CONTROL_CYCLE_GAP
SERIAL_OR_AUDIT_WRITE_FAILURE
```

with controller-manager read/write stalls of ~20–40 ms.

## Read-only probe

`serial_latency_probe.py` sends only the existing J5 register read; it never
enables, sends MIT, writes zero, or commands motion.

200 requests on the attached `/dev/ttyACM0`:

```text
write_ms  min=0.016 p50=0.028 p95=0.047 p99=0.064 max=0.093
read_ms   min=0.665 p50=1.010 p95=1.375 p99=3.356 max=12.184
timeouts  0
```

The link is normally fast, but the driver still sees rare 20–40 ms stalls,
consistent with USB/IP or the Windows USB stack, not with the 30-byte protocol.

## Code change

`ReadOnlyPort::open()` now applies a **best-effort** Linux `ASYNC_LOW_LATENCY`
flag through `TIOCGSERIAL/TIOCSSERIAL`, after the existing raw/921600 setup.

- A direct probe confirmed `TIOCGSERIAL`/`TIOCSSERIAL` both return success on
  this WSL cdc_acm device.
- Failure remains non-fatal: no protocol or safety behavior changes.
- It does not retry or loosen any deadline.

## Retest

After the change, a forward-only scheduled J1 +10 run received sampled holds
without a cycle-gap abort:

```text
J1 max_abs = 0.995 deg
JTC: goal_time_tolerance abort
```

The serial/audit failure did not recur in this run. This is a single run, not a
stability proof.

The A/B is still incomplete: constant A started at J1 -1.508 deg, schedule B at
J1 -1.093 deg (return left a 0.415 deg offset). Therefore no clean schedule
benefit is claimed.

## Current status

- driver stopped;
- no panel, no active motion;
- no backward motion beyond the previously approved slow return;
- no current/thermal/dynamic acceptance;
- no 1.5x margin claim.
