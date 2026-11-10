# J5 TIMEOUT setup tool (one-shot, bounded) — executed 2026-11-08

**Status: EXECUTED on real hardware on 2026-11-08** (operator instruction; DSH
stopped the old read-only owner first). The tool changed TIMEOUT 0 -> 2000 and
read it back successfully. Evidence:
`verification/hardware/five_axis_20261107/evidence/j5_timeout_setup_evidence.json`.
The mapping is now `commissioning_timeout_count: 2000`.
TIMEOUT is a RAM parameter, not saved to Flash; a power cycle may revert it.

Original candidate files (branch `feature/five-axis-unified-20261107`):

- `src/slaihand_bringup/scripts/j5_timeout_setup.py` — the one-shot tool
  (source only; not added to `CMakeLists.txt` and not installed by this change)
- `verification/hardware/five_axis_20261107/test_j5_timeout_setup.py` — PTY-only
  tests
- `verification/hardware/five_axis_20261107/j5_timeout_setup_README.md` — this file

The command below remains the reproducible procedure. The 2026-11-08 execution
used `/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00`
(realpath `/dev/ttyACM0`) and all five axes were fresh `motor_state=0` before the
single RID9 write.

## Preconditions (Codex must check before running)

- only the J5 motor is connected and reachable over the USB2CAN adapter;
- the arm is mechanically supported, all five drives are disabled
  (`motor_state == 0`), and the adapter's serial device is not held by the
  driver, panel, browser page, DMTool or any other process;
- power cutoff is available;
- no ROS driver/panel is running (this tool is the only serial owner for its
  short lifetime; it takes `flock LOCK_EX|LOCK_NB` + `TIOCEXCL` and closes at
  the end).

## Exact command (placeholder — do NOT run unless performing the real setup)

Run from the workspace root of the candidate worktree, in the environment where
the USB2CAN adapter is visible (Linux/WSL `/dev/ttyUSB*` or `/dev/ttyACM*`; a
Windows `COMx` name is refused by design):

```bash
python3 src/slaihand_bringup/scripts/j5_timeout_setup.py \
  --port /dev/ttyUSB0 \
  --apply-timeout 2000 \
  --confirm EXCLUSIVE_J5_SETUP \
  --audit verification/hardware/five_axis_20261107/evidence/j5_timeout_setup_evidence.json
```

- `--port` must be an absolute `/dev/...` path; the tool resolves its realpath,
  opens it with pyserial 921600 8N1, and requires `flock LOCK_EX|LOCK_NB` plus
  `TIOCEXCL` before any read or write. An already-locked port is refused.
- `--apply-timeout` must be exactly `2000` (no other value is accepted); the
  wire value is the reviewed Damiao count (nominally 100 ms at the 50 µs base).
- `--confirm` must be exactly `EXCLUSIVE_J5_SETUP`.
- `--audit` is the single JSON evidence/audit document. Keep exactly
  `verification/hardware/five_axis_20261107/evidence/j5_timeout_setup_evidence.json`
  so the J5 admission slice can consume it. If that file already records a
  parameter-write attempt (`write_attempts`/`parameter_writes` > 0), the tool
  refuses to overwrite it: move it aside and use a new `--audit` path, because
  a frame may already be on the wire. Evidence from a *pre-write* refusal may
  be overwritten by a later corrected run.
- Exit code: `0` only when the complete setup succeeded; `1` when a guarded
  step failed (an audit JSON is still written); `2` when the CLI was refused
  (nothing was opened or written).

## What it does, in order

1. **Open + exclusive lock** (realpath, `flock`, `TIOCEXCL`), then bounded
   preflight.
2. **Preflight BEFORE any write**, reading all J1–J5 from the same port:
   - fresh feedback for every axis J1–J5 (`age <= 0.25 s`) with actual
     `motor_state == 0` on all five;
   - official ID5 registers via `0x33`: CAN ID 5, master ID 0, control mode 1,
     TIMEOUT 0, PMAX 12.5, VMAX 10, TMAX 28, with at least two conflict-free
     observations per register, no conflicts and no missing register. A reply
     whose payload is not ID5 (e.g. a J4 reply) is not accepted as J5.
3. **Write exactly once**, no retry and no Flash/save:
   `CAN ID 0x7FF`, data `05 00 55 09 d0 07 00 00`
   (`CAN_ID_L`, `CAN_ID_H`, `0x55` parameter-write, `RID=9`, `uint32(2000)`),
   packed into the existing official 30-byte USB2CAN frame and submitted with
   the reviewed bounded `J5FrameWriter`. `eagain` / `partial` / `io_error`
   outcomes are recorded in the audit and fail the run; a partially or fully
   submitted frame is never resent.
4. **Bounded readback** of RID 9 via `0x33` until it reads `2000` (deadline
   ~2.5 s), then a bounded post state/profile read. `READBACK_MISMATCH`,
   `READBACK_TIMEOUT` or a changed post profile fail the run.
5. Always close the port and write the JSON evidence (also on guarded failure).

Never sent: Flash/save command, ID write, zero write, enable, disable, MIT or
any motion frame. All non-write frames are `0x33` register reads or `0xCC`
feedback requests on the broadcast parameter CAN ID `0x7FF`.

## Evidence contract (consumed by the J5 admission slice)

File: `verification/hardware/five_axis_20261107/evidence/j5_timeout_setup_evidence.json`
Schema: `slaihand_j5_timeout_setup/1`. Admission accepts only when:

- `ok is true`, `write_outcome == "complete"`, `write_attempts == 1`,
  `parameter_writes == 1`;
- `timeout_before == 0`, `timeout_after == 2000`,
  `readback.matched is true` with `readback.observed == 2000`;
- `motor_state_all_zero is true` (and `all_motor_state_zero` agrees), with
  `axes` listing J1–J5 `motor_state == 0` and `valid is true`;
- `j5_id == 5`, `master_id == 0`, `control_mode == 1`,
  `profile == {pmax_rad: 12.5, vmax_rad_s: 10.0, tmax_nm: 28.0}`;
- `motor_zero_written`, `flash_written`, `enabled`, `mit_sent` are all `false`;
- `device` (realpath), `port_argument`, `session_id`, `host_boot_id`,
  `observed_at` (UTC ISO wall time), `tool.sha256` (64 hex) and `audit_path`
  are present.

Write-related evidence: `write_frame_hex` (full 30-byte USB frame),
`write_data_hex` (`05 00 55 09 d0 07 00 00`), `readback_hex` (8-byte RID9
reply data), plus `preflight`/`postflight`/`trace` details. The tool does not
modify the mapping or any TIMEOUT declaration; if the real run is never
performed; the 2026-11-08 execution recorded TIMEOUT 0 -> 2000 and mapping now
contains 2000.

## Limits

- This tool only sets TIMEOUT and reads it back. It does not save to Flash, so
  a power cycle may revert the value (that is expected and is why no save
  command exists here); it does not enable the drive, does not verify the
  watchdog fault latency, and does not write the mapping.
- A successful run proves the parameter was accepted and read back on that
  session/port; the admission still needs its own qualification steps.
- It refuses to run when TIMEOUT is already nonzero (one-shot setup only), and
  it refuses to overwrite an `--audit` file that already records a
  parameter-write attempt.

## PTY-only tests (run in WSL; no real device)

```bash
cd /path/to/slaihand_five_axis_ws
python3 verification/hardware/five_axis_20261107/test_j5_timeout_setup.py
# or: python3 -m pytest verification/hardware/five_axis_20261107/test_j5_timeout_setup.py -q
```

The tests use a generated PTY (`/dev/pts/N`) and a fake five-axis emulator.
They cover: success with exactly one `0x55 RID9` write and no other command
frames; refusal of missing/wrong CLI and non-`/dev/` ports; wrong J5 CAN
ID/master ID/mode/profile/nonzero TIMEOUT; any axis in state 1; stale
feedback; register conflicts; locked port; `eagain`/`partial`/`io_error`
parameter-write faults with no retry; readback mismatch; and post-write profile
change. No real serial/CAN/USB device is opened by the tests.
