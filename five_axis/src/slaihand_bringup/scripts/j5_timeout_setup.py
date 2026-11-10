"""One-shot J5-only bounded TIMEOUT setup; exactly one official parameter write.

Scope: set the J5 (DM-J4340-2EC, CAN ID 5) driver watchdog parameter
RID 9 (TIMEOUT) to the reviewed value 2000 once, then read it back. This exists
so Codex can execute a bounded, exclusive, evidence-producing step later.

Safety properties (all enforced here):
- one-shot process, no thread, no daemon, no owner hand-over;
- exclusive tty: realpath, flock LOCK_EX|LOCK_NB and TIOCEXCL before any I/O;
- preflight BEFORE any write: fresh (<= 0.25 s) feedback for J1-J5 must show
  all five motors disabled (state 0), and the official ID5 registers read via
  0x33 must be CAN ID 5, master ID 0, mode 1, TIMEOUT 0, PMAX 12.5, VMAX 10,
  TMAX 28. Each register needs at least two conflict-free observations before
  the write, so a register that changes between reads blocks the write;
- exactly one official Damiao parameter-write frame is submitted:
  CAN ID 0x7FF, data [CAN_ID_L, CAN_ID_H, 0x55, RID=9, uint32(2000)];
  a partial/eagain/io_error outcome is audited and fails without retry;
- no Flash/save command, no ID write, no zero write, no enable, no MIT;
- RID 9 is read back via 0x33 until it equals 2000 or a bounded deadline;
- one JSON evidence document is written even when the setup is refused/fails.

The transport keeps to the protocol facts already reviewed for J5 in
j5_readonly (30-byte USB2CAN framing, 0x33/0xCC request payloads, bounded
J5FrameWriter completion semantics, J5State conflict tracking, feedback
decode). J5Reader is deliberately NOT imported or started: this tool polls
only inside its own bounded preflight/readback windows.
"""
from __future__ import annotations

import argparse
import datetime
import fcntl
import hashlib
import json
import math
import os
import struct
import sys
import termios
import time
import uuid
from pathlib import Path

try:
    from .j5_readonly import (J5_ID, MAX_SAMPLE_AGE_S, REGS, J5FrameWriter, J5State,
                              decode_feedback, usb_packet)
except ImportError:  # scripts are also executed directly
    from j5_readonly import (J5_ID, MAX_SAMPLE_AGE_S, REGS, J5FrameWriter, J5State,
                             decode_feedback, usb_packet)

SCHEMA = "slaihand_j5_timeout_setup/1"
CONFIRM_TOKEN = "<redacted-panel-token>"
APPLY_TIMEOUT = 2000
TIMEOUT_RID = 9
WRITE_CAN_ID = 0x7FF
BAUD = 921600
FRESH_LIMIT_S = MAX_SAMPLE_AGE_S  # 0.25 s, same limit as the reviewed J5 reader
PREFLIGHT_TIMEOUT_S = 3.0
POST_TIMEOUT_S = 2.5
SEND_INTERVAL_S = 0.003
CYCLE_INTERVAL_S = 0.010
DRAIN_WINDOW_S = 0.020
TRACE_LIMIT = 5000
AXES = (1, 2, 3, 4, 5)
EXPECTED_J5 = {7: 0, 8: J5_ID, 9: 0, 10: 1, 21: 12.5, 22: 10.0, 23: 28.0}
EXPECTED_J5_POST = {**EXPECTED_J5, 9: APPLY_TIMEOUT}
EXPECTED_J5_EXISTING = {**EXPECTED_J5, 9: APPLY_TIMEOUT}
REGISTER_NAMES = {7: "master_id", 8: "can_id", 9: "timeout", 10: "control_mode",
                  21: "pmax_rad", 22: "vmax_rad_s", 23: "tmax_nm"}
FLOAT_REGISTERS = (21, 22, 23)

# Reviewed USB2CAN 30-byte output frame. DLC 8 at [18]; CAN ID at [13:15]
# (0x7FF, little-endian); the 8-byte CAN data goes to [21:29]. Byte 29 stays 0.
USB_FRAME = bytes([0x55, 0xAA, 0x1E, 3, 1, 0, 0, 0, 0x0A, 0, 0, 0, 0,
                   0xFF, 0x07, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
PARAMETER_WRITE_DATA = (bytes([J5_ID & 0xFF, (J5_ID >> 8) & 0xFF, 0x55, TIMEOUT_RID])
                        + struct.pack("<I", APPLY_TIMEOUT))


class SetupError(RuntimeError):
    """A bounded, audited setup failure. The port is always closed afterwards."""


class UsageRefused(ValueError):
    """A refused invocation; nothing is opened or written."""


def _iso(wall_time):
    return datetime.datetime.fromtimestamp(
        wall_time, datetime.timezone.utc).isoformat(timespec="milliseconds")


def _boot_id():
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _tool_sha256():
    try:
        return hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
    except OSError:
        return None


def _as_int(value):
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def value_matches(rid, value, expected):
    """Exact compare for reviewed integer registers / 1e-6 for float registers."""
    if value is None:
        return False
    if rid in FLOAT_REGISTERS:
        try:
            return math.isfinite(float(value)) and abs(float(value) - float(expected)) <= 1e-6
        except (TypeError, ValueError):
            return False
    return _as_int(value) == int(expected)


def usb_frame(data):
    """Wrap one 8-byte CAN data payload into the reviewed 30-byte USB frame."""
    if len(data) != 8:
        raise SetupError("USB frame payload must be exactly 8 bytes, got %d" % len(data))
    frame = bytearray(USB_FRAME)
    frame[21:29] = bytes(data)
    return bytes(frame)


def request_frame(motor_id, rid=None):
    """0xCC feedback or 0x33 register read request for any reviewed axis ID."""
    if rid is None:
        data = bytes([motor_id & 0xFF, 0, 0xCC, 0, 0, 0, 0, 0])
    else:
        data = bytes([motor_id & 0xFF, 0, 0x33, rid, 0, 0, 0, 0])
    return usb_frame(data)


def parameter_write_frame(data=PARAMETER_WRITE_DATA):
    """One official Damiao parameter write to the broadcast CAN ID 0x7FF."""
    return usb_frame(data)


class FeedbackFrames:
    """16-byte AA 11 08 <CAN ID LE> <data 8> 55 feedback transport decoder."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while self.buffer:
            if self.buffer[0] != 0xAA:
                del self.buffer[0]
                continue
            if len(self.buffer) < 16:
                break
            if self.buffer[1] != 0x11 or self.buffer[2] != 8 or self.buffer[15] != 0x55:
                del self.buffer[0]
                continue
            raw = bytes(self.buffer[:16])
            del self.buffer[:16]
            frames.append((int.from_bytes(raw[3:7], "little"), raw[7:15]))
        return frames


class Trace:
    """Bounded TX/RX/event audit trail; the evidence JSON is the audit file."""

    def __init__(self, clock=time.monotonic, wall=time.time, limit=TRACE_LIMIT):
        self.clock = clock
        self.wall = wall
        self.limit = limit
        self.events = []
        self.dropped = 0

    def event(self, direction, can_id=None, payload=None, **fields):
        if len(self.events) >= self.limit:
            self.dropped += 1
            return
        entry = {"seq": len(self.events) + 1, "at": round(self.clock(), 6),
                 "wall": round(self.wall(), 6), "direction": direction}
        if can_id is not None:
            entry["can_id"] = int(can_id)
        if payload is not None:
            entry["payload"] = payload
        entry.update(fields)
        self.events.append(entry)


class ExclusivePort:
    """One open, exclusively locked tty for this one-shot process."""

    def __init__(self, path, trace, clock=time.monotonic, serial_factory=None,
                 frame_writer_factory=None):
        self.argument = str(path)
        self.path = os.path.realpath(str(path))
        self.trace = trace
        self.clock = clock
        self.serial_factory = serial_factory
        self.frame_writer_factory = frame_writer_factory
        self.serial = None
        self.writer = None
        self._locked = False

    def open(self):
        import serial as pyserial
        factory = self.serial_factory or pyserial.Serial
        try:
            self.serial = factory(self.path, BAUD, timeout=0, write_timeout=.05)
        except Exception as error:
            self.serial = None
            raise SetupError("PORT_OPEN_FAILED: %s: %s" % (self.path, error)) from error
        try:
            fd = self.serial.fileno()
        except (AttributeError, OSError) as error:
            self._close_serial()
            raise SetupError("PORT_OPEN_FAILED: %s has no usable fd: %s"
                             % (self.path, error)) from error
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
        except OSError as error:
            self._close_serial()
            raise SetupError(
                "PORT_BUSY: %s is already locked or cannot be made exclusive: %s"
                % (self.path, error)) from error
        self._locked = True
        try:
            os.set_blocking(fd, False)
            self.writer = (self.frame_writer_factory(fd, self.clock)
                           if self.frame_writer_factory
                           else J5FrameWriter(fd, clock=self.clock))
            try:
                self.serial.reset_input_buffer()
            except AttributeError:
                pass
        except Exception:
            self.close()
            raise
        self.trace.event("open", path=self.path, baud=BAUD,
                         flock="LOCK_EX|LOCK_NB", tiocm="TIOCEXCL")

    def write(self, frame):
        """Submit one complete 30-byte frame. Never retries here."""
        outcome = self.writer.write(frame)
        kind = outcome.get("kind")
        if kind == "complete":
            self.trace.event("tx", WRITE_CAN_ID, frame[21:29].hex(" "))
        else:
            self.trace.event("write_fault", WRITE_CAN_ID, json.dumps(
                {"outcome": outcome, "frame": frame.hex(" ")}, sort_keys=True))
        return outcome

    def read_available(self):
        if self.serial is None:
            return b""
        try:
            count = self.serial.in_waiting
        except AttributeError:
            count = 0
        return self.serial.read(count)

    def close(self):
        self._close_serial()

    def _close_serial(self):
        serial = self.serial
        if serial is None:
            return
        try:
            fd = serial.fileno()
            if self._locked:
                try:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                except OSError:
                    pass
                try:
                    fcntl.flock(fd, fcntl.LOCK_UN)
                except OSError:
                    pass
        except (AttributeError, OSError):
            pass
        try:
            serial.close()
        finally:
            self.serial = None
            self._locked = False
            self.trace.event("close", path=self.path)


class Wire:
    """Accumulates per-axis feedback and J5 register replies from one port."""

    def __init__(self, trace, clock=time.monotonic, j5_state=None):
        self.trace = trace
        self.clock = clock
        self.frames = FeedbackFrames()
        self.j5 = j5_state
        self.axes = {}
        self.regs = {}
        self.register_payloads = {}
        self.register_reads = {}
        self.rx_count = 0
        self.unexpected = 0

    def drain(self, port, now=None):
        chunk = port.read_available()
        if not chunk:
            return False
        now = self.clock() if now is None else now
        for can_id, data in self.frames.feed(chunk):
            self.rx_count += 1
            self.trace.event("rx", can_id, data.hex(" "))
            self._feed(can_id, data, now)
        return True

    def _feed(self, can_id, data, now):
        if self.j5 is not None:
            self.j5.feed(can_id, data, now)
        if len(data) != 8:
            self.unexpected += 1
            return
        # 0x33 register reply: data[0] carries the official motor ID exactly
        # (state nibble 0). This is how a J4/other reply is refused as J5.
        if (data[0] == J5_ID and data[1] == 0 and data[2] in (0x33, 0xCC)
                and data[3] in REGS):
            rid = data[3]
            value = struct.unpack("<f" if rid in FLOAT_REGISTERS else "<I", data[4:])[0]
            self.regs[rid] = value
            self.register_payloads[rid] = data.hex(" ")
            self.register_reads[rid] = self.register_reads.get(rid, 0) + 1
            return
        axis = data[0] & 15
        if axis in AXES:
            self.axes[axis] = {"can_id": int(can_id), "at": now,
                               "payload": data.hex(" "), "state": data[0] >> 4}
        else:
            self.unexpected += 1


def _values_match(values, expected):
    return all(value_matches(rid, values.get(rid), want) for rid, want in expected.items())


def _axis_rows(wire, now):
    rows = []
    all_fresh = True
    all_zero = True
    for axis in AXES:
        entry = wire.axes.get(axis)
        if entry is None:
            all_fresh = False
            all_zero = False
            rows.append({"axis": "J%d" % axis, "can_id": axis, "motor_state": None,
                         "feedback_age": None, "valid": False, "feedback_payload": None})
            continue
        age = max(0.0, now - entry["at"])
        fresh = age <= FRESH_LIMIT_S
        state = entry["state"]
        all_fresh = all_fresh and fresh
        all_zero = all_zero and fresh and state == 0
        rows.append({"axis": "J%d" % axis, "can_id": axis, "motor_state": state,
                     "feedback_age": round(age, 4), "valid": fresh,
                     "feedback_payload": entry["payload"]})
    return rows, all_fresh, all_zero


def _profile_evidence(regs):
    return {"pmax_rad": regs.get(21), "vmax_rad_s": regs.get(22), "tmax_nm": regs.get(23)}


def _decoded_j5_feedback(wire, profile):
    entry = wire.axes.get(J5_ID)
    if entry is None or profile is None:
        return None
    try:
        return decode_feedback(bytes.fromhex(entry["payload"]),
                               profile["pmax_rad"], profile["vmax_rad_s"],
                               profile["tmax_nm"])
    except (ValueError, TypeError):
        return None


class TimeoutSetup:
    """Runs the bounded preflight -> one write -> bounded readback sequence."""

    def __init__(self, port, audit_path, clock=time.monotonic, wall=time.time,
                 serial_factory=None, frame_writer_factory=None,
                 verify_existing=False):
        self.clock = clock
        self.wall = wall
        self.trace = Trace(clock=clock, wall=wall)
        self.port = ExclusivePort(port, self.trace, clock=clock,
                                  serial_factory=serial_factory,
                                  frame_writer_factory=frame_writer_factory)
        self.audit_path = Path(audit_path)
        self.session_id = uuid.uuid4().hex
        self.started_wall = wall()
        self.ok = False
        self.exit_code = 1
        self.verify_existing = bool(verify_existing)
        self.error = None
        self.evidence = self._skeleton()

    def _skeleton(self):
        return {
            "schema": SCHEMA,
            "ok": False,
            "exit_code": 1,
            "error": None,
            "device": None,
            "port_argument": self.port.argument,
            "baud": BAUD,
            "exclusive": {"flock": "LOCK_EX|LOCK_NB", "tiocm": "TIOCEXCL",
                          "locked": False, "released": False},
            "observed_at": None,
            "started_at": _iso(self.started_wall),
            "completed_at": None,
            "session_id": self.session_id,
            "host_boot_id": _boot_id(),
            "tool": {"name": "j5_timeout_setup.py", "sha256": _tool_sha256()},
            "timeout_before": None,
            "timeout_after": None,
            "j5_id": J5_ID,
            "master_id": None,
            "control_mode": None,
            "profile": None,
            "motor_state_all_zero": None,
            "all_motor_state_zero": None,
            "axes": [],
            "readback": {"rid": TIMEOUT_RID, "expected": APPLY_TIMEOUT,
                         "observed": None, "attempts": 0, "matched": False},
            "readback_hex": None,
            "write_outcome": None,
            "write_frame_hex": None,
            "write_data_hex": PARAMETER_WRITE_DATA.hex(" "),
            "write_attempts": 0,
            "parameter_writes": 0,
            "read_only_requests": 0,
            "motor_zero_written": False,
            "flash_written": False,
            "enabled": False,
            "mit_sent": False,
            "audit_path": str(self.audit_path),
            "preflight": {"completed": False, "conflicts": [],
                          "missing_registers": []},
            "postflight": {"completed": False, "profile": None, "axes": []},
            "warnings": [],
            "trace": [],
            "trace_dropped": 0,
        }

    def execute(self):
        try:
            self.port.open()
            self.evidence["device"] = self.port.path
            self.evidence["exclusive"]["locked"] = True
            self._preflight()
            if self.verify_existing:
                self.evidence["write_outcome"] = "verified_existing"
            else:
                self._write_once()
            self._readback_and_postflight()
            self.ok = True
            self.exit_code = 0
        except SetupError as error:
            self.error = str(error)
            self.evidence["error"] = self.error
        except Exception as error:  # never leave an unaudited failure
            self.error = "UNEXPECTED_%s: %s" % (type(error).__name__, error)
            self.evidence["error"] = self.error
        finally:
            try:
                self.port.close()
            except Exception as error:
                self.evidence["warnings"].append("close_failed: %s" % error)
            if self.evidence["exclusive"]["locked"]:
                self.evidence["exclusive"]["released"] = True
            self.evidence["ok"] = self.ok
            self.evidence["exit_code"] = self.exit_code
            self.evidence["completed_at"] = _iso(self.wall())
            if self.evidence["observed_at"] is None:
                self.evidence["observed_at"] = self.evidence["completed_at"]
            self.evidence["trace"] = list(self.trace.events)
            self.evidence["trace_dropped"] = self.trace.dropped
            try:
                self._write_audit()
            except OSError as error:
                print("AUDIT_WRITE_FAILED: %s: %s" % (self.audit_path, error),
                      file=sys.stderr)
                self.exit_code = 1
                self.evidence["exit_code"] = 1
        return self.exit_code

    # -- bounded exchange ---------------------------------------------------

    def _drain(self, wire, duration):
        deadline = self.clock() + duration
        while True:
            wire.drain(self.port)
            if self.clock() >= deadline:
                wire.drain(self.port)
                return
            time.sleep(0.001)

    def _cycle(self, wire):
        """One bounded request batch: J5 feedback, J1-J4 feedback, J5 registers."""
        requests = ([usb_packet(J5_ID)] + [request_frame(axis) for axis in AXES[:-1]]
                    + [usb_packet(J5_ID, rid) for rid in REGS])
        for frame in requests:
            outcome = self.port.write(frame)
            if outcome.get("kind") != "complete":
                raise SetupError("READ_REQUEST_WRITE_FAULT: %r; port closed, no write sent"
                                 % (outcome,))
            self.evidence["read_only_requests"] += 1
            self._drain(wire, SEND_INTERVAL_S)
        self._drain(wire, DRAIN_WINDOW_S)

    # -- phase 1: preflight, before any write --------------------------------

    def _preflight(self):
        deadline = self.clock() + PREFLIGHT_TIMEOUT_S
        wire = Wire(self.trace, clock=self.clock, j5_state=J5State())
        while True:
            self._cycle(wire)
            now = self.clock()
            if self._preflight_ok(wire, now):
                self._preflight_evidence(wire, now)
                return
            if now >= deadline:
                self._preflight_failure(wire, now)
            time.sleep(CYCLE_INTERVAL_S)

    def _preflight_ok(self, wire, now):
        state = wire.j5
        if state is None or state.conflicts or not state.ready():
            return False
        if any(rid not in wire.regs for rid in REGS):
            return False
        # Two consistent observations per register: one read can never expose a
        # register that changed between reads, and a hidden write is refused.
        if any(wire.register_reads.get(rid, 0) < 2 for rid in REGS):
            return False
        profile = state.snapshot(now)["profile"]
        expected = EXPECTED_J5_EXISTING if self.verify_existing else EXPECTED_J5
        if profile is None or not _values_match(wire.regs, expected):
            return False
        _, all_fresh, all_zero = _axis_rows(wire, now)
        return all_fresh and all_zero

    def _preflight_evidence(self, wire, now):
        profile = wire.j5.snapshot(now)["profile"]
        rows, all_fresh, all_zero = _axis_rows(wire, now)
        self.evidence.update({
            "timeout_before": _as_int(wire.regs.get(TIMEOUT_RID)),
            "j5_id": _as_int(wire.regs.get(8)),
            "master_id": _as_int(wire.regs.get(7)),
            "control_mode": _as_int(wire.regs.get(10)),
            "profile": _profile_evidence(wire.regs),
            "motor_state_all_zero": all_zero,
            "all_motor_state_zero": all_zero,
            "axes": rows,
            "preflight": {
                "completed": True,
                "fresh_limit_s": FRESH_LIMIT_S,
                "all_feedback_fresh": all_fresh,
                "conflicts": sorted(wire.j5.conflicts),
                "missing_registers": [],
                "registers": {REGISTER_NAMES[rid]: wire.regs.get(rid) for rid in REGS},
                "register_payloads": dict(wire.register_payloads),
                "register_read_counts": dict(wire.register_reads),
                "j5_feedback": _decoded_j5_feedback(wire, profile),
            },
        })

    def _preflight_failure(self, wire, now):
        state = wire.j5
        rows, all_fresh, all_zero = _axis_rows(wire, now)
        missing = [REGISTER_NAMES[rid] for rid in REGS if rid not in wire.regs]
        mismatched = {}
        expected = EXPECTED_J5_EXISTING if self.verify_existing else EXPECTED_J5
        for rid in REGS:
            if rid in wire.regs and not value_matches(rid, wire.regs[rid], expected[rid]):
                mismatched[REGISTER_NAMES[rid]] = {
                    "observed": wire.regs[rid], "expected": expected[rid]}
        stale = [row["axis"] for row in rows if not row["valid"]]
        nonzero = [row["axis"] for row in rows if row["motor_state"] not in (None, 0)]
        self.evidence.update({
            "timeout_before": _as_int(wire.regs.get(TIMEOUT_RID)),
            "axes": rows,
            "motor_state_all_zero": all_zero,
            "all_motor_state_zero": all_zero,
            "preflight": {
                "completed": False,
                "fresh_limit_s": FRESH_LIMIT_S,
                "all_feedback_fresh": all_fresh,
                "conflicts": sorted(state.conflicts) if state is not None else [],
                "missing_registers": missing,
                "mismatched_registers": mismatched,
                "stale_axes": stale,
                "nonzero_axes": nonzero,
                "j5_ready": bool(state is not None and state.ready()),
                "registers": {REGISTER_NAMES[rid]: wire.regs.get(rid) for rid in REGS},
                "register_payloads": dict(wire.register_payloads),
                "register_read_counts": dict(wire.register_reads),
            },
        })
        raise SetupError(
            "PREFLIGHT_FAILED: missing=%s conflicts=%s mismatch=%s stale=%s nonzero=%s"
            % (missing, sorted(state.conflicts) if state is not None else [],
               mismatched, stale, nonzero))

    # -- phase 2: exactly one official parameter write -----------------------

    def _write_once(self):
        frame = parameter_write_frame()
        self.evidence["write_frame_hex"] = frame.hex(" ")
        self.evidence["write_attempts"] += 1
        try:
            outcome = self.port.write(frame)
        except OSError as error:
            self.evidence["write_outcome"] = "io_error"
            raise SetupError("PARAMETER_WRITE_IO_ERROR: %s; frame not retried" % error) from error
        self.evidence["write_outcome"] = outcome.get("kind")
        if outcome.get("kind") != "complete":
            raise SetupError(
                "PARAMETER_WRITE_FAULT: %r accepted %r/%d bytes; frame not retried, "
                "port closed, no Flash/save command sent"
                % (outcome.get("kind"), outcome.get("written"), len(frame)))
        self.evidence["parameter_writes"] += 1

    # -- phase 3: bounded RID9 readback + post state -------------------------

    def _readback_and_postflight(self):
        deadline = self.clock() + POST_TIMEOUT_S
        wire = Wire(self.trace, clock=self.clock)
        attempts = 0
        while True:
            self._cycle(wire)
            attempts += 1
            now = self.clock()
            observed = wire.regs.get(TIMEOUT_RID)
            if observed is not None:
                self.evidence["readback"]["observed"] = _as_int(observed)
                self.evidence["readback"]["attempts"] = attempts
                self.evidence["readback_hex"] = wire.register_payloads.get(TIMEOUT_RID)
            rows, all_fresh, all_zero = _axis_rows(wire, now)
            registers_complete = all(rid in wire.regs for rid in REGS)
            if (observed is not None and int(observed) == APPLY_TIMEOUT
                    and registers_complete and _values_match(wire.regs, EXPECTED_J5_POST)
                    and all_fresh and all_zero):
                self._readback_success(wire, now, rows, all_fresh, all_zero, attempts)
                return
            if now >= deadline:
                self._readback_failure(wire, now, rows, attempts)
            time.sleep(CYCLE_INTERVAL_S)

    def _readback_success(self, wire, now, rows, all_fresh, all_zero, attempts):
        profile = _profile_evidence(wire.regs)
        self.evidence.update({
            "timeout_after": APPLY_TIMEOUT,
            "observed_at": _iso(self.wall()),
            "readback": {"rid": TIMEOUT_RID, "expected": APPLY_TIMEOUT,
                         "observed": APPLY_TIMEOUT, "attempts": attempts,
                         "matched": True},
            "readback_hex": wire.register_payloads.get(TIMEOUT_RID),
            "postflight": {
                "completed": True,
                "fresh_limit_s": FRESH_LIMIT_S,
                "all_feedback_fresh": all_fresh,
                "all_motor_state_zero": all_zero,
                "profile": profile,
                "registers": {REGISTER_NAMES[rid]: wire.regs.get(rid) for rid in REGS},
                "register_payloads": dict(wire.register_payloads),
                "axes": rows,
            },
        })

    def _readback_failure(self, wire, now, rows, attempts):
        observed = wire.regs.get(TIMEOUT_RID)
        self.evidence["timeout_after"] = _as_int(observed)
        self.evidence["readback"]["observed"] = _as_int(observed)
        self.evidence["readback"]["attempts"] = attempts
        self.evidence["readback_hex"] = wire.register_payloads.get(TIMEOUT_RID)
        self.evidence["postflight"] = {
            "completed": False,
            "profile": _profile_evidence(wire.regs),
            "registers": {REGISTER_NAMES[rid]: wire.regs.get(rid) for rid in REGS},
            "register_payloads": dict(wire.register_payloads),
            "axes": rows,
        }
        if observed is None:
            raise SetupError("READBACK_TIMEOUT: no RID%d reply within %.1f s after the "
                             "single committed write; port closed" % (TIMEOUT_RID, POST_TIMEOUT_S))
        if int(observed) != APPLY_TIMEOUT:
            raise SetupError("READBACK_MISMATCH: RID%d read back %r, expected %d; the write "
                             "was committed once and is not retried"
                             % (TIMEOUT_RID, observed, APPLY_TIMEOUT))
        raise SetupError("POSTFLIGHT_MISMATCH: RID%d reads %d but the rest of the J5 "
                         "profile did not stay at the reviewed values"
                         % (TIMEOUT_RID, APPLY_TIMEOUT))

    # -- evidence -----------------------------------------------------------

    def _write_audit(self):
        self.audit_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.audit_path.with_name(self.audit_path.name + ".tmp")
        temporary.write_text(
            json.dumps(self.evidence, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        temporary.replace(self.audit_path)


def _validate_args(args):
    port = str(args.port or "").strip()
    if not os.path.isabs(port) or not port.startswith("/dev/"):
        raise UsageRefused(
            "--port must be an absolute /dev/ device path, got %r" % port)
    real = os.path.realpath(port)
    if not real.startswith("/dev/"):
        raise UsageRefused(
            "--port must resolve inside /dev/, got %r -> %r" % (port, real))
    if args.apply_timeout != APPLY_TIMEOUT:
        raise UsageRefused("--apply-timeout must be exactly %d (the only reviewed "
                           "value), got %r" % (APPLY_TIMEOUT, args.apply_timeout))
    if args.confirm != CONFIRM_TOKEN:
        raise UsageRefused("--confirm must be exactly %r, got %r"
                           % (CONFIRM_TOKEN, args.confirm))
    if not str(args.audit or "").strip():
        raise UsageRefused("--audit PATH is required")
    audit = Path(str(args.audit))
    if audit.exists():
        # Never overwrite evidence of a frame that may already be on the wire:
        # a committed/partial write must stay visible to a human and the
        # admission slice, so a retry needs a fresh --audit path.
        try:
            previous = json.loads(audit.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise UsageRefused("--audit %s exists but is not readable JSON (%s); "
                               "choose a new evidence path" % (audit, error))
        if isinstance(previous, dict) and (previous.get("write_attempts")
                                           or previous.get("parameter_writes")):
            raise UsageRefused(
                "--audit %s already records a parameter-write attempt; move it "
                "aside and use a new evidence path instead of overwriting it" % audit)
    try:
        audit.parent.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise UsageRefused("--audit parent cannot be created: %s: %s"
                           % (audit.parent, error))


def _parser():
    parser = argparse.ArgumentParser(
        prog="j5_timeout_setup.py",
        description="One-shot exclusive J5-only setup of RID9 TIMEOUT=2000. "
                    "Writes exactly one official parameter frame, then reads it back. "
                    "No Flash/save, no ID/zero write, no enable, no MIT.")
    parser.add_argument("--port", required=True,
                        help="absolute /dev/... USB2CAN tty (e.g. /dev/ttyUSB0)")
    parser.add_argument("--apply-timeout", required=True, type=int,
                        help="reviewed TIMEOUT count to write; only 2000 is accepted")
    parser.add_argument("--confirm", required=True,
                        help="exclusive J5 setup confirmation token: %s" % CONFIRM_TOKEN)
    parser.add_argument("--audit", required=True,
                        help="path of the one JSON evidence/audit document to write")
    parser.add_argument("--verify-existing", action="store_true",
                        help="verify an existing TIMEOUT=2000 by readback; do not write TIMEOUT")
    return parser


def run(argv=None):
    args = _parser().parse_args(argv)
    try:
        _validate_args(args)
    except UsageRefused as error:
        print("REFUSED: %s" % error, file=sys.stderr)
        return 2
    setup = TimeoutSetup(args.port, args.audit, verify_existing=args.verify_existing)
    exit_code = setup.execute()
    print(json.dumps({
        "schema": SCHEMA,
        "ok": setup.evidence["ok"],
        "device": setup.evidence["device"],
        "timeout_before": setup.evidence["timeout_before"],
        "timeout_after": setup.evidence["timeout_after"],
        "write_outcome": setup.evidence["write_outcome"],
        "audit_path": setup.evidence["audit_path"],
    }, ensure_ascii=False))
    if exit_code != 0:
        print("J5 TIMEOUT SETUP FAILED: %s (audit: %s)"
              % (setup.evidence["error"], setup.evidence["audit_path"]), file=sys.stderr)
    return exit_code


def main():
    sys.exit(run())


if __name__ == "__main__":
    main()
