"""J5 read-only discovery and manual calibration, with no enable/motion path.

The module reuses the established J5 protocol facts from slai_console:
- CAN ID 5 (saved and power-cycle verified),
- read registers 7/8/9/10/21/22/23,
- the existing USB2CAN 30-byte request framing and feedback decode.

It deliberately does not guess model/PMAX/VMAX/TMAX, direction, zero or limits.
Registers are shown as raw observations. A calibration record is written only
after the operator explicitly confirms each manual step while the drive is
disabled; no zero/MIT/parameter write command exists in this module.
"""
from __future__ import annotations

import errno
import fcntl
import json
import math
import os
import select
import struct
import termios
import time
import uuid
from pathlib import Path

try:
    from .j5_calibration import build_j5_calibration, load_j5_calibration
except ImportError:  # scripts are also executed directly
    from j5_calibration import build_j5_calibration, load_j5_calibration

J5_ID = 5
REGS = (7, 8, 9, 10, 21, 22, 23)
REGISTER_NAMES = {7: "master_id", 8: "can_id", 9: "timeout", 10: "mode",
                  21: "pmax", 22: "vmax", 23: "tmax"}
DECLARED_MODEL = "DM-J4340-2EC"
MAX_SAMPLE_AGE_S = 0.25
MIN_DIRECTION_DELTA_RAD = math.radians(5.0)  # operator instruction: ~10 degrees
# Match C++ ReadOnlyPort::write_serial: a complete 30-byte write returns at
# once; only a partial/EAGAIN frame waits, with the same 5 ms overall deadline.
FRAME_WRITE_TIMEOUT_S = 0.005
MAX_CONSECUTIVE_WRITE_DEFERRALS = 3


class J5ProtocolError(ValueError):
    pass


class J5TransportError(RuntimeError):
    """A read/write transport fault; the port is closed and a reconnect is required."""


def request_payload(motor_id=J5_ID, rid=None):
    if motor_id != J5_ID or (rid is not None and rid not in REGS):
        raise J5ProtocolError("Only J5 ID 5 and approved read registers are allowed")
    return bytes([motor_id, 0, 0xCC if rid is None else 0x33,
                  0 if rid is None else rid, 0, 0, 0, 0])


def usb_packet(motor_id=J5_ID, rid=None):
    data = request_payload(motor_id, rid)
    frame = bytearray([0x55, 0xAA, 0x1E, 3, 1, 0, 0, 0, 0x0A, 0, 0, 0, 0,
                       0xFF, 7, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    frame[21:29] = data
    return bytes(frame)


def feedback_frame(can_id, data):
    if not 0 <= int(can_id) <= 0x7FF or len(data) != 8:
        raise J5ProtocolError("Invalid 16-byte J5 feedback frame")
    raw = bytearray([0xAA, 0x11, 0x08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x55])
    raw[3:7] = int(can_id).to_bytes(4, "little")
    raw[7:15] = bytes(data)
    return bytes(raw)


def decode_feedback(payload, pmax, vmax, tmax):
    if len(payload) != 8:
        raise J5ProtocolError("J5 feedback must contain exactly 8 bytes")
    if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
               for v in (pmax, vmax, tmax)):
        raise J5ProtocolError("PMAX/VMAX/TMAX must be observed finite positive values")
    position = ((payload[1] << 8) | payload[2]) * (2 * pmax) / 65535 - pmax
    velocity = ((payload[3] << 4) | (payload[4] >> 4)) * (2 * vmax) / 4095 - vmax
    torque = (((payload[4] & 15) << 8) | payload[5]) * (2 * tmax) / 4095 - tmax
    return {"payload": payload.hex(" "), "motor_id": payload[0] & 15,
            "state": payload[0] >> 4, "raw_position_rad": position,
            "raw_position_deg": math.degrees(position),
            "raw_velocity_rad_s": velocity, "raw_torque_nm": torque,
            "driver_temperature_c": payload[6], "motor_temperature_c": payload[7]}


class J5State:
    def __init__(self):
        self.regs = {}
        self.conflicts = set()
        self.feedback = None
        self.feedback_at = None
        self.feedback_can_id = None
        self.notice = ""
        self.rx = 0
        self.tx = 0
        self.encoder_representation = "single-turn raw motor position; no wrap expansion"

    def ready(self):
        return (not self.conflicts and all(rid in self.regs for rid in REGS)
                and self.regs[8] == J5_ID and 0 <= self.regs[7] <= 0x7FF
                and 0 <= self.regs[9] <= 0xFFFFFFFF
                and all(math.isfinite(float(self.regs[rid])) and self.regs[rid] > 0
                        for rid in (21, 22, 23)))

    def feed(self, can_id, data, now):
        if len(data) != 8:
            return
        if data[0] == J5_ID and data[1] == 0 and data[2] in (0x33, 0xCC) and data[3] in REGS:
            rid = data[3]
            if rid in self.regs:
                expected = self.regs[rid]
                value = struct.unpack("<f", data[4:])[0] if rid in (21, 22, 23) \
                    else struct.unpack("<I", data[4:])[0]
                if value != expected:
                    self.conflicts.add(rid)
                    self.feedback = None
                    self.feedback_at = None
                    self.notice = "Same J5 register returned conflicting values; duplicate CAN ID or stale device?"
            else:
                self.regs[rid] = struct.unpack("<f", data[4:])[0] \
                    if rid in (21, 22, 23) else struct.unpack("<I", data[4:])[0]
            return
        if data[0] & 15 != J5_ID or can_id != self.regs.get(7):
            return
        if not self.ready():
            return
        self.feedback = decode_feedback(data, self.regs[21], self.regs[22], self.regs[23])
        self.feedback["can_id"] = int(can_id)
        self.feedback_at = now
        self.feedback_can_id = int(can_id)

    def snapshot(self, now):
        ready = self.ready()
        fresh = bool(ready and self.feedback is not None and self.feedback_at is not None
                     and 0 <= now - self.feedback_at <= MAX_SAMPLE_AGE_S)
        profile = None
        if ready:
            profile = {"model_declared": DECLARED_MODEL, "model_verified": False,
                       "model_source": "project record only; register model string not independently confirmed",
                       "pmax_rad": self.regs[21], "vmax_rad_s": self.regs[22],
                       "tmax_nm": self.regs[23], "master_id": self.regs[7],
                       "can_id": self.regs[8], "timeout": self.regs[9], "mode": self.regs[10]}
        return {"available": True, "connected": self.feedback is not None,
                "ready": ready, "fresh": fresh,
                "age_s": None if self.feedback_at is None else round(max(0.0, now - self.feedback_at), 4),
                "sample_at": self.feedback_at,
                "can_id": J5_ID, "registers": {REGISTER_NAMES[k]: v for k, v in self.regs.items()},
                "profile": profile, "feedback": self.feedback,
                "hardware_sign": None, "zero_offset_rad": None,
                "limits": None, "calibrated": False,
                "encoder_representation": self.encoder_representation,
                "notice": self.notice, "conflicts": sorted(self.conflicts),
                "tx": self.tx, "rx": self.rx,
                "authorized": False, "motion_authorized": False}


class J5FrameWriter:
    """Write one complete 30-byte frame with C++-equivalent completion semantics.

    pyserial 3.5's Serial.write calls select() for writability even after the
    final os.write() accepted all bytes; a slow output queue can therefore raise
    SerialTimeoutException for a frame that was fully submitted. This writer
    returns immediately once all 30 bytes were accepted. A partial frame may
    wait only until a bounded 5 ms deadline; EAGAIN with zero bytes accepted is
    reported as deferred and is never retried as a possibly-sent frame.
    """

    def __init__(self, fd, clock=time.monotonic, timeout_s=FRAME_WRITE_TIMEOUT_S,
                 write_fn=None, select_fn=None):
        self.fd = fd
        self.clock = clock
        self.timeout_s = timeout_s
        self.write_fn = write_fn or os.write
        self.select_fn = select_fn or select.select

    def write(self, packet):
        started = self.clock()
        deadline = started + self.timeout_s
        sent = 0
        while sent < len(packet):
            # Positive progress does not extend the total deadline. Check
            # before every continuation; the first write is still allowed to
            # make progress, and a completed frame returns at once.
            if sent > 0 and self.clock() >= deadline:
                return {"kind": "partial", "written": sent,
                        "waited_s": round(self.clock() - started, 6)}
            try:
                count = self.write_fn(self.fd, packet[sent:])
            except OSError as error:
                if error.errno not in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR,
                                       errno.EALREADY, errno.EINPROGRESS):
                    return {"kind": "io_error", "written": sent,
                            "error": str(error),
                            "waited_s": round(self.clock() - started, 6)}
                count = 0
            if count is None or count < 0:
                count = 0
            if count > 0:
                sent += count
                # Complete submission returns immediately, exactly like C++.
                if sent == len(packet):
                    return {"kind": "complete", "written": sent,
                            "waited_s": round(self.clock() - started, 6)}
                continue
            remaining = deadline - self.clock()
            if remaining <= 0:
                return {"kind": "partial" if sent else "eagain",
                        "written": sent,
                        "waited_s": round(self.clock() - started, 6)}
            try:
                ready = self.select_fn([], [self.fd], [], remaining)
            except OSError as error:
                return {"kind": "io_error", "written": sent,
                        "error": str(error),
                        "waited_s": round(self.clock() - started, 6)}
            if not ready[1]:
                return {"kind": "partial" if sent else "eagain",
                        "written": sent,
                        "waited_s": round(self.clock() - started, 6)}
            # select can report writable while the driver still returns EAGAIN;
            # the deadline above bounds this loop, so it cannot spin forever.
        return {"kind": "complete", "written": sent,
                "waited_s": round(self.clock() - started, 6)}


class J5Reader:
    """Read-only polling reader. Only register/feedback requests are sent."""

    def __init__(self, port, serial_factory=None, clock=time.monotonic, audit_path=None,
                 frame_writer=None):
        self.state = J5State()
        self.clock = clock
        self.frame_writer = frame_writer
        # Resolve symlinks before opening so aliases share the same device lock.
        self.port = os.path.realpath(str(port))
        self.session_id = uuid.uuid4().hex
        self.audit_path = Path(audit_path) if audit_path else None
        self.audit_stream = None
        self.tx = 0
        self.rx = 0
        self.stream = bytearray()
        self.setup = [(5, rid) for _ in range(2) for rid in REGS]
        self.next_send = 0.0
        self.serial = None
        self._locked = False
        self.closed = False
        self.fault = None
        self.frames_sent = 0
        self.last_rx_at = None
        self.last_tx_at = None
        self.write_deferred = 0
        self.last_write_result = None
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        try:
            self.serial = serial_factory(self.port, 921600, timeout=0, write_timeout=.05)
            fd = self.serial.fileno()
            # Match a native exclusive reader: flock on the device fd plus the
            # kernel tty TIOCEXCL flag. Both happen before reset/read/write.
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.ioctl(fd, termios.TIOCEXCL)
            except OSError:
                self._close_serial()
                raise J5ProtocolError(
                    "J5 serial is already owned by another process or lacks exclusive tty support")
            self._locked = True
            if self.frame_writer is None:
                # Use the fd directly so a complete submission does not wait for
                # writability again (pyserial's post-write select is the bug).
                os.set_blocking(fd, False)
                self.frame_writer = J5FrameWriter(fd, clock=self.clock)
            try:
                self.serial.reset_input_buffer()
            except AttributeError:
                pass
            if self.audit_path is not None:
                self.audit_path.parent.mkdir(parents=True, exist_ok=True)
                self.audit_stream = self.audit_path.open("a", encoding="utf-8")
                self._audit("open", 0x7FF,
                            "canonical=%s session=%s flock=LOCK_EX|LOCK_NB tiocm=TIOCEXCL"
                            % (self.port, self.session_id), self.clock())
        except Exception:
            self._close_serial()
            if self.audit_stream is not None:
                self.audit_stream.close()
                self.audit_stream = None
            raise

    def _audit(self, direction, can_id, payload, now):
        if self.audit_stream is None:
            return
        self.audit_stream.write(json.dumps(
            {"at": now, "wall": time.time(), "direction": direction,
             "can_id": int(can_id), "payload": payload}, ensure_ascii=False) + "\n")
        self.audit_stream.flush()

    def _read_available(self):
        try:
            count = self.serial.in_waiting
        except AttributeError:
            count = 0
        return self.serial.read(count)

    def _transport_fields(self):
        if self.fault is not None:
            state = "fault"
        elif self.closed or self.serial is None:
            state = "closed"
        else:
            state = "online"
        return {"state": state, "fault": self.fault,
                "session_id": self.session_id, "device_path": self.port,
                "tx": self.tx, "rx": self.rx, "frames_sent": self.frames_sent,
                "last_rx_at": self.last_rx_at, "last_tx_at": self.last_tx_at,
                "write_deferred": self.write_deferred,
                "last_write": self.last_write_result,
                "reconnect_required": self.fault is not None or self.closed}

    def _fail(self, kind, message, detail=None):
        if self.fault is None:
            now = self.clock()
            fault = {"kind": kind, "message": str(message), "at": time.time(),
                     "frames_sent": self.frames_sent,
                     "last_rx_age_s": None if self.last_rx_at is None
                     else round(max(0.0, now - self.last_rx_at), 4),
                     "last_tx_age_s": None if self.last_tx_at is None
                     else round(max(0.0, now - self.last_tx_at), 4),
                     "detail": detail or {}}
            self.fault = fault
            self._audit("fault", 0x7FF, json.dumps(fault, ensure_ascii=False), now)
        self.closed = True
        self._close_serial()
        if self.audit_stream is not None:
            self.audit_stream.close()
            self.audit_stream = None

    def poll(self, now=None):
        if self.closed or self.serial is None:
            raise J5TransportError("J5 serial is closed")
        now = self.clock() if now is None else now
        try:
            chunk = self._read_available()
            if chunk:
                self.stream.extend(chunk)
                while self.stream:
                    if self.stream[0] != 0xAA:
                        del self.stream[0]
                        continue
                    if len(self.stream) < 16:
                        break
                    if self.stream[1] != 0x11 or self.stream[2] != 8 or self.stream[15] != 0x55:
                        del self.stream[0]
                        continue
                    raw = bytes(self.stream[:16])
                    del self.stream[:16]
                    can_id = int.from_bytes(raw[3:7], "little")
                    data = raw[7:15]
                    self.rx += 1
                    self.state.rx += 1
                    self.last_rx_at = now
                    self._audit("rx", can_id, data.hex(" "), now)
                    self.state.feed(can_id, data, now)
            if now < self.next_send:
                return
            # Do not pop a pending discovery request before it is accepted.
            # A zero-byte EAGAIN must leave it at the head for a later poll;
            # only a complete frame consumes it. A partially submitted frame
            # is never retried (the transport is closed below).
            while self.setup and self.setup[0][1] in self.state.regs:
                self.setup.pop(0)
            rid = self.setup[0][1] if self.setup else None
            packet = usb_packet(J5_ID, rid)
            outcome = self.frame_writer.write(packet)
            self.last_write_result = outcome
            kind = outcome.get("kind")
            if kind == "complete":
                if rid is not None and self.setup and self.setup[0][1] == rid:
                    self.setup.pop(0)
                self.write_deferred = 0
                self.tx += 1
                self.state.tx += 1
                self.frames_sent += 1
                self.last_tx_at = now
                self._audit("tx", 0x7FF, packet[21:29].hex(" "), now)
                self.next_send = now + .05
                return
            if kind in ("eagain", "partial"):
                detail = {"outcome": outcome, "request": packet[21:29].hex(" ")}
                self.write_deferred += 1
                self._audit("write_deferred" if kind == "eagain" else "partial_write",
                            0x7FF, json.dumps(detail, ensure_ascii=False), now)
                if kind == "partial":
                    self._fail("PARTIAL_WRITE",
                               "J5 request accepted only %r/%d bytes; frame abandoned"
                               % (outcome.get("written"), len(packet)), detail)
                    raise J5TransportError(
                        "PARTIAL_WRITE: frame boundary abandoned; port closed, reconnect required")
                # No bytes of this request were accepted. Retain and requeue
                # the discovery request so another register can make progress;
                # a later poll may retry it. A frame that was partially or
                # fully submitted is never retried.
                if rid is not None and self.setup and self.setup[0][1] == rid:
                    self.setup.append(self.setup.pop(0))
                self.next_send = now + .05
                if self.write_deferred >= MAX_CONSECUTIVE_WRITE_DEFERRALS:
                    self._fail("WRITE_BACKPRESSURE",
                               "J5 write remained non-writable with zero bytes accepted "
                               "for %d attempts" % self.write_deferred, detail)
                    raise J5TransportError(
                        "WRITE_BACKPRESSURE: no bytes accepted; port closed, reconnect required")
                return
            self._fail("WRITE_ERROR", "J5 frame write failed: %r" % (outcome,),
                       {"outcome": outcome, "request": packet[21:29].hex(" ")})
            raise J5TransportError(
                "WRITE_ERROR: %r; port closed, reconnect required" % (outcome,))
        except J5TransportError:
            raise
        except Exception as error:
            self._fail("READ_OR_WRITE_FAULT", "%s: %s" % (type(error).__name__, error),
                       {"exception": type(error).__name__})
            raise J5TransportError(
                "READ_OR_WRITE_FAULT: %s; port closed, reconnect required" % error) from error

    def snapshot(self):
        snapshot = self.state.snapshot(self.clock())
        snapshot["session_id"] = self.session_id
        snapshot["device_path"] = self.port
        snapshot["transport"] = self._transport_fields()
        if self.fault is not None or self.closed:
            # A closed/faulted transport must never present the last angle as
            # live data. Keep registers/profile for diagnostics but make all
            # liveness fields explicitly false.
            snapshot["connected"] = False
            snapshot["ready"] = False
            snapshot["fresh"] = False
            snapshot["feedback"] = None
            snapshot["age_s"] = None
            snapshot["sample_at"] = None
        return snapshot

    def _close_serial(self):
        serial = getattr(self, "serial", None)
        if serial is None:
            return
        try:
            fd = serial.fileno()
            if getattr(self, "_locked", False):
                try:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                except OSError:
                    pass
                try:
                    fcntl.flock(fd, fcntl.LOCK_UN)
                except OSError:
                    pass
        except (OSError, AttributeError):
            pass
        try:
            serial.close()
        finally:
            self.serial = None
            self._locked = False

    def close(self):
        self.closed = True
        try:
            self._close_serial()
        finally:
            if self.audit_stream is not None:
                self.audit_stream.close()
                self.audit_stream = None


class J5ManualCalibration:
    """Sequential operator-confirmed manual calibration; no hardware write."""

    def __init__(self, record_path, audit_path=None):
        self.record_path = Path(record_path)
        self.audit_path = Path(audit_path) if audit_path else self.record_path.with_name(
            self.record_path.stem + "_audit.json")
        self.zero = None
        self.direction = None
        self.limit_min = None
        self.limit_max = None
        self.notes = ""
        self.events = []
        self.generation = 0
        self.frozen_identity = None
        self.last_sample_at = None
        self.saved = False
        self.invalidated_reason = None

    def invalidate(self, reason):
        """Drop all calibration state after a transport/session identity break."""
        self.zero = None
        self.direction = None
        self.limit_min = None
        self.limit_max = None
        self.frozen_identity = None
        self.last_sample_at = None
        self.saved = False
        self.generation += 1
        self.invalidated_reason = str(reason)
        self._audit("invalidate", None, {"reason": str(reason)})

    @staticmethod
    def _identity(sample):
        profile = sample.get("profile") or {}
        feedback = sample.get("feedback") or {}
        return {
            "session_id": sample.get("session_id"),
            "device_path": sample.get("device_path"),
            "can_id": profile.get("can_id"),
            "master_id": profile.get("master_id"),
            "mode": profile.get("mode"),
            "timeout": profile.get("timeout"),
            "pmax_rad": profile.get("pmax_rad"),
            "vmax_rad_s": profile.get("vmax_rad_s"),
            "tmax_nm": profile.get("tmax_nm"),
            "model_declared": profile.get("model_declared"),
            "model_verified": profile.get("model_verified"),
            "decode_profile": {
                "pmax_rad": profile.get("pmax_rad"),
                "vmax_rad_s": profile.get("vmax_rad_s"),
                "tmax_nm": profile.get("tmax_nm"),
            },
            "first_payload": feedback.get("payload"),
            "last_payload": feedback.get("payload"),
            "sample_at": sample.get("sample_at"),
            "baud": 921600,
        }

    def _check_identity(self, sample):
        if self.frozen_identity is None:
            raise J5ProtocolError("Capture zero first to freeze device/decode identity")
        current = self._identity(sample)
        frozen = self.frozen_identity
        for key in ("session_id", "device_path", "can_id", "master_id", "mode",
                    "timeout", "pmax_rad", "vmax_rad_s", "tmax_nm"):
            if current.get(key) != frozen.get(key):
                raise J5ProtocolError(
                    "DEVICE_IDENTITY_CHANGED: %s changed (%r -> %r); reconnect/re-zero and "
                    "restart this calibration" % (key, frozen.get(key), current.get(key)))
        current["last_payload"] = (sample.get("feedback") or {}).get("payload")
        return current

    def _advance_sample(self, sample):
        at = sample.get("sample_at")
        if not isinstance(at, (int, float)) or not math.isfinite(at):
            raise J5ProtocolError("SAMPLE_TIME_MISSING: J5 sample has no timestamp")
        if self.last_sample_at is not None and at <= self.last_sample_at:
            raise J5ProtocolError(
                "SAMPLE_TIME_REGRESSED: captures are not in increasing order; restart calibration")
        self.last_sample_at = at

    def _audit(self, step, sample, extra=None):
        event = {"at": time.time(), "generation": self.generation, "step": step,
                 "sample": sample, "extra": extra or {}}
        self.events.append(event)
        self.audit_path.parent.mkdir(parents=True, exist_ok=True)
        self.audit_path.write_text(json.dumps(self.events, ensure_ascii=False, indent=2),
                                   encoding="utf-8")

    def status(self):
        return {"available": True, "zero": self.zero is not None,
                "direction": self.direction,
                "limits": None if self.limit_min is None or self.limit_max is None
                else [self.limit_min["feedback"]["raw_position_rad"],
                      self.limit_max["feedback"]["raw_position_rad"]],
                "saved": self.saved, "record_path": str(self.record_path),
                "audit_path": str(self.audit_path),
                "generation": self.generation,
                "frozen_identity": self.frozen_identity,
                "invalidated_reason": self.invalidated_reason,
                "requires": "disabled + support, current zero, manual +10deg direction, cable-limited travel; no motor commands",
                "zero_offset_rad": None, "hardware_sign": None}

    @staticmethod
    def _validate(sample, now, *, need_disabled=True):
        if not isinstance(sample, dict):
            raise J5ProtocolError("No J5 sample")
        if not sample.get("fresh"):
            raise J5ProtocolError("J5 feedback is stale; keep the drive disabled and retry")
        feedback = sample.get("feedback") or {}
        if not isinstance(feedback, dict):
            raise J5ProtocolError("J5 feedback payload missing")
        if need_disabled and feedback.get("state") != 0:
            raise J5ProtocolError("J5 drive is not disabled (state=%s); do not continue" % feedback.get("state"))
        return sample

    def capture_zero(self, sample, *, confirm_disabled_support):
        if confirm_disabled_support is not True:
            raise J5ProtocolError("Explicit disabled/support confirmation is required")
        self._validate(sample, time.monotonic())
        self.last_sample_at = None
        self._advance_sample(sample)
        # A new zero invalidates all downstream direction/travel captures.
        self.direction = None
        self.limit_min = None
        self.limit_max = None
        self.generation += 1
        self.zero = dict(sample)
        self.frozen_identity = self._identity(sample)
        self.saved = False
        self._audit("zero", self.zero, {"confirm_disabled_support": True,
                                        "cleared_direction_and_limits": True})
        return self.status()

    def capture_direction(self, sample, *, confirm_manual_rotation):
        if self.zero is None:
            raise J5ProtocolError("Capture the current zero before direction")
        if confirm_manual_rotation is not True:
            raise J5ProtocolError("Explicit manual rotation confirmation is required")
        self._validate(sample, time.monotonic())
        current = self._check_identity(sample)
        self._advance_sample(sample)
        zero_pos = self.zero["feedback"]["raw_position_rad"]
        pos = sample["feedback"]["raw_position_rad"]
        delta = pos - zero_pos
        if abs(delta) < MIN_DIRECTION_DELTA_RAD - 1e-12:
            raise J5ProtocolError("Rotate J5 a little more (~10deg) before confirming direction")
        if abs(delta) > math.radians(60.0):
            raise J5ProtocolError(
                "SINGLE_TURN_BOUNDARY_OR_JUMP: raw delta %.3f rad is too large for a small manual "
                "rotation; return J5 to the same single-turn representation branch, re-zero and retry. "
                "No automatic turn unwrapping is enabled." % delta)
        self.limit_min = None
        self.limit_max = None
        self.generation += 1
        self.frozen_identity = dict(self.frozen_identity,
                                    last_payload=(sample.get("feedback") or {}).get("payload"))
        self.saved = False
        self.direction = {"sign": 1 if delta > 0 else -1,
                          "raw_delta_rad": delta,
                          "raw_position_rad": pos,
                          "generation": self.generation,
                          "sample": dict(sample)}
        self._audit("direction", sample,
                    {"confirm_manual_rotation": True, "raw_delta_rad": delta,
                     "sign": self.direction["sign"], "cleared_limits": True})
        return self.status()

    def capture_limit(self, which, sample, *, confirm_cable_limit):
        if self.direction is None:
            raise J5ProtocolError("Confirm direction before travel limits")
        if which not in ("min", "max"):
            raise J5ProtocolError("limit must be min or max")
        if confirm_cable_limit is not True:
            raise J5ProtocolError("Explicit cable/mechanical-clearance confirmation is required")
        self._validate(sample, time.monotonic())
        current = self._check_identity(sample)
        self._advance_sample(sample)
        target = dict(sample)
        self.saved = False
        if which == "min":
            self.limit_min = target
        else:
            self.limit_max = target
        self.frozen_identity = dict(self.frozen_identity,
                                    last_payload=(sample.get("feedback") or {}).get("payload"))
        self._audit("limit_" + which, sample, {"confirm_cable_limit": True})
        return self.status()

    def save(self, *, confirm_save, notes=""):
        if confirm_save is not True:
            raise J5ProtocolError("Explicit save confirmation is required")
        if self.zero is None or self.direction is None or self.limit_min is None or self.limit_max is None:
            raise J5ProtocolError("Zero, direction and both travel limits must be captured first")
        if self.direction.get("generation") != self.generation:
            raise J5ProtocolError("STALE_DIRECTION: direction belongs to an older calibration generation")
        zero_at = self.zero.get("sample_at")
        direction_at = (self.direction.get("sample") or {}).get("sample_at")
        if not isinstance(zero_at, (int, float)) or not isinstance(direction_at, (int, float)):
            raise J5ProtocolError("SAMPLE_TIME_MISSING: zero/direction timestamp")
        if direction_at <= zero_at:
            raise J5ProtocolError("SAMPLE_TIME_REGRESSED: direction must follow zero")
        for label, sample in (("limit_min", self.limit_min), ("limit_max", self.limit_max)):
            at = sample.get("sample_at")
            if not isinstance(at, (int, float)):
                raise J5ProtocolError("SAMPLE_TIME_MISSING: %s" % label)
            if at <= direction_at:
                raise J5ProtocolError("SAMPLE_TIME_REGRESSED: %s must follow direction" % label)
        for label, sample in (("zero", self.zero),
                              ("direction", self.direction.get("sample")),
                              ("limit_min", self.limit_min),
                              ("limit_max", self.limit_max)):
            if not sample:
                raise J5ProtocolError("MISSING_CAPTURE: %s sample missing" % label)
            self._check_identity(sample)
        zero = self.zero["feedback"]["raw_position_rad"]
        positive = self.direction["raw_position_rad"]
        lo = self.limit_min["feedback"]["raw_position_rad"]
        hi = self.limit_max["feedback"]["raw_position_rad"]
        record = build_j5_calibration(
            zero_position_rad=zero, positive_position_rad=positive,
            travel_min_position_rad=lo, travel_max_position_rad=hi,
            zero_payload=self.zero["feedback"].get("payload", ""),
            positive_payload=self.direction["sample"]["feedback"].get("payload", ""),
            travel_min_payload=self.limit_min["feedback"].get("payload", ""),
            travel_max_payload=self.limit_max["feedback"].get("payload", ""),
            user_zero_confirmed=True, user_direction_confirmed=True,
            user_limits_confirmed=True, notes=str(notes),
            calibration_generation=self.generation,
            device_identity=self.frozen_identity,
            decode_profile=(self.frozen_identity or {}).get("decode_profile"))
        self.record_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.record_path.with_suffix(self.record_path.suffix + ".tmp")
        temporary.write_text(json.dumps(record.to_dict(), ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
        temporary.replace(self.record_path)
        self.saved = True
        self._audit("save", {"record": record.to_dict()},
                    {"confirm_save": True, "generation": self.generation,
                     "no_zero_or_parameter_write": True})
        return {"saved": True, "record": record.to_dict(), "path": str(self.record_path)}
