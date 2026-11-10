"""Optional J5 read-only/calibration controller for the control-panel web entry.

No motion, enable, zero write or parameter write exists here. The reader is
started only when an explicit J5 port is supplied (isolated PTY in tests); the
default four-axis deployment never opens a second serial port.

A transport fault closes and releases the serial port, invalidates the manual
calibration session, stops polling, and exposes an explicit reconnect action.
Reconnect is read-only and always requires re-zeroing; it never enables motors.
"""
from __future__ import annotations

import threading
from pathlib import Path

from j5_readonly import J5ManualCalibration, J5ProtocolError, J5Reader


class J5Controller:
    def __init__(self, port, record_path, audit_path=None, serial_factory=None,
                 frame_writer=None):
        self.port = str(port)
        self.serial_factory = serial_factory
        self.frame_writer = frame_writer
        self.record_path = Path(record_path)
        self.audit_path = Path(audit_path) if audit_path else self.record_path.with_suffix(
            self.record_path.suffix + ".readonly.jsonl")
        self.calibration = J5ManualCalibration(self.record_path,
                                               self.record_path.with_suffix(
                                                   self.record_path.suffix + ".audit.json"))
        self.lock = threading.Lock()
        # Serializes the whole close/reconnect lifecycle. A normal poll lock is
        # not enough: close must not race a reader swap, and two reconnects must
        # not stop each other's newly opened reader.
        self._lifecycle = threading.Lock()
        self.error = None
        self._closed = False
        self._stop = threading.Event()
        self.reader = self._open_reader()
        self._thread = threading.Thread(target=self._loop, name="j5-readonly", daemon=True)
        self._thread.start()

    def _open_reader(self):
        return J5Reader(self.port, serial_factory=self.serial_factory,
                        audit_path=self.audit_path,
                        frame_writer=self.frame_writer)

    def _fault(self, message):
        try:
            self.calibration.invalidate(message)
        except Exception:
            pass
        if self.reader is not None:
            try:
                self.reader.close()
            except Exception:
                pass
        self.error = str(message)
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            try:
                with self.lock:
                    self.reader.poll()
            except Exception as error:
                with self.lock:
                    self._fault("%s: %s; port released; reconnect and re-zero required"
                                % (type(error).__name__, error))
                return
            self._stop.wait(.05)

    def _snapshot_locked(self):
        if self.reader is None:
            return {"available": False, "connected": False, "fresh": False,
                    "feedback": None, "age_s": None}
        return self.reader.snapshot()

    def status(self):
        with self.lock:
            try:
                snapshot = self._snapshot_locked()
            except Exception as error:
                snapshot = {"available": False, "connected": False, "fresh": False,
                            "feedback": None, "age_s": None,
                            "reason": "%s: %s" % (type(error).__name__, error)}
            calibration = self.calibration.status()
            error = self.error
        faulted = bool(error or snapshot.get("transport", {}).get("state") == "fault"
                       or snapshot.get("transport", {}).get("state") == "closed")
        snapshot["calibration"] = calibration
        snapshot["error"] = error
        snapshot["available"] = True
        snapshot["stage"] = "read_only_manual_calibration"
        snapshot["motion_authorized"] = False
        snapshot["reconnect_available"] = bool(faulted and not self._closed)
        snapshot["zero_offset_rad"] = None
        snapshot["hardware_sign"] = calibration.get("direction", {}).get("sign") \
            if calibration.get("direction") else None
        snapshot["limits"] = calibration.get("limits")
        if faulted:
            # Never present a last angle as live when polling has stopped.
            snapshot["connected"] = False
            snapshot["fresh"] = False
            snapshot["feedback"] = None
            snapshot["age_s"] = None
            snapshot["sample_at"] = None
        return snapshot

    def _sample(self):
        # Called while self.lock is held by a calibration method.
        sample = self._snapshot_locked()
        if not sample.get("connected") or not sample.get("fresh"):
            raise J5ProtocolError(
                "J5 read-only transport is disconnected or stale; reconnect and re-zero required")
        return sample

    def capture_zero(self, confirm_disabled_support):
        with self.lock:
            return self.calibration.capture_zero(
                self._sample(), confirm_disabled_support=confirm_disabled_support)

    def capture_direction(self, confirm_manual_rotation):
        with self.lock:
            return self.calibration.capture_direction(
                self._sample(), confirm_manual_rotation=confirm_manual_rotation)

    def capture_limit(self, which, confirm_cable_limit):
        with self.lock:
            return self.calibration.capture_limit(
                which, self._sample(), confirm_cable_limit=confirm_cable_limit)

    def save(self, confirm_save, notes=""):
        with self.lock:
            return self.calibration.save(confirm_save=confirm_save, notes=notes)

    def reconnect(self):
        """Explicit read-only recovery. Old session/calibration never survives."""
        with self._lifecycle:
            with self.lock:
                if self._closed:
                    raise J5ProtocolError("J5 controller is closed")
                if self.reader is not None and self.reader.serial is not None \
                        and not self.reader.closed and self.error is None:
                    return {"ok": True, "reconnected": False}
                old = self.reader
                old_thread = self._thread
            if old is not None:
                old.close()
            if old_thread is not threading.current_thread() and old_thread.is_alive():
                old_thread.join(timeout=2)
            with self.lock:
                if self._closed:
                    raise J5ProtocolError("J5 controller is closed")
                # Any earlier calibration generation is invalid across reconnect.
                self.calibration.invalidate("transport reconnect; re-zero required")
                try:
                    reader = self._open_reader()
                except Exception as error:
                    self.error = "%s: %s; reconnect failed" % (type(error).__name__, error)
                    self._stop.set()
                    failed = self.error
                else:
                    self.reader = reader
                    self.error = None
                    self._stop.clear()
                    self._thread = threading.Thread(target=self._loop,
                                                    name="j5-readonly", daemon=True)
                    self._thread.start()
                    failed = None
            if failed is not None:
                return {"ok": False, "reconnected": False, "error": failed}
            return {"ok": True, "reconnected": True}

    def close(self):
        # Same lifecycle mutex as reconnect: after close returns, no reconnect
        # can have installed a new reader/thread behind it.
        with self._lifecycle:
            with self.lock:
                if self._closed:
                    return
                self._closed = True
                self._stop.set()
                reader = self.reader
                thread = self._thread
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=2)
            if reader is not None:
                reader.close()
