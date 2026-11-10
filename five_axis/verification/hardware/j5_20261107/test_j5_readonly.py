"""J5 read-only discovery/calibration tests in isolated PTY; no real port."""
import json
import math
import os
import pty
import struct
import tempfile
import threading
import time
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
from j5_readonly import (J5ManualCalibration, J5Reader, feedback_frame,
                         usb_packet)
from j5_calibration import load_j5_calibration

PMAX = 12.5
VMAX = 25.0
TMAX = 5.0
MASTER = 0x10


def pack_register(rid, value):
    data = bytes([5, 0, 0x33, rid]) + (struct.pack("<f", value)
                                          if rid in (21, 22, 23) else struct.pack("<I", int(value)))
    return feedback_frame(0, data)


def pack_feedback(position):
    p = int(round((position + PMAX) * 65535 / (2 * PMAX)))
    v = 2048
    t = 2048
    data = bytes([(0 << 4) | 5, p >> 8, p & 255, v >> 4,
                  ((v & 15) << 4) | (t >> 8), t & 255, 30, 28])
    return feedback_frame(MASTER, data)


class FakeJ5:
    def __init__(self, master_fd, position=0.0):
        self.fd = master_fd
        self.position = position
        self.writes = []
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while self.running:
            try:
                chunk = os.read(self.fd, 64)
            except OSError:
                break
            if not chunk:
                break
            self.writes.extend(chunk)
            while len(chunk) >= 30:
                packet, chunk = chunk[:30], chunk[30:]
                request = packet[21:29]
                if request[0] != 5:
                    continue
                if request[2] == 0x33:
                    rid = request[3]
                    values = {7: MASTER, 8: 5, 9: 2000, 10: 1,
                              21: PMAX, 22: VMAX, 23: TMAX}
                    if rid in values:
                        os.write(self.fd, pack_register(rid, values[rid]))
                elif request[2] == 0xCC:
                    os.write(self.fd, pack_feedback(self.position))

    def stop(self):
        self.running = False
        try:
            os.write(self.fd, b"")
        except OSError:
            pass


def wait_position(reader, target, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        reader.poll()
        snapshot = reader.snapshot()
        if snapshot["fresh"] and snapshot["feedback"] and \
                abs(snapshot["feedback"]["raw_position_rad"] - target) < .01:
            return snapshot
        time.sleep(.02)
    raise AssertionError("J5 fake position did not update: %r" % reader.snapshot())


def wait_ready(reader, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        reader.poll()
        snapshot = reader.snapshot()
        if snapshot["ready"] and snapshot["fresh"]:
            return snapshot
        time.sleep(.02)
    raise AssertionError("J5 fake feedback did not become ready: %r" % reader.snapshot())


def main():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fake = FakeJ5(master)
    evidence_dir = Path(__file__).resolve().parent / "evidence"
    evidence_dir.mkdir(exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix="j5-readonly-"))
    reader = J5Reader(path, audit_path=tmp / "readonly.jsonl")
    report = {"passed": False, "physical_port_opened": False, "pty": path}
    try:
        snapshot = wait_ready(reader)
        assert snapshot["profile"]["pmax_rad"] == PMAX
        assert snapshot["profile"]["model_verified"] is False
        assert snapshot["hardware_sign"] is None and snapshot["zero_offset_rad"] is None
        assert snapshot["limits"] is None
        assert snapshot["profile"]["can_id"] == 5
        assert snapshot["profile"]["master_id"] == MASTER
        assert "single-turn" in snapshot["encoder_representation"]
        report["profile"] = snapshot["profile"]

        calibration = J5ManualCalibration(tmp / "j5_calibration.json",
                                          tmp / "j5_calibration_audit.json")
        zero = calibration.capture_zero(reader.snapshot(), confirm_disabled_support=True)
        assert zero["zero"] is True
        # Light manual rotation, no command. +10 deg.
        fake.position = math.radians(10)
        wait_position(reader, math.radians(10))
        direction = calibration.capture_direction(reader.snapshot(),
                                                  confirm_manual_rotation=True)
        assert direction["direction"]["sign"] == 1
        # Cable-limited travel endpoints, operator-confirmed.
        fake.position = math.radians(-35)
        wait_position(reader, math.radians(-35))
        calibration.capture_limit("min", reader.snapshot(), confirm_cable_limit=True)
        fake.position = math.radians(45)
        wait_position(reader, math.radians(45))
        calibration.capture_limit("max", reader.snapshot(), confirm_cable_limit=True)
        saved = calibration.save(confirm_save=True, notes="isolated PTY manual calibration")
        record = load_j5_calibration(tmp / "j5_calibration.json", required=True)
        assert saved["saved"] and record.is_usable()
        assert record.sign == 1
        assert record.zero_payload and record.positive_payload
        assert record.travel_min_payload and record.travel_max_payload
        audit = json.loads((tmp / "j5_calibration_audit.json").read_text(encoding="utf-8"))
        assert [event["step"] for event in audit[:5]] == [
            "zero", "direction", "limit_min", "limit_max", "save"]
        assert audit[-1]["extra"]["no_zero_or_parameter_write"] is True
        # Every write is a read request (0x33/0xCC), never a MIT frame.
        for offset in range(0, len(fake.writes), 30):
            payload = fake.writes[offset + 21:offset + 29]
            assert payload[0] == 5 and payload[2] in (0x33, 0xCC), payload.hex()
        report.update({"passed": True, "record": record.to_dict(),
                       "audit_path": str(tmp / "j5_calibration_audit.json"),
                       "readonly_path": str(tmp / "readonly.jsonl"),
                       "tx": reader.tx, "rx": reader.rx,
                       "commands_sent": "only register/feedback read requests",
                       "zero_offset_rad": record.zero_offset_rad,
                       "hardware_sign": record.sign})
        (evidence_dir / "j5_readonly_pty.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8")
        print("PASS", json.dumps(report, ensure_ascii=False))
    finally:
        reader.close()
        fake.stop()
        try:
            os.close(master)
            os.close(slave)
        except OSError:
            pass


if __name__ == "__main__":
    main()
