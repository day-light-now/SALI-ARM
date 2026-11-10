"""PTY-only acceptance tests for the one-shot J5 TIMEOUT setup tool.

No real serial/CAN/USB device is opened and no real setup is claimed: every
test runs against a generated PTY and an in-process fake five-axis emulator.
The fake port path is /dev/pts/N, so the tool's absolute /dev/ requirement is
exercised for real rather than bypassed.
"""
import contextlib
import datetime
import fcntl
import io
import json
import os
import pty
import select
import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import j5_timeout_setup as tool  # noqa: E402
from j5_readonly import J5_ID, REGS, feedback_frame, usb_packet  # noqa: E402

CONFIRM = tool.CONFIRM_TOKEN
EXPECTED_WRITE = bytes.fromhex("05 00 55 09 d0 07 00 00")
EXPECTED_FRAME_HEX = tool.parameter_write_frame().hex(" ")
FAST = {"PREFLIGHT_TIMEOUT_S": 0.3, "POST_TIMEOUT_S": 0.3,
        "SEND_INTERVAL_S": 0.0005, "CYCLE_INTERVAL_S": 0.004,
        "DRAIN_WINDOW_S": 0.004}


class FakeFiveAxis:
    """Generated-PTY J1-J5 emulator; records every host 30-byte frame."""

    def __init__(self, master_fd):
        self.master = master_fd
        self.registers = {J5_ID: {7: 0, 8: J5_ID, 9: 0, 10: 1,
                                  21: 12.5, 22: 10.0, 23: 28.0}}
        self.motor_states = {axis: 0 for axis in (1, 2, 3, 4, 5)}
        self.suppressed = set()
        self.register_id = J5_ID
        self.conflict_9 = False
        self._toggle = 0
        self.apply_parameter_writes = True
        self.mode_change_on_write = None
        self.frames = []
        self.buffer = bytearray()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                ready, _, _ = select.select([self.master], [], [], 0.02)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            try:
                chunk = os.read(self.master, 4096)
            except OSError:
                continue
            if not chunk:
                return
            self.buffer.extend(chunk)
            while len(self.buffer) >= 30:
                frame = bytes(self.buffer[:30])
                del self.buffer[:30]
                self._handle(frame)

    def _handle(self, frame):
        can_id = int.from_bytes(frame[13:15], "little")
        data = frame[21:29]
        self.frames.append((can_id, data))
        if can_id != 0x7FF:
            return
        if data[2] == 0xCC:
            axis = data[0]
            if axis in self.motor_states and axis not in self.suppressed:
                self._send_feedback(axis)
        elif data[2] == 0x33:
            if data[0] != J5_ID or data[3] not in self.registers[J5_ID]:
                return
            rid = data[3]
            value = self._value(rid)
            payload = bytes([self.register_id, 0, 0x33, rid]) + (
                struct.pack("<f", value) if rid in tool.FLOAT_REGISTERS
                else struct.pack("<I", int(value)))
            self._send(feedback_frame(0, payload))
        elif data[2] == 0x55:
            motor = data[0] | (data[1] << 8)
            rid = data[3]
            if motor != J5_ID or rid != tool.TIMEOUT_RID:
                return
            if self.apply_parameter_writes:
                self.registers[J5_ID][rid] = struct.unpack("<I", data[4:])[0]
            if self.mode_change_on_write is not None:
                self.registers[J5_ID][10] = self.mode_change_on_write

    def _value(self, rid):
        if rid == tool.TIMEOUT_RID and self.conflict_9:
            self._toggle ^= 1
            return 0 if self._toggle else 1
        return self.registers[J5_ID][rid]

    def _send_feedback(self, axis):
        state = self.motor_states[axis]
        raw = int(round((0.0 + 12.5) * 65535 / 25.0))
        velocity = 2048
        torque = 2048
        payload = bytes([(state << 4) | axis, raw >> 8, raw & 255, velocity >> 4,
                         ((velocity & 15) << 4) | (torque >> 8), torque & 255, 30, 28])
        self._send(feedback_frame(0, payload))

    def _send(self, raw):
        try:
            os.write(self.master, raw)
        except OSError:
            pass

    def parameter_writes(self):
        return [(can_id, data) for can_id, data in self.frames if data[2] == 0x55]

    def stop(self):
        self.running = False
        self.thread.join(timeout=1)


class PtyCase(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.tempdir = tempfile.TemporaryDirectory(prefix="j5-timeout-")
        self.audit = Path(self.tempdir.name) / "evidence" / "j5_timeout_setup_evidence.json"
        self.device = FakeFiveAxis(self.master)
        self.patches = [mock.patch.object(tool, name, value)
                        for name, value in FAST.items()]
        for patch in self.patches:
            patch.start()
        self.last_stdout = ""
        self.last_stderr = ""

    def tearDown(self):
        for patch in self.patches:
            patch.stop()
        self.device.stop()
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        self.tempdir.cleanup()

    def argv(self, port=None, apply_timeout=2000, confirm=CONFIRM, audit=None):
        return ["--port", self.port if port is None else port,
                "--apply-timeout", str(apply_timeout),
                "--confirm", confirm,
                "--audit", str(self.audit if audit is None else audit)]

    def run_tool(self, argv=None):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = tool.run(self.argv() if argv is None else argv)
        self.last_stdout, self.last_stderr = stdout.getvalue(), stderr.getvalue()
        return code

    def evidence(self):
        return json.loads(self.audit.read_text(encoding="utf-8"))

    def assert_no_io(self):
        self.assertFalse(self.audit.exists())
        self.assertEqual(self.device.frames, [])


class FrameLayoutTest(unittest.TestCase):
    """The tool must keep the reviewed official 30-byte USB framing."""

    def test_reviewed_usb_layout_is_reused(self):
        self.assertEqual(len(tool.USB_FRAME), 30)
        self.assertEqual(tool.USB_FRAME[:3], bytes([0x55, 0xAA, 0x1E]))
        self.assertEqual(tool.USB_FRAME[13:15], b"\xff\x07")
        self.assertEqual(tool.USB_FRAME[18], 8)
        self.assertEqual(tool.request_frame(J5_ID), usb_packet(J5_ID))
        for rid in REGS:
            self.assertEqual(tool.request_frame(J5_ID, rid), usb_packet(J5_ID, rid))
        for axis in (1, 2, 3, 4):
            frame = tool.request_frame(axis)
            self.assertEqual(frame[13:15], b"\xff\x07")
            self.assertEqual(frame[21:29], bytes([axis, 0, 0xCC, 0, 0, 0, 0, 0]))
        with self.assertRaises(tool.SetupError):
            tool.usb_frame(b"\x00" * 7)

    def test_parameter_write_frame_is_official_damiao_write(self):
        frame = tool.parameter_write_frame()
        self.assertEqual(len(frame), 30)
        self.assertEqual(frame[13:15], b"\xff\x07")
        self.assertEqual(frame[21:29], EXPECTED_WRITE)
        self.assertEqual(tool.parameter_write_frame().hex(" "), EXPECTED_FRAME_HEX)
        self.assertEqual(tool.PARAMETER_WRITE_DATA.hex(" "), "05 00 55 09 d0 07 00 00")


class SuccessTest(PtyCase):
    def test_success_evidence_and_exactly_one_write(self):
        code = self.run_tool()
        self.assertEqual(code, 0, self.last_stderr)
        document = self.evidence()

        self.assertEqual(document["schema"], "slaihand_j5_timeout_setup/1")
        self.assertIs(document["ok"], True)
        self.assertEqual(document["exit_code"], 0)
        self.assertIsNone(document["error"])
        self.assertEqual(document["device"], os.path.realpath(self.port))
        self.assertEqual(document["port_argument"], self.port)
        self.assertTrue(document["device"].startswith("/dev/"))
        self.assertEqual(document["baud"], 921600)
        self.assertIs(document["exclusive"]["locked"], True)
        self.assertIs(document["exclusive"]["released"], True)
        self.assertEqual(document["session_id"], document["session_id"].lower())
        self.assertEqual(len(document["session_id"]), 32)
        self.assertEqual(len(document["tool"]["sha256"]), 64)
        int(document["tool"]["sha256"], 16)
        self.assertTrue(document["host_boot_id"])

        self.assertEqual(document["j5_id"], 5)
        self.assertEqual(document["master_id"], 0)
        self.assertEqual(document["control_mode"], 1)
        self.assertEqual(document["profile"],
                         {"pmax_rad": 12.5, "vmax_rad_s": 10.0, "tmax_nm": 28.0})
        self.assertEqual(document["timeout_before"], 0)
        self.assertEqual(document["timeout_after"], 2000)

        self.assertIs(document["motor_state_all_zero"], True)
        self.assertIs(document["all_motor_state_zero"], True)
        self.assertEqual([row["axis"] for row in document["axes"]],
                         ["J1", "J2", "J3", "J4", "J5"])
        for row in document["axes"]:
            self.assertEqual(row["can_id"], int(row["axis"][1]))
            self.assertEqual(row["motor_state"], 0)
            self.assertIs(row["valid"], True)
            self.assertLessEqual(row["feedback_age"], 0.25)
            self.assertTrue(row["feedback_payload"])

        self.assertEqual(document["write_outcome"], "complete")
        self.assertEqual(document["write_attempts"], 1)
        self.assertEqual(document["parameter_writes"], 1)
        self.assertEqual(document["write_data_hex"], "05 00 55 09 d0 07 00 00")
        self.assertEqual(document["write_frame_hex"], EXPECTED_FRAME_HEX)
        self.assertIs(document["readback"]["matched"], True)
        self.assertEqual(document["readback"]["rid"], 9)
        self.assertEqual(document["readback"]["expected"], 2000)
        self.assertEqual(document["readback"]["observed"], 2000)
        self.assertGreaterEqual(document["readback"]["attempts"], 1)
        self.assertTrue(document["readback_hex"].startswith("05 00 33 09"))

        self.assertIs(document["motor_zero_written"], False)
        self.assertIs(document["flash_written"], False)
        self.assertIs(document["enabled"], False)
        self.assertIs(document["mit_sent"], False)
        self.assertEqual(document["audit_path"], str(self.audit))
        self.assertIs(document["preflight"]["completed"], True)
        self.assertIs(document["postflight"]["completed"], True)
        self.assertEqual(document["postflight"]["registers"]["timeout"], 2000)
        self.assertTrue(document["observed_at"])
        datetime.datetime.fromisoformat(document["observed_at"])
        self.assertTrue(any(event["direction"] == "tx"
                            for event in document["trace"]))
        self.assertTrue(any(event["direction"] == "rx"
                            for event in document["trace"]))

        frames = self.device.frames
        self.assertTrue(frames)
        self.assertTrue(all(can_id == 0x7FF for can_id, _ in frames))
        self.assertEqual({data[0] for _, data in frames if data[2] == 0xCC},
                         {1, 2, 3, 4, 5})
        self.assertEqual({data[3] for _, data in frames if data[2] == 0x33},
                         set(REGS))
        writes = self.device.parameter_writes()
        self.assertEqual(len(writes), 1)
        self.assertEqual(writes[0], (0x7FF, EXPECTED_WRITE))
        # Only the one reviewed 0x55 RID9 write exists: no flash/save, no ID,
        # no zero/enable/MIT (all of those would use a motor CAN ID, not 0x7FF).
        self.assertTrue(all(data[2] in (0x33, 0xCC, 0x55) for _, data in frames))
        self.assertTrue(all(data[2] != 0x55 or data[3] == 9 for _, data in frames))

    def test_second_run_refuses_to_overwrite_committed_evidence(self):
        self.assertEqual(self.run_tool(), 0, self.last_stderr)
        first = self.audit.read_text(encoding="utf-8")
        self.assertEqual(self.run_tool(), 2, self.last_stderr)
        self.assertIn("REFUSED", self.last_stderr)
        self.assertEqual(self.audit.read_text(encoding="utf-8"), first)
        self.assertEqual(len(self.device.parameter_writes()), 1)

    def test_already_configured_device_is_refused_before_any_second_write(self):
        self.assertEqual(self.run_tool(), 0, self.last_stderr)
        other = self.audit.with_name("second_evidence.json")
        code = self.run_tool(argv=self.argv(audit=other))
        self.assertEqual(code, 1, self.last_stderr)
        document = json.loads(other.read_text(encoding="utf-8"))
        self.assertIn("PREFLIGHT_FAILED", document["error"])
        self.assertEqual(document["timeout_before"], 2000)
        self.assertEqual(document["write_attempts"], 0)
        self.assertEqual(len(self.device.parameter_writes()), 1)

    def test_retry_after_prewrite_failure_may_reuse_audit_path(self):
        self.device.registers[J5_ID][10] = 2
        self.assertEqual(self.run_tool(), 1, self.last_stderr)
        failed = self.evidence()
        self.assertEqual(failed["write_attempts"], 0)
        self.assertIs(failed["ok"], False)
        self.device.registers[J5_ID][10] = 1
        self.assertEqual(self.run_tool(), 0, self.last_stderr)
        self.assertIs(self.evidence()["ok"], True)


class RefusalTest(PtyCase):
    def test_refuses_missing_confirm(self):
        argv = ["--port", self.port, "--apply-timeout", "2000",
                "--audit", str(self.audit)]
        with self.assertRaises(SystemExit) as caught:
            with contextlib.redirect_stderr(io.StringIO()):
                tool.run(argv)
        self.assertNotEqual(caught.exception.code, 0)
        self.assert_no_io()

    def test_refuses_missing_port(self):
        argv = ["--apply-timeout", "2000", "--confirm", CONFIRM,
                "--audit", str(self.audit)]
        with self.assertRaises(SystemExit) as caught:
            with contextlib.redirect_stderr(io.StringIO()):
                tool.run(argv)
        self.assertNotEqual(caught.exception.code, 0)
        self.assert_no_io()

    def test_refuses_port_not_under_dev(self):
        for port in ("COM9", "ttyUSB0", "/tmp/fake-tty", "./dev/ttyUSB0"):
            with self.subTest(port=port):
                code = self.run_tool(argv=self.argv(port=port))
                self.assertEqual(code, 2, self.last_stderr)
                self.assertIn("REFUSED", self.last_stderr)
                self.assert_no_io()

    def test_refuses_timeout_other_than_2000(self):
        for value in (0, 1000, 2001, -2000):
            with self.subTest(value=value):
                code = self.run_tool(argv=self.argv(apply_timeout=value))
                self.assertEqual(code, 2, self.last_stderr)
                self.assertIn("REFUSED", self.last_stderr)
                self.assert_no_io()

    def test_refuses_wrong_confirmation_token(self):
        for confirm in ("", "J5_SETUP", "exclusive_j5_setup", "EXCLUSIVE_J5_SETUP "):
            with self.subTest(confirm=confirm):
                code = self.run_tool(argv=self.argv(confirm=confirm))
                self.assertEqual(code, 2, self.last_stderr)
                self.assertIn("REFUSED", self.last_stderr)
                self.assert_no_io()


class PreflightRejectionTest(PtyCase):
    def assert_rejected(self, fragment="PREFLIGHT_FAILED"):
        code = self.run_tool()
        self.assertEqual(code, 1, self.last_stderr)
        document = self.evidence()
        self.assertIs(document["ok"], False)
        self.assertEqual(document["exit_code"], 1)
        self.assertIn(fragment, document["error"])
        self.assertEqual(document["write_attempts"], 0)
        self.assertIsNone(document["write_outcome"])
        self.assertEqual(document["parameter_writes"], 0)
        self.assertEqual(document["read_only_requests"] > 0, True)
        self.assertEqual(self.device.parameter_writes(), [])
        return document

    def test_rejects_wrong_j5_can_id_register(self):
        self.device.registers[J5_ID][8] = 4
        document = self.assert_rejected()
        self.assertEqual(document["preflight"]["mismatched_registers"]["can_id"],
                         {"observed": 4, "expected": 5})

    def test_rejects_register_reply_not_from_id5(self):
        self.device.register_id = 4
        document = self.assert_rejected()
        self.assertIn("can_id", document["preflight"]["missing_registers"])

    def test_rejects_wrong_master_id(self):
        self.device.registers[J5_ID][7] = 0x10
        document = self.assert_rejected()
        self.assertIn("master_id", document["preflight"]["mismatched_registers"])

    def test_rejects_wrong_control_mode(self):
        self.device.registers[J5_ID][10] = 2
        document = self.assert_rejected()
        self.assertIn("control_mode", document["preflight"]["mismatched_registers"])

    def test_rejects_wrong_profile(self):
        self.device.registers[J5_ID][22] = 45.0
        document = self.assert_rejected()
        self.assertIn("vmax_rad_s", document["preflight"]["mismatched_registers"])

    def test_rejects_nonzero_timeout_before_write(self):
        self.device.registers[J5_ID][9] = 2000
        document = self.assert_rejected()
        self.assertIn("timeout", document["preflight"]["mismatched_registers"])
        self.assertEqual(document["timeout_before"], 2000)

    def test_rejects_nonzero_motor_state_on_any_axis(self):
        for axis in (1, 2, 3, 4, 5):
            with self.subTest(axis=axis):
                self.device.motor_states[axis] = 1
                try:
                    document = self.assert_rejected()
                    self.assertIn("J%d" % axis, document["preflight"]["nonzero_axes"])
                finally:
                    self.device.motor_states[axis] = 0

    def test_rejects_stale_feedback(self):
        self.device.suppressed.add(2)
        document = self.assert_rejected()
        self.assertIn("J2", document["preflight"]["stale_axes"])

    def test_rejects_register_conflict(self):
        self.device.conflict_9 = True
        document = self.assert_rejected()
        self.assertTrue(document["preflight"]["conflicts"])

    def test_rejects_already_locked_port(self):
        holder = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        try:
            fcntl.flock(holder, fcntl.LOCK_EX | fcntl.LOCK_NB)
            code = self.run_tool()
            self.assertEqual(code, 1, self.last_stderr)
            document = self.evidence()
            self.assertIn("PORT_BUSY", document["error"])
            self.assertEqual(document["write_attempts"], 0)
            self.assertEqual(self.device.frames, [])
        finally:
            fcntl.flock(holder, fcntl.LOCK_UN)
            os.close(holder)


class FaultyWriter:
    def __init__(self, kind):
        self.kind = kind
        self.calls = 0

    def write(self, packet):
        self.calls += 1
        return {"kind": self.kind,
                "written": 0 if self.kind == "eagain" else 7,
                "waited_s": 0.001}


class WriteFaultTest(PtyCase):
    def test_parameter_write_faults_fail_without_retry(self):
        for kind in ("eagain", "partial", "io_error"):
            with self.subTest(kind=kind):
                setup = tool.TimeoutSetup(self.port, self.audit)
                try:
                    setup.port.open()
                    setup._preflight()
                    writer = FaultyWriter(kind)
                    setup.port.writer = writer
                    with self.assertRaises(tool.SetupError) as caught:
                        setup._write_once()
                    self.assertIn("PARAMETER_WRITE_FAULT", str(caught.exception))
                    self.assertEqual(writer.calls, 1)
                    self.assertEqual(setup.evidence["write_attempts"], 1)
                    self.assertEqual(setup.evidence["parameter_writes"], 0)
                    self.assertEqual(setup.evidence["write_outcome"], kind)
                    self.assertEqual(self.device.parameter_writes(), [])
                finally:
                    setup.port.close()

    def test_read_request_write_fault_fails_before_parameter_write(self):
        setup = tool.TimeoutSetup(
            self.port, self.audit,
            frame_writer_factory=lambda fd, clock: FaultyWriter("eagain"))
        self.assertEqual(setup.execute(), 1)
        document = self.evidence()
        self.assertIn("READ_REQUEST_WRITE_FAULT", document["error"])
        self.assertEqual(document["write_attempts"], 0)
        self.assertEqual(document["parameter_writes"], 0)
        self.assertEqual(self.device.parameter_writes(), [])


class ReadbackTest(PtyCase):
    def test_rejects_readback_mismatch_and_never_retries_write(self):
        self.device.apply_parameter_writes = False
        code = self.run_tool()
        self.assertEqual(code, 1, self.last_stderr)
        document = self.evidence()
        self.assertIs(document["ok"], False)
        self.assertIn("READBACK", document["error"])
        self.assertEqual(document["write_outcome"], "complete")
        self.assertEqual(document["write_attempts"], 1)
        self.assertEqual(document["parameter_writes"], 1)
        self.assertEqual(document["timeout_before"], 0)
        self.assertEqual(document["timeout_after"], 0)
        self.assertIs(document["readback"]["matched"], False)
        self.assertEqual(document["readback"]["observed"], 0)
        self.assertIs(document["motor_state_all_zero"], True)
        self.assertEqual(len(self.device.parameter_writes()), 1)

    def test_rejects_postflight_profile_change_after_committed_write(self):
        self.device.mode_change_on_write = 2
        code = self.run_tool()
        self.assertEqual(code, 1, self.last_stderr)
        document = self.evidence()
        self.assertIn("POSTFLIGHT_MISMATCH", document["error"])
        self.assertEqual(document["write_attempts"], 1)
        self.assertEqual(len(self.device.parameter_writes()), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
