"""PTY transport-fault injection for the J5 read-only candidate.

The fake USB device is still a PTY peer; the serial wrapper injects write
timeout, partial write, and read disconnect exactly where pyserial would raise.
No real port, CAN frame, MIT write or parameter write is used.
"""
import json
import os
import pty
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(HERE))
from test_j5_readonly import FakeJ5, wait_ready  # noqa: E402
from j5_controller import J5Controller  # noqa: E402
from j5_readonly import J5Reader, J5TransportError  # noqa: E402


class FaultSerial:
    def __init__(self, path, baudrate, timeout=0, write_timeout=.05):
        import serial
        self.serial_module = serial
        self.raw = serial.Serial(path, baudrate, timeout=timeout,
                                 write_timeout=write_timeout)
        self.mode = "normal"
        self.fault_after = 1000
        self.writes = 0
        self.reads = 0
        self.read_checks = 0

    def fileno(self):
        return self.raw.fileno()

    @property
    def in_waiting(self):
        self.read_checks += 1
        if self.mode == "read_disconnect" and self.read_checks >= self.fault_after:
            raise OSError("injected read disconnect")
        return self.raw.in_waiting

    def read(self, count):
        if self.mode == "read_disconnect":
            raise OSError("injected read disconnect")
        return self.raw.read(count)

    def write(self, packet):
        self.writes += 1
        if self.mode == "write_timeout" and self.writes >= self.fault_after:
            raise self.serial_module.SerialTimeoutException("injected write timeout")
        if self.mode == "partial_write" and self.writes >= self.fault_after:
            # Deliberately return a partial count. The reader must close and
            # never append a fresh 30-byte frame after this partial frame.
            return self.raw.write(packet[:7])
        return self.raw.write(packet)

    def reset_input_buffer(self):
        return self.raw.reset_input_buffer()

    def close(self):
        self.raw.close()


class FaultWriter:
    """Test seam for J5FrameWriter outcomes; no pyserial write is used."""
    def __init__(self):
        self.mode = "normal"
        self.fault_after = 1000
        self.writes = 0
        self.outcomes = []
        self.fd = None

    def write(self, packet):
        self.writes += 1
        if self.mode in ("eagain", "write_timeout") and self.writes >= self.fault_after:
            outcome = {"kind": "eagain", "written": 0, "waited_s": .005}
        elif self.mode == "partial_write" and self.writes >= self.fault_after:
            outcome = {"kind": "partial", "written": 7, "waited_s": .005}
        else:
            count = os.write(self.fd, packet) if self.fd is not None else 0
            outcome = ({"kind": "complete", "written": count, "waited_s": 0.0}
                       if count == len(packet) else
                       {"kind": "partial", "written": count, "waited_s": 0.0})
        self.outcomes.append(outcome)
        return outcome


class FaultFactory:
    def __init__(self):
        self.mode = "normal"
        self.fault_after = 1000
        self.instances = []
        self.writer = FaultWriter()
        self.fail_next = False

    def __call__(self, *args, **kwargs):
        if self.fail_next:
            self.fail_next = False
            raise OSError("injected reconnect open failure")
        serial = FaultSerial(*args, **kwargs)
        serial.mode = self.mode
        serial.fault_after = self.fault_after
        self.instances.append(serial)
        self.writer.fd = serial.fileno()
        return serial


def wait_controller(controller, predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        status = controller.status()
        if predicate(status):
            return status
        time.sleep(.03)
    raise AssertionError("J5 controller status timeout: %r" % controller.status())


def new_fixture():
    master, slave = pty.openpty()
    fake = FakeJ5(master)
    return master, slave, fake, os.ttyname(slave)


def close_fixture(master, slave, fake):
    fake.stop()
    try:
        os.close(master)
        os.close(slave)
    except OSError:
        pass


def test_long_normal():
    master, slave, fake, path = new_fixture()
    factory = FaultFactory()
    with tempfile.TemporaryDirectory() as folder:
        controller = J5Controller(path, Path(folder) / "cal.json", serial_factory=factory,
                                  frame_writer=factory.writer)
        try:
            wait_ready_controller = wait_controller(
                controller, lambda s: s.get("fresh") and s.get("connected"))
            assert wait_ready_controller["transport"]["frames_sent"] >= 2
            time.sleep(10.0)
            status = controller.status()
            assert status["error"] is None, status
            assert status["connected"] and status["fresh"]
            assert status["transport"]["frames_sent"] > 150
        finally:
            controller.close()
    close_fixture(master, slave, fake)


def run_fault(mode, after, expect_error):
    master, slave, fake, path = new_fixture()
    factory = FaultFactory()
    with tempfile.TemporaryDirectory() as folder:
        controller = J5Controller(path, Path(folder) / "cal.json", serial_factory=factory,
                                  frame_writer=factory.writer)
        try:
            wait_controller(controller, lambda s: s.get("fresh") and s.get("connected"))
            controller.capture_zero(True)
            # Inject after a healthy handshake/zero so this is a real transport
            # fault, not an inability to complete the initial register discovery.
            serial = factory.instances[-1]
            serial.mode = mode
            serial.fault_after = after
            serial.read_checks = 0
            factory.writer.mode = mode
            factory.writer.fault_after = after
            factory.writer.writes = 0
            status = wait_controller(controller, lambda s: s.get("error") is not None, 10)
            assert expect_error in status["error"], status["error"]
            assert status["connected"] is False and status["fresh"] is False
            assert status["feedback"] is None and status["age_s"] is None
            assert status["transport"]["state"] == "fault"
            assert status["reconnect_available"] is True
            assert status["calibration"]["zero"] is False
            assert status["calibration"]["invalidated_reason"]
            assert os.path.exists(path), "device node disappeared"
            if mode == "partial_write":
                assert any(outcome["kind"] == "partial"
                           for outcome in factory.writer.outcomes), factory.writer.outcomes
            # The old exclusive owner has released the TTY: a fresh reader can open.
            probe = None
            try:
                probe = J5Reader(path)
                assert probe.snapshot()["connected"] in (True, False)
            finally:
                if probe is not None:
                    probe.close()
            # Explicit read-only reconnect, then re-zero is required.
            factory.mode = "normal"
            factory.writer.mode = "normal"
            result = controller.reconnect()
            assert result["ok"] and result["reconnected"], result
            reconnected = wait_controller(controller, lambda s: s.get("fresh") and s.get("connected"))
            assert reconnected["calibration"]["zero"] is False
            assert reconnected["error"] is None
            controller.capture_zero(True)
            assert controller.status()["calibration"]["zero"] is True
        finally:
            controller.close()
    close_fixture(master, slave, fake)


def test_nonconsecutive_eagain_keeps_discovery_pending():
    """A zero-byte EAGAIN for J5 rid7 must requeue, not lose, discovery."""
    master, slave, fake, path = new_fixture()

    class DeferredDiscoveryWriter:
        def __init__(self):
            self.fd = None
            self.deferred = 0

        def write(self, packet):
            if packet[23] == 0x33 and packet[24] == 7 and self.deferred < 2:
                self.deferred += 1
                return {"kind": "eagain", "written": 0, "waited_s": 0.0}
            count = os.write(self.fd, packet)
            return ({"kind": "complete", "written": count, "waited_s": 0.0}
                    if count == len(packet) else
                    {"kind": "partial", "written": count, "waited_s": 0.0})

    writer = DeferredDiscoveryWriter()

    class DeferredFactory:
        def __call__(self, *args, **kwargs):
            serial = FaultSerial(*args, **kwargs)
            writer.fd = serial.fileno()
            return serial

    with tempfile.TemporaryDirectory() as folder:
        reader = J5Reader(path, serial_factory=DeferredFactory(),
                          audit_path=Path(folder) / "read.jsonl",
                          frame_writer=writer)
        try:
            end = time.monotonic() + 4
            while time.monotonic() < end:
                reader.poll()
                snapshot = reader.snapshot()
                if snapshot["ready"] and snapshot["connected"] and snapshot["fresh"]:
                    break
                time.sleep(.01)
            snapshot = reader.snapshot()
            assert writer.deferred == 2, writer.deferred
            assert snapshot["ready"] and snapshot["connected"], snapshot
            assert snapshot["registers"]["master_id"] == 16, snapshot["registers"]
            assert reader.setup == [], reader.setup
            assert reader.fault is None
        finally:
            reader.close()
    close_fixture(master, slave, fake)


def test_feedback_stale_not_write_fault():
    """Writes stay complete; only the 250ms feedback age gate marks stale."""
    master, slave, fake, path = new_fixture()
    factory = FaultFactory()
    with tempfile.TemporaryDirectory() as folder:
        reader = J5Reader(path, serial_factory=factory,
                          audit_path=Path(folder) / "read.jsonl",
                          frame_writer=factory.writer)
        try:
            end = time.monotonic() + 5
            while time.monotonic() < end:
                reader.poll()
                if reader.snapshot()["fresh"]:
                    break
                time.sleep(.02)
            assert reader.snapshot()["fresh"]
            fake.stop()  # peer no longer responds; PTY/device still exists
            stale = None
            for _ in range(20):
                reader.poll()
                time.sleep(.03)
                snapshot = reader.snapshot()
                if not snapshot["fresh"]:
                    stale = snapshot
                    break
            assert stale is not None, reader.snapshot()
            assert reader.fault is None
            assert stale["transport"]["state"] == "online"
            assert stale["feedback"] is not None  # last sample retained for diagnosis
            assert stale["age_s"] > .25
            assert reader.frame_writer is not None
        finally:
            reader.close()
    close_fixture(master, slave, fake)


def test_request_pressure():
    """Beyond the field failure count, with no real hardware and no sleep-busy loop."""
    master, slave, fake, path = new_fixture()
    factory = FaultFactory()
    with tempfile.TemporaryDirectory() as folder:
        reader = J5Reader(path, serial_factory=factory,
                          audit_path=Path(folder) / "read.jsonl",
                          frame_writer=factory.writer)
        try:
            for index in range(4005):
                reader.poll(now=1000.0 + index * .05)
                time.sleep(.0005)
            snapshot = reader.snapshot()
            assert reader.fault is None, reader.fault
            assert snapshot["transport"]["state"] == "online"
            assert snapshot["transport"]["frames_sent"] >= 4000
        finally:
            reader.close()
    close_fixture(master, slave, fake)


def main():
    test_long_normal()
    test_request_pressure()
    test_nonconsecutive_eagain_keeps_discovery_pending()
    test_feedback_stale_not_write_fault()
    run_fault("eagain", 25, "WRITE_BACKPRESSURE")
    run_fault("partial_write", 25, "PARTIAL_WRITE")
    # Read fault is detected before the next write; accept the transport fault name.
    run_fault("read_disconnect", 25, "READ_OR_WRITE_FAULT")
    evidence = {"passed": True, "physical_port_opened": False,
                "cases": ["long_normal_10s_150plus_frames", "4005_request_pressure",
                          "nonconsecutive_eagain_keeps_rid7_discovery",
                          "feedback_age_250ms_stale", "eagain_backpressure",
                          "partial_write_7_of_30", "read_disconnect_and_reconnect"],
                "assertions": ["connected_false_after_fault", "feedback_none_after_fault",
                               "port_reopen_after_fault", "reconnect_read_only",
                               "calibration_invalidated_rezero_required"],
                "no_motion_or_parameter_write": True}
    (HERE / "evidence" / "j5_transport_injection.json").write_text(
        json.dumps(evidence, indent=2), encoding="utf-8")
    print("PASS", json.dumps(evidence, ensure_ascii=False))


if __name__ == "__main__":
    main()
