"""Lifecycle races: close/reconnect serialization and concurrent reconnects."""
import os
import pty
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(HERE))
from test_j5_readonly import FakeJ5  # noqa: E402
from test_j5_transport import FaultFactory  # noqa: E402
from j5_controller import J5Controller  # noqa: E402
from j5_readonly import J5ProtocolError  # noqa: E402


class BlockingFactory(FaultFactory):
    def __init__(self):
        super().__init__()
        self.block_next = False
        self.entered = threading.Event()
        self.release = threading.Event()

    def __call__(self, *args, **kwargs):
        serial = super().__call__(*args, **kwargs)
        if self.block_next:
            self.block_next = False
            self.entered.set()
            self.release.wait(10)
        return serial


def fixture(factory):
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


def wait_online(controller, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        status = controller.status()
        if status.get("connected") and status.get("fresh"):
            return status
        time.sleep(.03)
    raise AssertionError(status)


def main():
    # 1) close must win the lifecycle race; no reader/thread may be installed after it.
    master, slave, fake, path = fixture(BlockingFactory())
    with tempfile.TemporaryDirectory() as folder:
        controller_factory = BlockingFactory()
        controller = J5Controller(path, Path(folder) / "cal.json",
                                  serial_factory=controller_factory)
        try:
            wait_online(controller)
            controller._fault("injected lifecycle fault")
            time.sleep(.1)
            controller_factory.block_next = True
            reconnect_result = {}
            reconnect_thread = threading.Thread(
                target=lambda: reconnect_result.update(controller.reconnect()))
            reconnect_thread.start()
            assert controller_factory.entered.wait(5), "reconnect did not reach factory"
            closed = threading.Event()
            def close_controller():
                controller.close()
                closed.set()
            close_thread = threading.Thread(target=close_controller)
            close_thread.start()
            controller_factory.release.set()
            reconnect_thread.join(5)
            close_thread.join(5)
            assert closed.is_set()
            assert close_thread.is_alive() is False
            assert controller._closed is True
            assert controller.reader is not None and controller.reader.closed is True
            assert controller.reader.serial is None
            assert not controller._thread.is_alive()
        finally:
            controller.close()
    close_fixture(master, slave, fake)

    # 2) two reconnects serialize; the second sees the first's live reader and
    #    must not stop or replace it.
    master, slave, fake, path = fixture(FaultFactory())
    factory = FaultFactory()
    with tempfile.TemporaryDirectory() as folder:
        controller = J5Controller(path, Path(folder) / "cal.json", serial_factory=factory)
        try:
            wait_online(controller)
            controller._fault("injected concurrent reconnect fault")
            time.sleep(.1)
            results = []
            start = threading.Barrier(3)
            def reconnect():
                start.wait()
                results.append(controller.reconnect())
            threads = [threading.Thread(target=reconnect) for _ in range(2)]
            for thread in threads:
                thread.start()
            start.wait()
            for thread in threads:
                thread.join(8)
            assert len(results) == 2
            assert sum(1 for result in results if result.get("reconnected")) == 1, results
            assert all(result.get("ok") for result in results), results
            status = wait_online(controller)
            assert status.get("error") is None
            assert not controller.reader.closed
            assert controller.reader.serial is not None
            # The constructor opened one reader; exactly one reconnect opened another.
            assert len(factory.instances) == 2, len(factory.instances)
        finally:
            controller.close()
    close_fixture(master, slave, fake)

    # 3) close first, reconnect later must refuse instead of reopening.
    master, slave, fake, path = fixture(FaultFactory())
    with tempfile.TemporaryDirectory() as folder:
        controller = J5Controller(path, Path(folder) / "cal.json",
                                  serial_factory=FaultFactory())
        controller.close()
        try:
            controller.reconnect()
        except J5ProtocolError:
            pass
        else:
            raise AssertionError("reconnect reopened after close")
        assert controller._closed is True
    close_fixture(master, slave, fake)
    evidence = {"passed": True, "physical_port_opened": False,
                "cases": ["close_during_blocked_reconnect_no_reopen",
                          "two_concurrent_reconnects_one_new_reader",
                          "close_then_reconnect_refused"],
                "assertions": ["close_wins_lifecycle", "no_thread_after_close",
                               "serial_closed_after_close",
                               "second_reconnect_does_not_stop_first",
                               "no_reopen_after_close"]}
    (HERE / "evidence" / "j5_lifecycle.json").write_text(
        __import__("json").dumps(evidence, indent=2), encoding="utf-8")
    print("PASS", __import__("json").dumps(evidence, ensure_ascii=False))


if __name__ == "__main__":
    main()
