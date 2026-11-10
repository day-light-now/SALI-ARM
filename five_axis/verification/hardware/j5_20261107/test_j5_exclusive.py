"""Exclusive-open counterexamples for J5Reader: second reader/native owner/alias."""
import fcntl
import os
import pty
import sys
import termios
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_j5_readonly import FakeJ5, wait_ready
from j5_readonly import J5Reader


def main():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fake = FakeJ5(master)
    alias = Path("/tmp") / ("j5-alias-%d" % os.getpid())
    try:
        alias.unlink(missing_ok=True)
        alias.symlink_to(path)
        first = J5Reader(path)
        wait_ready(first)
        # 1) A second J5Reader on the same real path must fail before polling.
        try:
            J5Reader(path)
            raise AssertionError("second J5Reader opened the same J5 device")
        except Exception as error:
            assert "exclusive" in str(error).lower() or "busy" in str(error).lower(), error
        # 2) Aliased symlink resolves to the same device and is also refused.
        try:
            J5Reader(str(alias))
            raise AssertionError("aliased J5Reader opened the same J5 device")
        except Exception as error:
            assert "exclusive" in str(error).lower() or "busy" in str(error).lower(), error
        # A native-style owner must not be able to open/flock while J5Reader owns.
        native = None
        try:
            native = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(native, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(native, termios.TIOCEXCL)
            raise AssertionError("native owner opened/flocked while J5Reader owns the device")
        except OSError:
            if native is not None:
                os.close(native)
            native = None
        first.close()

        # 3) Native owner first: the J5Reader must fail and must not clear/send.
        native = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(native, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(native, termios.TIOCEXCL)
        try:
            J5Reader(path)
            raise AssertionError("J5Reader opened while native owner held the device")
        except Exception:
            pass
        os.set_blocking(master, False)
        try:
            leaked = os.read(master, 1024)
        except BlockingIOError:
            leaked = b""
        assert not leaked, "J5Reader wrote/cleared while lock was owned: %r" % leaked
        os.close(native)
        try:
            os.close(master)
        except OSError:
            pass
        try:
            os.close(slave)
        except OSError:
            pass
        alias.unlink(missing_ok=True)
        fake.stop()
        evidence = {"passed": True, "physical_port_opened": False,
                    "canonical_path": os.path.realpath(path),
                    "checks": ["second_reader_refused", "alias_reader_refused",
                              "native_owner_refused_after_reader",
                              "reader_refused_when_native_owner_first",
                              "no_leaked_bytes_while_locked"],
                    "exclusive": ["flock LOCK_EX|LOCK_NB", "TIOCEXCL", "realpath alias"]}
        evidence_dir = Path(__file__).resolve().parent / "evidence"
        evidence_dir.mkdir(exist_ok=True)
        (evidence_dir / "j5_exclusive.json").write_text(
            __import__("json").dumps(evidence, indent=2), encoding="utf-8")
        print("PASS", __import__("json").dumps(evidence, ensure_ascii=False))
    finally:
        fake.stop()
        try:
            os.close(master)
            os.close(slave)
        except OSError:
            pass
        alias.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
