"""Test-layer Reader factory with a unique-log-name retry.

The production Reader names its read-only log with second + microsecond-of-ns.
On this Windows host time.time_ns() has a ~15.6ms tick, so two Readers created
in the same tick can collide with FileExistsError. This fixture retries the
constructor with a real short sleep; it does not patch the clock, the Reader,
or any control timing. Codex requested a test-layer filename-unique fixture
rather than a global time shim.
"""
import time

from four_axis_monitor import Reader


def make_reader(port, mapping, serial_factory=None, attempts=50, delay=.02):
    last = None
    for _ in range(attempts):
        try:
            return Reader(port, mapping, serial_factory=serial_factory)
        except FileExistsError as exc:
            last = exc
            time.sleep(delay)
    raise last


def make_reader5(port, mapping, serial_factory=None, cal_path=None, attempts=50, delay=.02):
    from .five_axis_monitor import Reader5
    from .j5_calibration import DEFAULT_PATH
    last = None
    for _ in range(attempts):
        try:
            return Reader5(port, mapping, serial_factory=serial_factory,
                           cal_path=cal_path or DEFAULT_PATH)
        except FileExistsError as exc:
            last = exc
            time.sleep(delay)
    raise last
