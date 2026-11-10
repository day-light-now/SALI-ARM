"""Read-only CANUSB serial latency probe (no enable, no MIT, no motion).

Sends the existing J5 register-read request and measures write/response latency.
Requires the driver to be stopped and the port free.
"""
from __future__ import annotations

import os
import select
import statistics
import sys
import termios
import time
from pathlib import Path

sys.path.insert(0, str(Path("/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/src/slaihand_bringup/scripts")))
from j5_readonly import usb_packet  # noqa: E402


def configure(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = termios.B921600
    attrs[5] = termios.B921600
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        configure(fd)
        writes, reads, timeouts = [], [], 0
        for _ in range(200):
            packet = usb_packet(5, 7)
            start = time.perf_counter()
            written = 0
            while written < len(packet):
                written += os.write(fd, packet[written:])
            write_s = time.perf_counter() - start
            wanted = 16
            got = b""
            read_start = time.perf_counter()
            while len(got) < wanted and time.perf_counter() - read_start < 0.05:
                ready, _, _ = select.select([fd], [], [], 0.01)
                if not ready:
                    continue
                chunk = os.read(fd, wanted - len(got))
                if chunk:
                    got += chunk
            if len(got) < wanted:
                timeouts += 1
            writes.append(write_s * 1000.0)
            reads.append((time.perf_counter() - read_start) * 1000.0)
            time.sleep(0.01)
        for name, values in (("write_ms", writes), ("read_ms", reads)):
            ordered = sorted(values)
            print(name, "n=%d min=%.3f p50=%.3f p95=%.3f p99=%.3f max=%.3f" % (
                len(ordered), ordered[0], statistics.median(ordered),
                ordered[int(len(ordered) * .95)], ordered[int(len(ordered) * .99)], ordered[-1]))
        print("timeouts", timeouts)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
