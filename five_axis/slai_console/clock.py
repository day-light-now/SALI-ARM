"""One monotonic high-resolution domain for console observation and scheduling.

Python 3.12 Windows monotonic() uses GetTickCount64 at 15.625 ms resolution.
perf_counter() uses QueryPerformanceCounter and must not be mixed with it.
"""
import time

CLOCK_NAME='perf_counter'
now=time.perf_counter
sleep=time.sleep


def describe():
    return dict(name=CLOCK_NAME,**vars(time.get_clock_info(CLOCK_NAME)))
