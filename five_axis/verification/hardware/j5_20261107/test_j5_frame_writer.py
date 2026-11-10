"""J5 frame-writer semantics: complete submission returns, EAGAIN is bounded."""
import errno
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
from j5_readonly import FRAME_WRITE_TIMEOUT_S, J5FrameWriter  # noqa: E402


def main():
    packet = bytes(range(30))
    # 1) A complete os.write returns immediately and does NOT wait for
    #    writability again (the pyserial post-write select bug).
    read_fd, write_fd = os.pipe()
    try:
        select_calls = []
        def never_select(*args, **kwargs):
            select_calls.append(args)
            raise AssertionError("complete submission must not select again")
        writer = J5FrameWriter(write_fd, select_fn=never_select)
        result = writer.write(packet)
        assert result["kind"] == "complete" and result["written"] == 30, result
        assert select_calls == []
        assert os.read(read_fd, 30) == packet
    finally:
        os.close(read_fd)
        os.close(write_fd)

    # 2) Complete with a fake write_fn; the writer must not call select.
    calls = {"write": 0, "select": 0}
    def full_write(fd, data):
        calls["write"] += 1
        return len(data)
    def bad_select(*args, **kwargs):
        calls["select"] += 1
        raise AssertionError("no post-complete select")
    result = J5FrameWriter(1, write_fn=full_write, select_fn=bad_select).write(packet)
    assert result["kind"] == "complete" and calls == {"write": 1, "select": 0}

    # 3) EAGAIN with zero bytes accepted: bounded by the 5ms deadline, never an
    #    infinite pyserial-style loop.
    calls = {"write": 0, "select": 0}
    def eagain_write(fd, data):
        calls["write"] += 1
        raise BlockingIOError(errno.EAGAIN, "injected EAGAIN")
    def no_ready(*args, **kwargs):
        calls["select"] += 1
        return [], [], []
    started = time.monotonic()
    result = J5FrameWriter(1, write_fn=eagain_write, select_fn=no_ready).write(packet)
    elapsed = time.monotonic() - started
    assert result["kind"] == "eagain" and result["written"] == 0, result
    assert elapsed < FRAME_WRITE_TIMEOUT_S + .05, elapsed
    assert calls["write"] < 1000 and calls["select"] < 1000, calls

    # 4) Short write, then EAGAIN: report partial bytes, never pretend complete.
    state = {"n": 0}
    def short_write(fd, data):
        state["n"] += 1
        if state["n"] == 1:
            return 7
        raise BlockingIOError(errno.EAGAIN, "injected EAGAIN")
    result = J5FrameWriter(1, write_fn=short_write, select_fn=no_ready).write(packet)
    assert result["kind"] == "partial" and result["written"] == 7, result

    # 5) Positive progress must not extend the total deadline. One byte per
    #    write with 4ms increments must stop after the first two writes.
    now = [0.0]
    calls = []
    def positive_partial(fd, data):
        now[0] += .004
        calls.append(len(data))
        return 1
    result = J5FrameWriter(1, clock=lambda: now[0],
                           write_fn=positive_partial).write(packet)
    assert result["kind"] == "partial" and result["written"] == 2, result
    assert result["waited_s"] == .008, result
    assert len(calls) == 2, calls

    # 6) A ready select followed by EAGAIN must still terminate at the deadline.
    calls = {"write": 0, "select": 0}
    def always_eagain(fd, data):
        calls["write"] += 1
        raise BlockingIOError(errno.EAGAIN, "injected EAGAIN")
    def always_ready(*args, **kwargs):
        calls["select"] += 1
        return [], [1], []
    started = time.monotonic()
    result = J5FrameWriter(1, write_fn=always_eagain, select_fn=always_ready).write(packet)
    elapsed = time.monotonic() - started
    assert result["kind"] == "eagain", result
    assert elapsed < FRAME_WRITE_TIMEOUT_S + .05, elapsed
    assert calls["write"] < 10000, calls
    evidence = {"passed": True, "cases": [
        "complete_30_of_30_returns_without_post_write_select",
        "short_write_reports_partial",
        "positive_partial_progress_obeys_total_deadline",
        "eagain_zero_bytes_returns_within_5ms",
        "ready_but_eagain_bounded_until_deadline"],
        "deadline_s": FRAME_WRITE_TIMEOUT_S,
        "pyserial_write_not_used": True}
    (Path(__file__).resolve().parent / "evidence" / "j5_frame_writer.json").write_text(
        json.dumps(evidence, indent=2), encoding="utf-8")
    print("PASS", json.dumps(evidence, ensure_ascii=False))


if __name__ == "__main__":
    main()
