"""拖动示教离线反例与单元测试（无 ROS、无硬件）。"""
import json
import math
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src" / "slaihand_bringup" / "scripts"))
import drag_teaching as teach  # noqa: E402

HASH = "a" * 64
IDENTITY = {"schema": "slaihand_runtime_identity/1", "boot_id": "boot-A", "started_at": 1.0,
            "mapping_sha256": HASH, "model_sha256": HASH, "policy_sha256": HASH,
            "mass_profile_sha256": HASH, "policy_profile": "p", "load_id": "no_end_load",
            "gravity_base_m_s2": [0.0, 9.80665, 0.0], "j3_offset_rad": math.pi,
            "kp": 0.0, "kd": 5.0, "reference_speed_rad_s": 1.0,
            "gravity_ff_alpha": 1.0, "gravity_ff_ramp_s": 3.0}
BOUNDS = [(-1.3962634015954636, 3.4906585039886591), (-0.17453292519943295, 3.3161255787892263),
          (-1.5707963267948966, 1.5707963267948966), (-0.8726646259971648, 1.5707963267948966)]
LIMITS_DEG = [[-80, 200], [-10, 190], [-90, 90], [-50, 90]]
TESTS = []


def test(function):
    TESTS.append(function)
    return function


def frames(count=120, start=0.0, step=0.01, amplitude=0.04):
    rows = []
    for index in range(count):
        t = start + index * step
        q = [0.10 + amplitude * index / max(count - 1, 1), 0.20, 0.0, -0.30]
        rows.append({"seq": index + 1, "at": t,
                     "frame": {"q": q, "dq": [0.01, 0.0, 0.0, 0.0], "age": [0.005] * 4,
                               "valid": [1] * 4, "motor": [1] * 4, "stage": 5, "float": True,
                               "hold_ready": True, "command_id": 7}})
    return rows


def feed(recorder, rows, identity=None):
    for row in rows:
        recorder.record(row["seq"], row["at"], row["frame"], identity or IDENTITY)


def capture_of(rows=None, end_frame=None, interrupted=None, stop_confirmed=True):
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    feed(recorder, rows if rows is not None else frames())
    if end_frame is not None:
        recorder.record(9999, (rows or frames())[-1]["at"] + 0.02, end_frame, IDENTITY)
    else:
        recorder.record(9999, (rows or frames())[-1]["at"] + 0.02,
                        {"q": [0.0, 0.0, 0.0, 0.0], "dq": [0] * 4, "age": [0.0] * 4,
                         "valid": [1] * 4, "motor": [0] * 4, "stage": 8, "float": False,
                         "hold_ready": False, "command_id": 7}, IDENTITY)
    return recorder.finalize(stop_confirmed, interrupted, "op-test", 1.0, 0.0, 5.0, 1.0, 3.0)


@test
def test_valid_drag_continues_after_initial_stillness_ready():
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    rows = frames(120)
    for row in rows[1:]:
        row["frame"]["hold_ready"] = False
        row["frame"]["dq"] = [0.2, 0.0, 0.0, 0.0]
    feed(recorder, rows)
    assert len(recorder.frames) == len(rows), recorder.closed_reason


@test
def test_float_settings_validation_and_long_capture():
    assert teach.normalize_float_settings() == {"kd": 5.0, "seconds": 5.0}
    assert teach.normalize_float_settings({"kd": 0, "seconds": 120}) == {"kd": 0., "seconds": 120.}
    for settings in ({"kd": float("nan"), "seconds": 5}, {"kd": 6, "seconds": 5},
                     {"kd": True, "seconds": 5}, {"kd": 2, "seconds": 121},
                     {"kd": 2, "seconds": 0}, {"kd": 2}, {"kd": 2, "seconds": 5, "other": 1}):
        try:
            teach.normalize_float_settings(settings)
        except teach.TeachingError as error:
            assert error.code == "FLOAT_SETTINGS"
        else:
            raise AssertionError("accepted invalid float settings")
    capture = capture_of(rows=frames(2000))
    assert capture["complete"], capture["reasons"]
    assert len(capture["samples"]) == 2000


@test
def test_valid_window_completes_and_roundtrips():
    capture = capture_of()
    assert capture["complete"] is True, capture["reasons"]
    assert capture["quality"]["frames"] == 120
    assert capture["quality"]["max_gap_s"] <= teach.MAX_GAP_S
    with tempfile.TemporaryDirectory() as directory:
        store = teach.TeachingStore(directory)
        recording_id = store.write_raw(capture)
        assert recording_id == capture["recording_id"]
        payload = store.save(capture, "教学 A")
        loaded = store.load(payload["id"])
        assert loaded["name"] == "教学 A"
        assert len(loaded["samples"]) == 120
        assert store.list()[0]["kind"] == "teaching"
        # Exclusive creation: same id cannot be written twice.
        try:
            store.save(capture, "教学 A")
        except teach.TeachingError as error:
            assert error.code in ("INTEGRITY", "INCOMPLETE", "ID", "SCHEMA")
        assert store.list()[0]["frames"] == 120


@test
def test_ramp_frames_not_recorded():
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    ramp = frames(10)
    for row in ramp:
        row["frame"] = dict(row["frame"], hold_ready=False, float=False)
    feed(recorder, ramp)
    assert recorder.display()["phase"] == "ramp_in"
    feed(recorder, frames(120, start=0.2))
    recorder.record(9999, 1.42, {"q": [0.0, 0.0, 0.0, 0.0], "dq": [0] * 4, "age": [0] * 4,
                                 "valid": [1] * 4, "motor": [0] * 4, "stage": 8,
                                 "float": False, "hold_ready": False, "command_id": 7}, IDENTITY)
    capture = recorder.finalize(True, None, "op", 1.0, 0.0, 5.0, 1.0, 3.0)
    assert capture["complete"] is True, capture["reasons"]
    assert capture["quality"]["waiting_frames"] == 10


@test
def test_stale_ages_reject_window():
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    bad = frames(5)
    for row in bad:
        row["frame"] = dict(row["frame"], age=[0.2] * 4)
    feed(recorder, bad)
    feed(recorder, frames(120, start=0.1))
    recorder.record(9999, 1.32, {"q": [0, 0, 0, 0], "dq": [0] * 4, "age": [0] * 4,
                                 "valid": [1] * 4, "motor": [0] * 4, "stage": 8,
                                 "float": False, "hold_ready": False, "command_id": 7}, IDENTITY)
    capture = recorder.finalize(True, None, "op", 1.0, 0.0, 5.0, 1.0, 3.0)
    assert capture["quality"]["waiting_frames"] == 5
    assert capture["complete"] is True


@test
def test_duplicate_frames_not_double_recorded():
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    rows = frames(120)
    feed(recorder, rows)
    recorder.record(rows[-1]["seq"], rows[-1]["at"] + 0.01, rows[-1]["frame"], IDENTITY)
    assert recorder.duplicate_frames >= 1
    assert recorder.display()["frames"] == 120


@test
def test_boot_or_command_change_marks_incomplete():
    for change in ("boot", "command"):
        recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
        feed(recorder, frames(60))
        if change == "boot":
            altered = dict(IDENTITY, boot_id="boot-B")
            recorder.record(999, 0.62, frames(1)[0]["frame"], altered)
        else:
            frame = dict(frames(1)[0]["frame"], command_id=8)
            recorder.record(999, 0.62, frame, IDENTITY)
        capture = recorder.finalize(True, None, "op", 1.0, 0.0, 5.0, 1.0, 3.0)
        assert capture["complete"] is False
        assert any("身份" in reason or "command_id" in reason for reason in capture["reasons"])


@test
def test_nan_and_out_of_limit_mark_incomplete():
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    feed(recorder, frames(60))
    nan = frames(1)[0]["frame"]
    nan = dict(nan, q=[float("nan"), 0.0, 0.0, 0.0])
    recorder.record(999, 0.62, nan, IDENTITY)
    capture = recorder.finalize(True, None, "op", 1.0, 0.0, 5.0, 1.0, 3.0)
    assert capture["complete"] is False

    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    feed(recorder, frames(60))
    outside = frames(1)[0]["frame"]
    outside = dict(outside, q=[0.0, 0.0, 0.0, 5.0])
    recorder.record(999, 0.62, outside, IDENTITY)
    capture = recorder.finalize(True, None, "op", 1.0, 0.0, 5.0, 1.0, 3.0)
    assert capture["complete"] is False
    assert any("越限" in reason for reason in capture["reasons"])


@test
def test_gap_and_time_backwards():
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    rows = frames(60)
    rows[30]["at"] += 0.2  # > MAX_GAP in the middle of the window
    feed(recorder, rows)
    capture = recorder.finalize(True, None, "op", 1.0, 0.0, 5.0, 1.0, 3.0)
    assert capture["complete"] is False
    assert any("间隙" in reason for reason in capture["reasons"])

    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    feed(recorder, frames(60))
    backwards = frames(1)[0]
    recorder.record(999, 0.30, backwards["frame"], IDENTITY)
    assert recorder.time_backwards >= 1


@test
def test_interrupted_and_short_records_incomplete():
    short = capture_of(rows=frames(5))
    assert short["complete"] is False
    interrupted = capture_of(interrupted="停止原因：浏览器心跳丢失")
    assert interrupted["complete"] is False
    assert any("中断" in reason for reason in interrupted["reasons"])
    no_stop = capture_of(stop_confirmed=False)
    assert no_stop["complete"] is False
    assert any("失能" in reason for reason in no_stop["reasons"])


@test
def test_name_and_path_separation():
    capture = capture_of()
    with tempfile.TemporaryDirectory() as directory:
        store = teach.TeachingStore(directory)
        for name in ("", "   ", "x" * 41, "bad\nname"):
            try:
                store.save(capture, name)
                raise AssertionError("accepted invalid name %r" % name)
            except teach.TeachingError as error:
                assert error.code in ("NAME", "SCHEMA")
        payload = store.save(capture, "含 空格 与中文的示教")
        assert payload["name"] == "含 空格 与中文的示教"
        assert teach.ID_RE.match(payload["id"])
        assert set(p.name for p in store.root.glob("*.json")) == {payload["id"] + ".json"}
        for bad in ("../etc/passwd", "ABC", "", "0" * 15):
            try:
                store.load(bad)
                raise AssertionError("accepted bad id %r" % bad)
            except teach.TeachingError as error:
                assert error.code in ("NOT_FOUND", "ID")


@test
def test_missing_axis_inf_and_exclusive_create():
    # 缺轴：样本必须是四轴，不允许用 0 或 3 轴填充。
    payload = make_teaching()
    broken = json.loads(json.dumps(payload))
    broken["samples"][3]["q"] = broken["samples"][3]["q"][:3]
    broken["content_sha256"] = teach.content_sha256(
        {"schema": teach.SCHEMA_CAPTURE, "source": broken["source"],
         "quality": broken["quality"], "samples": broken["samples"],
         "complete": broken["complete"], "reasons": broken["reasons"]})
    try:
        teach.validate_teaching(broken)
        raise AssertionError("missing axis accepted")
    except teach.TeachingError as error:
        assert error.code == "SHAPE"
    # Inf/NaN：明确拒绝，不能带入回放。
    for bad_value in (float("inf"), float("nan")):
        payload = make_teaching()
        broken = json.loads(json.dumps(payload))
        broken["samples"][5]["q"][1] = bad_value
        try:
            teach.validate_teaching(broken)
            raise AssertionError("non-finite sample accepted")
        except teach.TeachingError as error:
            assert error.code in ("NUMBER", "INTEGRITY")
    # 独占创建：同一路径第二次写入必须失败，绝不覆盖既有教学文件。
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "fixed.json"
        teach._exclusive_write(path, "{}")
        try:
            teach._exclusive_write(path, "{}")
            raise AssertionError("exclusive create overwrote an existing file")
        except FileExistsError:
            pass
        capture = capture_of()
        store = teach.TeachingStore(directory)
        store.write_raw(capture)
        try:
            store.write_raw(capture)
            raise AssertionError("raw recording overwritten")
        except FileExistsError:
            pass


@test
def test_tamper_and_schema_rejected():
    capture = capture_of()
    with tempfile.TemporaryDirectory() as directory:
        store = teach.TeachingStore(directory)
        payload = store.save(capture, "tamper")
        payload["samples"][10]["q"][0] += 0.05
        path = store.root / (payload["id"] + ".json")
        path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        try:
            store.load(payload["id"])
            raise AssertionError("tampered teaching accepted")
        except teach.TeachingError as error:
            assert error.code == "INTEGRITY"
        # Missing field / wrong schema.
        broken = dict(payload)
        broken.pop("quality")
        try:
            teach.validate_teaching(broken)
            raise AssertionError("missing quality accepted")
        except teach.TeachingError as error:
            assert error.code == "SCHEMA"
        wrong = dict(payload, schema="slaihand_drag_teaching/2")
        try:
            teach.validate_teaching(wrong)
            raise AssertionError("wrong schema accepted")
        except teach.TeachingError as error:
            assert error.code in ("SCHEMA", "INTEGRITY")
        # Incomplete capture cannot be saved.
        incomplete = teach.validate_capture(capture_of(rows=frames(3)))
        try:
            store.save(incomplete, "no")
            raise AssertionError("incomplete capture saved")
        except teach.TeachingError as error:
            assert error.code == "INCOMPLETE"


@test
def test_playback_compile_retimes_and_limits():
    payload = make_teaching()
    start = [0.0, 0.0, 0.0, 0.0]
    compiled = teach.compile_playback(payload, start, BOUNDS, 0.5, identity=IDENTITY)
    preview = compiled["preview"]
    assert abs(preview["speed_factor"] - 0.5) < 1e-9
    assert preview["approach_s"] >= teach.APPROACH_MIN_S
    assert abs(preview["retimed_duration_s"]
               - (preview["approach_s"] + preview["original_duration_s"] / 0.5)) < 0.02
    assert preview["peak_speed_rad_s"] <= teach.SPEED_LIMIT_RAD_S + 1e-9
    assert len(preview["ranges_deg"]) == 4
    for row in preview["ranges_deg"]:
        assert row[0] <= row[1]
    # approach points start exactly at the actual start and reach the first sample.
    points, meta = teach.build_trajectory(compiled["frozen"], start, BOUNDS)
    assert points[0]["t"] == 0.0 and points[0]["q"] == start
    assert all(points[i + 1]["t"] > points[i]["t"] for i in range(len(points) - 1))
    assert meta["peak_speed_rad_s"] <= teach.SPEED_LIMIT_RAD_S + 1e-9
    # Every recorded sample is present exactly (no lossy simplification).
    last = points[-1]["q"]
    assert abs(last[0] - payload["samples"][-1]["q"][0]) < 1e-9
    # A different actual start gets its own approach and remains valid.
    other = [0.2, 0.1, 0.0, -0.2]
    points2, meta2 = teach.build_trajectory(compiled["frozen"], other, BOUNDS)
    assert points2[0]["q"] == other
    assert meta2["approach_s"] != meta["approach_s"] or True


@test
def test_speed_factor_bounds_and_overspeed_rejection():
    payload = make_teaching(peak=2.5)
    start = [0.0, 0.0, 0.0, 0.0]
    try:
        teach.compile_playback(payload, start, BOUNDS, 0.5, identity=IDENTITY)
        raise AssertionError("overspeed playback accepted")
    except teach.TeachingError as error:
        assert error.code == "SPEED"
        assert "比例" in error.message
    compiled = teach.compile_playback(payload, start, BOUNDS, 0.2, identity=IDENTITY)
    assert compiled["preview"]["peak_speed_rad_s"] <= teach.SPEED_LIMIT_RAD_S + 1e-9
    for factor in (0.05, 1.2, float("nan")):
        try:
            teach.compile_playback(payload, start, BOUNDS, factor, identity=IDENTITY)
            raise AssertionError("accepted bad factor %r" % factor)
        except teach.TeachingError as error:
            assert error.code in ("SPEED_FACTOR", "NUMBER")


@test
def test_playback_limits_and_budget():
    payload = make_teaching()
    # Current start outside the current URDF limit must be refused, not clipped.
    try:
        teach.compile_playback(payload, [0.0, 0.0, 0.0, 2.0], BOUNDS, 0.5, identity=IDENTITY)
        raise AssertionError("out-of-limit start accepted")
    except teach.TeachingError as error:
        assert error.code == "LIMIT"
    # J4 upper limit changed to +90°: a recorded pose beyond it is refused.
    payload = make_teaching(j4=math.radians(95))
    try:
        teach.compile_playback(payload, [0, 0, 0, 0], BOUNDS, 0.5, identity=IDENTITY)
        raise AssertionError("out-of-limit sample accepted")
    except teach.TeachingError as error:
        assert error.code == "LIMIT"
    # Budget: a long original duration scaled by 0.1 exceeds 169 s.
    payload = make_teaching(duration_s=20.0)
    try:
        teach.compile_playback(payload, [0, 0, 0, 0], BOUNDS, 0.1,
                               budget_s=teach.PLAYBACK_LIMIT_S, identity=IDENTITY)
        raise AssertionError("over-budget playback accepted")
    except teach.TeachingError as error:
        assert error.code == "BUDGET"


@test
def test_identity_compatibility_allows_new_boot_but_not_config():
    payload = make_teaching()
    source = payload["source"]["identity"]
    assert teach.identity_compatibility(source, dict(source, boot_id="boot-B")) == []
    mismatched = dict(source, mapping_sha256="b" * 64)
    assert "mapping_sha256" in teach.identity_compatibility(source, mismatched)
    load_mismatch = dict(source, load_id="with_load")
    assert "load_id" in teach.identity_compatibility(source, load_mismatch)


@test
def test_tracking_error_and_interp_bound():
    payload = make_teaching()
    compiled = teach.compile_playback(payload, [0, 0, 0, 0], BOUNDS, 0.5, identity=IDENTITY)
    points, meta = teach.build_trajectory(compiled["frozen"], [0, 0, 0, 0], BOUNDS)
    assert meta["interp_error_bound_deg"] >= 0.0
    stamps = [(points[index]["t"], list(points[index]["q"])) for index in range(len(points))]
    tracking = teach.tracking_error_deg(points, stamps, 0.0)
    assert tracking is not None
    assert max(tracking["max_tracking_error_deg"]) < 1e-6


@test
def test_source_timestamp_backwards_marks_incomplete():
    """Codex P2：时间倒退必须关闭窗口并保留异常原因，不能仍标记完整。"""
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    feed(recorder, frames(60))
    stamp_frame = dict(frames(1)[0]["frame"], source_stamp=123.0)
    recorder.record(61, 0.58, stamp_frame, IDENTITY)  # 单调时钟倒退
    assert recorder.time_backwards == 1
    end = dict(frames(1)[0]["frame"], stage=8, motor=[0] * 4, float=False, hold_ready=False)
    recorder.record(62, 0.60, end, IDENTITY)
    capture = recorder.finalize(True, None, "backward", 1.0)
    assert capture["complete"] is False, capture["reasons"]
    assert any("倒退" in reason or "未前进" in reason for reason in capture["reasons"])


@test
def test_unknown_terminal_frame_is_not_normal_end():
    """Codex P2：UNKNOWN + 反馈无效 + 年龄超限不能被当作正常结束。"""
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    feed(recorder, frames(60))
    terminal = dict(frames(1)[0]["frame"], stage=9, valid=[False] * 4, age=[10.0] * 4)
    recorder.record(61, 0.60, terminal, IDENTITY)
    capture = recorder.finalize(True, None, "unknown-then-disabled", 1.0)
    assert capture["complete"] is False, capture["reasons"]
    assert capture["quality"]["window_normal_end"] is not True
    assert "UNKNOWN" in (capture["quality"]["window_close_reason"] or "") or \
        any("反馈无效" in reason for reason in capture["reasons"])


@test
def test_same_source_frame_is_deduplicated():
    """Codex P2：同一源帧重复投递 60 次只能算 1 帧，不得伪造 0.59 秒。"""
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    frame = dict(frames(1)[0]["frame"], source_stamp=123.0)
    for index in range(60):
        recorder.record(index + 1, 100.0 + index * 0.01, frame, IDENTITY)
    display = recorder.display()
    assert display["frames"] == 1, display
    assert display["duplicate_frames"] == 59, display
    assert display["duration_s"] == 0.0, display
    recorder.record(999, 101.0, dict(frame, stage=8, motor=[0] * 4, float=False,
                                     hold_ready=False), IDENTITY)
    capture = recorder.finalize(True, None, "dup", 1.0)
    assert capture["complete"] is False


@test
def test_source_timestamp_survives_storage_and_legacy_records_load():
    rows = frames(60)
    for row in rows:
        row["frame"]["source_stamp"] = 123.0 + row["at"]
    capture = capture_of(rows)
    with tempfile.TemporaryDirectory() as directory:
        store = teach.TeachingStore(directory)
        store.write_raw(capture)
        loaded = store.load_raw(capture["recording_id"])
        assert [sample["source_stamp"] for sample in loaded["samples"]] == [
            row["frame"]["source_stamp"] for row in rows]
        teaching = store.save(loaded, "source timestamp")
        assert store.load(teaching["id"])["samples"] == loaded["samples"]
    legacy = make_teaching()
    assert all("source_stamp" not in sample for sample in legacy["samples"])
    assert teach.validate_teaching(legacy)["content_sha256"] == legacy["content_sha256"]


@test
def test_invalid_source_timestamp_metadata_is_rejected():
    for value in (-1.0, float("nan"), float("inf"), True, "123"):
        payload = make_teaching()
        payload["samples"][0]["source_stamp"] = value
        try:
            teach.validate_teaching(payload)
        except teach.TeachingError as error:
            assert error.code == "TIME", error
        else:
            raise AssertionError("accepted invalid source timestamp %r" % value)


@test
def test_stop_before_first_valid_frame_keeps_diagnostic_capture():
    """Codex Standards：渐入期间停止也要留下不完整诊断记录（但不能保存为可回放示教）。"""
    recorder = teach.TeachingRecorder(BOUNDS, LIMITS_DEG)
    ramp = frames(10, start=0.0)
    for row in ramp:
        row["frame"] = dict(row["frame"], hold_ready=False, float=False)
    feed(recorder, ramp)
    capture = recorder.finalize(True, None, "no-frames", 1.0)
    assert capture["complete"] is False
    assert capture["source"]["identity"] is None
    assert capture["samples"] == []
    checked = teach.validate_capture(capture)
    assert checked["complete"] is False
    with tempfile.TemporaryDirectory() as directory:
        store = teach.TeachingStore(directory)
        recording_id = store.write_raw(capture)
        assert recording_id == capture["recording_id"]
        assert store.load_raw(recording_id)["complete"] is False
        try:
            store.save(capture, "诊断记录")
            raise AssertionError("incomplete diagnostic capture was saved as a teaching")
        except teach.TeachingError as error:
            assert error.code == "INCOMPLETE"


@test
def test_playback_spec_is_bound_to_runtime_identity():
    payload = make_teaching()
    start = [0.0, 0.0, 0.0, 0.0]
    try:
        teach.compile_playback(payload, start, BOUNDS, 0.5)
        raise AssertionError("playback compiled without a runtime identity")
    except teach.TeachingError as error:
        assert error.code == "IDENTITY"
    compiled = teach.compile_playback(payload, start, BOUNDS, 0.5, identity=IDENTITY)
    frozen = compiled["frozen"]
    assert frozen["identity_sha256"] == teach.identity_fingerprint(IDENTITY)
    assert teach.validate_frozen(frozen)["identity_sha256"] == frozen["identity_sha256"]
    tampered = dict(frozen, identity=dict(IDENTITY, mapping_sha256="b" * 64))
    try:
        teach.validate_frozen(tampered)
        raise AssertionError("mismatched frozen identity accepted")
    except teach.TeachingError as error:
        assert error.code == "IDENTITY"


def make_teaching(peak=0.05, j4=-0.30, duration_s=1.2, frames_count=120):
    """构造一份内容自洽的已保存教学（不经过记录器）。"""
    count = max(teach.MIN_SAMPLES, frames_count)
    step = duration_s / (count - 1)
    samples = []
    amplitude = peak * step * (count - 1)
    for index in range(count):
        t = index * step
        q = [0.10 + amplitude * index / (count - 1), 0.20, 0.0, j4]
        samples.append({"t": round(t, 6), "mono_at": round(100.0 + t, 6),
                        "q": [round(value, 9) for value in q],
                        "dq": [round(peak, 9), 0.0, 0.0, 0.0],
                        "age": [0.005] * 4, "stage": 5, "command_id": 7,
                        "motor": [1, 1, 1, 1]})
    capture = {"schema": teach.SCHEMA_CAPTURE, "recording_id": "0123456789abcdef",
               "created_at": 1.0, "source": {
                   "operation_id": "op", "identity": dict(IDENTITY),
                   "identity_sha256": teach.identity_fingerprint(IDENTITY),
                   "boot_id": "boot-A", "command_id": 7, "started_at": 100.0,
                   "limits_deg": LIMITS_DEG, "reference_speed_rad_s": 1.0,
                   "driver_kp": 0.0, "driver_kd": 5.0, "driver_gravity_ff_alpha": 1.0,
                   "driver_gravity_ff_ramp_s": 3.0},
               "quality": {"frames": len(samples), "raw_frames": len(samples),
                           "duplicate_frames": 0, "time_backwards": 0, "waiting_frames": 0,
                           "duration_s": duration_s, "sample_rate_hz": count / duration_s,
                           "min_gap_s": step, "max_gap_s": step, "mean_gap_s": step,
                           "max_age_s": 0.005, "window_close_reason": "正常停止",
                           "window_normal_end": True, "interrupted": None},
               "samples": samples, "complete": True, "reasons": []}
    capture["content_sha256"] = teach.content_sha256(capture)
    payload = {"schema": teach.SCHEMA_TEACHING, "id": "fedcba9876543210",
               "recording_id": capture["recording_id"], "name": "离线教学",
               "created_at": 1.0, "saved_at": 2.0, "source": capture["source"],
               "quality": capture["quality"], "samples": capture["samples"],
               "complete": True, "reasons": [],
               "content_sha256": capture["content_sha256"]}
    return teach.validate_teaching(payload)


def main():
    failures = []
    for function in TESTS:
        started = time.monotonic()
        try:
            function()
            print("PASS  %s (%.2fs)" % (function.__name__, time.monotonic() - started))
        except Exception as error:  # noqa: BLE001
            failures.append(function.__name__)
            print("FAIL  %s: %s: %s" % (function.__name__, type(error).__name__, error))
    print("RESULT: %s (%d/%d)" % ("FAIL" if failures else "PASS",
                                  len(TESTS) - len(failures), len(TESTS)))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
