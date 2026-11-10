"""Offline four/five-axis protected client and drag-teaching schema tests.

No ROS node, driver, serial, CAN or USB access. The protected client is imported
only for its pure argument/bounds helpers; its ``main()`` is never allowed past
argument validation, so ``rclpy.init``/ROS graph are never touched. When ROS
Python modules are absent, inert stubs are used for that import only.

Run:
    python -m unittest verification.hardware.five_axis_20261107.test_five_axis_client_teaching -v
"""
import contextlib
import io
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
URDF = ROOT / "src" / "slaihand_description" / "urdf" / "slaihand_four_axis.urdf"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE))

import drag_teaching as teach  # noqa: E402
import joint_waypoint_plan as planner  # noqa: E402

# Import-time state must stay the historical four-axis /1 configuration.
DEFAULT_JOINT_COUNT = teach.JOINT_COUNT
DEFAULT_SCHEMAS = (teach.SCHEMA_CAPTURE, teach.SCHEMA_TEACHING, teach.SCHEMA_PLAYBACK)

FOUR_LIMITS_DEG = [(-80, 200), (-10, 190), (-90, 90), (-50, 90)]
J5_LIMITS_DEG = (-30, 30)
FIVE_LIMITS_DEG = FOUR_LIMITS_DEG + [list(J5_LIMITS_DEG)]
FOUR_BOUNDS = [tuple(map(math.radians, pair)) for pair in FOUR_LIMITS_DEG]
FIVE_BOUNDS = FOUR_BOUNDS + [tuple(map(math.radians, J5_LIMITS_DEG))]
HASH = "a" * 64
IDENTITY = {"schema": "slaihand_runtime_identity/1", "boot_id": "boot-A", "started_at": 1.0,
            "mapping_sha256": HASH, "model_sha256": HASH, "policy_sha256": HASH,
            "mass_profile_sha256": HASH, "policy_profile": "p", "load_id": "no_end_load",
            "gravity_base_m_s2": [0.0, 9.80665, 0.0], "j3_offset_rad": math.pi,
            "kp": 0.0, "kd": 5.0, "reference_speed_rad_s": 1.0,
            "gravity_ff_alpha": 1.0, "gravity_ff_ramp_s": 3.0}


# ---------------------------------------------------------------- client import
class _Message:
    def __init__(self, *args, **kwargs):
        pass


class _Enum:
    BEST_EFFORT = 1
    RELIABLE = 2
    TRANSIENT_LOCAL = 3
    VOLATILE = 4
    NO = 5


_STUBS = {
    "rclpy": {},
    "rclpy.duration": {"Duration": _Message},
    "rclpy.action": {"ActionClient": _Message},
    "rclpy.qos": {"QoSProfile": _Message, "ReliabilityPolicy": _Enum,
                  "DurabilityPolicy": _Enum},
    "rclpy.signals": {"SignalHandlerOptions": _Enum},
    "ament_index_python": {},
    "ament_index_python.packages": {"get_package_share_directory": lambda name: str(HERE)},
    "sensor_msgs": {},
    "sensor_msgs.msg": {"JointState": _Message},
    "control_msgs": {},
    "control_msgs.msg": {"DynamicJointState": _Message,
                         "JointTrajectoryControllerState": _Message, "JointTolerance": _Message},
    "control_msgs.action": {"FollowJointTrajectory": _Message},
    "controller_manager_msgs": {},
    "controller_manager_msgs.srv": {"SwitchController": _Message},
    "std_srvs": {},
    "std_srvs.srv": {"Trigger": _Message},
    "std_msgs": {},
    "std_msgs.msg": {"String": _Message, "Float64MultiArray": _Message},
    "rcl_interfaces": {},
    "rcl_interfaces.msg": {"Parameter": _Message, "ParameterValue": _Message,
                           "ParameterType": _Enum},
    "rcl_interfaces.srv": {"SetParametersAtomically": _Message},
    "trajectory_msgs": {},
    "trajectory_msgs.msg": {"JointTrajectoryPoint": _Message},
}


def _install_stubs():
    for name, attributes in _STUBS.items():
        module = types.ModuleType(name)
        for key, value in attributes.items():
            setattr(module, key, value)
        sys.modules[name] = module


def _load_client():
    try:
        import rclpy  # noqa: F401
        real_ros = hasattr(rclpy, "init")
    except ImportError:
        real_ros = False
    if not real_ros:
        _install_stubs()
    import commission_supported_trial as client
    return client


client = _load_client()


# ---------------------------------------------------------------- fixtures
def five_document(degrees=(1, 2, -3, 4, 5), mode="absolute_deg", move=4, dwell=1):
    return {"schema": planner.SCHEMA, "coordinates": mode,
            "waypoints": [{"degrees": list(degrees), "move_s": move, "dwell_s": dwell}]}


def legacy_document(degrees=(1, 2, 3, 4), mode="absolute_deg"):
    return {"coordinates": mode,
            "waypoints": [{"degrees": list(degrees), "move_s": 4, "dwell_s": 1}]}


def five_frames(count=120, start=0.0, step=0.01, amplitude=0.04):
    rows = []
    for index in range(count):
        t = start + index * step
        ratio = index / max(count - 1, 1)
        q = [0.10 + amplitude * ratio, 0.20, 0.0, -0.30, 0.15 + amplitude * ratio]
        rows.append({"seq": index + 1, "at": t,
                     "frame": {"q": q, "dq": [0.01, 0.0, 0.0, 0.0, 0.0],
                               "age": [0.005] * 5, "valid": [1] * 5, "motor": [1] * 5,
                               "stage": 5, "float": True, "hold_ready": True,
                               "command_id": 7}})
    return rows


def stop_frame(axes=5):
    return {"q": [0.0] * axes, "dq": [0] * axes, "age": [0.0] * axes,
            "valid": [1] * axes, "motor": [0] * axes, "stage": 8, "float": False,
            "hold_ready": False, "command_id": 7}


def feed(recorder, rows):
    for row in rows:
        recorder.record(row["seq"], row["at"], row["frame"], IDENTITY)


def limits_for(axes):
    return [list(row) for row in (FOUR_LIMITS_DEG if axes == 4 else FIVE_LIMITS_DEG)]


def make_capture(axes=4, count=120, peak=0.05, duration_s=1.2):
    """Self-consistent capture in the currently configured schema; no fabrication."""
    count = max(teach.MIN_SAMPLES, count)
    step = duration_s / (count - 1)
    amplitude = peak * step * (count - 1)
    samples = []
    for index in range(count):
        t = index * step
        q = [0.10 + amplitude * index / (count - 1), 0.20, 0.0, -0.30, 0.15][:axes]
        samples.append({"t": round(t, 6), "mono_at": round(100.0 + t, 6),
                        "q": [round(value, 9) for value in q],
                        "dq": [round(peak, 9)] + [0.0] * (axes - 1),
                        "age": [0.005] * axes, "stage": 5, "command_id": 7,
                        "motor": [1] * axes})
    capture = {"schema": teach.SCHEMA_CAPTURE, "recording_id": "0123456789abcdef",
               "created_at": 1.0,
               "source": {"operation_id": "op", "identity": dict(IDENTITY),
                          "identity_sha256": teach.identity_fingerprint(IDENTITY),
                          "boot_id": "boot-A", "command_id": 7, "started_at": 100.0,
                          "limits_deg": limits_for(axes), "reference_speed_rad_s": 1.0,
                          "driver_kp": 0.0, "driver_kd": 5.0,
                          "driver_gravity_ff_alpha": 1.0, "driver_gravity_ff_ramp_s": 3.0},
               "quality": {"frames": len(samples), "raw_frames": len(samples),
                           "duplicate_frames": 0, "time_backwards": 0, "waiting_frames": 0,
                           "duration_s": duration_s, "sample_rate_hz": count / duration_s,
                           "min_gap_s": step, "max_gap_s": step, "mean_gap_s": step,
                           "max_age_s": 0.005, "window_close_reason": "正常停止",
                           "window_normal_end": True, "interrupted": None},
               "samples": samples, "complete": True, "reasons": []}
    capture["content_sha256"] = teach.content_sha256(capture)
    return capture


def make_teaching(axes=4, **kwargs):
    capture = make_capture(axes=axes, **kwargs)
    payload = {"schema": teach.SCHEMA_TEACHING, "id": "fedcba9876543210",
               "recording_id": capture["recording_id"], "name": "离线教学",
               "created_at": 1.0, "saved_at": 2.0, "source": capture["source"],
               "quality": capture["quality"], "samples": capture["samples"],
               "complete": True, "reasons": [],
               "content_sha256": capture["content_sha256"]}
    return teach.validate_teaching(payload)


def rehash(capture):
    capture["content_sha256"] = teach.content_sha256(capture)
    return capture


class AxesBase(unittest.TestCase):
    def setUp(self):
        teach.configure_axes(4)
        self.addCleanup(teach.configure_axes, 4)


# ---------------------------------------------------------------- plan boundary
class PlanBoundary(AxesBase):
    def test_five_axis_plan_has_five_target_values(self):
        points, windows = planner.compile_plan(five_document(), [0.0] * 5, FIVE_BOUNDS)
        self.assertEqual(len(windows[0]["target_rad"]), 5)
        self.assertAlmostEqual(math.degrees(windows[0]["target_rad"][4]), 5.0)
        self.assertTrue(all(len(window["target_rad"]) == 5 for window in windows))

    def test_legacy_four_axis_plan_is_refused_by_five_axis_caller(self):
        with self.assertRaises(ValueError):
            planner.compile_plan(legacy_document(), [0.0] * 5, FIVE_BOUNDS)

    def test_five_axis_plan_is_refused_by_four_axis_caller(self):
        with self.assertRaises(ValueError):
            planner.compile_plan(five_document(), [0.0] * 4, FOUR_BOUNDS)

    def test_wrong_dimensions_are_never_padded_or_truncated(self):
        for degrees in ((1, 2, 3, 4), (1, 2, 3, 4, 5, 6)):
            with self.subTest(degrees=degrees), self.assertRaises(ValueError):
                planner.compile_plan(five_document(degrees=degrees), [0.0] * 5, FIVE_BOUNDS)
        for degrees in ((1, 2, 3), (1, 2, 3, 4, 5)):
            with self.subTest(degrees=degrees), self.assertRaises(ValueError):
                planner.compile_plan(legacy_document(degrees=degrees), [0.0] * 4, FOUR_BOUNDS)
        # Wrong axis count of feedback/limits never silently zips to a shorter row.
        with self.assertRaises(ValueError):
            planner.compile_plan(five_document(), [0.0] * 5, FOUR_BOUNDS)
        with self.assertRaises(ValueError):
            planner.compile_plan(legacy_document(), [0.0] * 4, FIVE_BOUNDS)

    def test_four_axis_plan_shape_is_unchanged(self):
        points, windows = planner.compile_plan(legacy_document(), [0.0] * 4, FOUR_BOUNDS)
        self.assertEqual(len(windows[0]["target_rad"]), 4)
        self.assertEqual(len(points), 3)


# ---------------------------------------------------------------- client axes
class ProtectedClientAxes(unittest.TestCase):
    def test_canonical_four_axis_bounds_are_unchanged(self):
        bounds = client.load_axis_bounds(4, URDF)
        self.assertEqual(len(bounds), 4)
        for got, want in zip(bounds, FOUR_BOUNDS):
            self.assertAlmostEqual(got[0], want[0], places=12)
            self.assertAlmostEqual(got[1], want[1], places=12)

    def test_five_axis_bounds_append_only_the_explicit_j5_limits(self):
        bounds = client.load_axis_bounds(5, URDF, J5_LIMITS_DEG)
        self.assertEqual(len(bounds), 5)
        for got, want in zip(bounds[:4], FOUR_BOUNDS):
            self.assertAlmostEqual(got[0], want[0], places=12)
            self.assertAlmostEqual(got[1], want[1], places=12)
        self.assertAlmostEqual(bounds[4][0], math.radians(J5_LIMITS_DEG[0]), places=12)
        self.assertAlmostEqual(bounds[4][1], math.radians(J5_LIMITS_DEG[1]), places=12)

    def test_missing_or_invalid_j5_limits_are_refused_not_inferred(self):
        for value in (None, (30.0, -30.0), (0.0, 0.0), (float("nan"), 30.0),
                      (float("inf"), 30.0)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                client.load_axis_bounds(5, URDF, value)
        with self.assertRaises(ValueError):
            client.load_axis_bounds(4, URDF, J5_LIMITS_DEG)
        with self.assertRaises(ValueError):
            client.load_axis_bounds(3, URDF, J5_LIMITS_DEG)

    def run_main(self, argv):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), \
                mock.patch.object(sys, "argv", ["commission_supported_trial.py"] + argv):
            with self.assertRaises(SystemExit) as caught:
                client.main()
        return caught.exception.code, stderr.getvalue()

    def test_cli_requires_explicit_j5_limits_for_five_axes(self):
        code, stderr = self.run_main(["--joint-count", "5"])
        self.assertEqual(code, 2)
        self.assertIn("--j5-limits-deg", stderr)

    def test_cli_refuses_j5_limits_for_four_axes(self):
        code, stderr = self.run_main(["--joint-count", "4", "--j5-limits-deg", "-30", "30"])
        self.assertEqual(code, 2)
        self.assertIn("只用于五轴", stderr)

    def test_cli_refuses_five_axis_capture_and_calibration_until_modules_are_axis_aware(self):
        for extra in (["--calibrate"], ["--supported-static-capture"]):
            with self.subTest(extra=extra):
                code, stderr = self.run_main(
                    ["--joint-count", "5", "--j5-limits-deg", "-30", "30"] + extra)
                self.assertEqual(code, 2)
                self.assertIn("五轴", stderr)

    def test_cli_rejects_unsupported_joint_count(self):
        code, stderr = self.run_main(["--joint-count", "6"])
        self.assertEqual(code, 2)
        self.assertIn("invalid choice", stderr)

    def test_cli_advertises_dynamic_axis_arguments(self):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout), \
                mock.patch.object(sys, "argv", ["commission_supported_trial.py", "--help"]):
            with self.assertRaises(SystemExit) as caught:
                client.main()
        self.assertEqual(caught.exception.code, 0)
        help_text = stdout.getvalue()
        self.assertIn("--joint-count", help_text)
        self.assertIn("--j5-limits-deg", help_text)


# ---------------------------------------------------------------- schema switch
class TeachingSchemaVersions(AxesBase):
    def test_default_import_state_is_four_axis_version_one(self):
        self.assertEqual(DEFAULT_JOINT_COUNT, 4)
        self.assertEqual(DEFAULT_SCHEMAS, ("slaihand_drag_teaching_capture/1",
                                          "slaihand_drag_teaching/1",
                                          "slaihand_drag_teaching_playback/1"))

    def test_four_axis_records_stay_version_one(self):
        teach.configure_axes(4)
        self.assertEqual(teach.JOINT_COUNT, 4)
        self.assertEqual(teach.SCHEMA_CAPTURE, "slaihand_drag_teaching_capture/1")
        self.assertEqual(teach.SCHEMA_TEACHING, "slaihand_drag_teaching/1")
        self.assertEqual(teach.SCHEMA_PLAYBACK, "slaihand_drag_teaching_playback/1")
        capture = teach.validate_capture(make_capture(axes=4))
        self.assertEqual(len(capture["samples"][0]["q"]), 4)
        self.assertEqual(teach.teaching_summary(make_teaching(axes=4))["axes"], 4)
        compiled = teach.compile_playback(make_teaching(axes=4), [0.0] * 4, FOUR_BOUNDS,
                                          0.5, identity=IDENTITY)
        self.assertEqual(len(compiled["frozen"]["samples"][0]), 5)

    def test_five_axis_configure_switches_to_version_two(self):
        teach.configure_axes(5)
        self.assertEqual(teach.JOINT_COUNT, 5)
        self.assertEqual(teach.SCHEMA_CAPTURE, "slaihand_drag_teaching_capture/2")
        self.assertEqual(teach.SCHEMA_TEACHING, "slaihand_drag_teaching/2")
        self.assertEqual(teach.SCHEMA_PLAYBACK, "slaihand_drag_teaching_playback/2")
        capture = teach.validate_capture(make_capture(axes=5))
        self.assertEqual(len(capture["samples"][0]["q"]), 5)
        self.assertEqual(len(capture["samples"][0]["motor"]), 5)
        payload = make_teaching(axes=5)
        self.assertEqual(teach.teaching_summary(payload)["axes"], 5)

    def test_configure_axes_rejects_unknown_counts(self):
        for value in (3, 6, True, "5", None):
            with self.subTest(value=value), self.assertRaises(teach.TeachingError) as caught:
                teach.configure_axes(value)
            self.assertEqual(caught.exception.code, "AXES")
        teach.configure_axes(5)
        teach.configure_axes(4)
        self.assertEqual(teach.JOINT_COUNT, 4)
        self.assertEqual(teach.SCHEMA_CAPTURE, "slaihand_drag_teaching_capture/1")

    def test_five_axis_store_roundtrip_writes_version_two(self):
        teach.configure_axes(5)
        capture = teach.validate_capture(make_capture(axes=5))
        with tempfile.TemporaryDirectory() as directory:
            store = teach.TeachingStore(directory)
            recording_id = store.write_raw(capture)
            loaded = store.load_raw(recording_id)
            self.assertEqual(loaded["schema"], teach.SCHEMA_CAPTURE)
            self.assertEqual(len(loaded["samples"][0]["q"]), 5)
            teaching = store.save(loaded, "五轴教学")
            stored = store.load(teaching["id"])
            self.assertEqual(stored["schema"], teach.SCHEMA_TEACHING)
            self.assertEqual(len(stored["samples"][0]["q"]), 5)
            self.assertEqual(store.list()[0]["axes"], 5)


class TeachingLegacyRejection(AxesBase):
    def test_version_one_records_are_rejected_in_five_axis_mode(self):
        teach.configure_axes(4)
        legacy_capture = make_capture(axes=4)
        legacy_teaching = make_teaching(axes=4)
        legacy_frozen = teach.compile_playback(
            legacy_teaching, [0.0] * 4, FOUR_BOUNDS, 0.5, identity=IDENTITY)["frozen"]
        teach.configure_axes(5)
        for validate, payload, code in (
                (teach.validate_capture, legacy_capture, "SCHEMA"),
                (teach.validate_teaching, legacy_teaching, "SCHEMA"),
                (teach.validate_frozen, legacy_frozen, "SCHEMA")):
            with self.subTest(validate=validate.__name__), \
                    self.assertRaises(teach.TeachingError) as caught:
                validate(payload)
            self.assertEqual(caught.exception.code, code)

    def test_version_one_files_are_rejected_by_five_axis_store(self):
        teach.configure_axes(4)
        capture = teach.validate_capture(make_capture(axes=4))
        with tempfile.TemporaryDirectory() as directory:
            store = teach.TeachingStore(directory)
            recording_id = store.write_raw(capture)
            teaching = store.save(capture, "四轴教学")
            playback = teach.compile_playback(capture and make_teaching(axes=4),
                                              [0.0] * 4, FOUR_BOUNDS, 0.5,
                                              identity=IDENTITY)["frozen"]
            playback_path = Path(directory) / "legacy_playback.json"
            import json
            playback_path.write_text(json.dumps(playback), encoding="utf-8")
            teach.configure_axes(5)
            with self.assertRaises(teach.TeachingError):
                store.load_raw(recording_id)
            with self.assertRaises(teach.TeachingError):
                store.load(teaching["id"])
            with self.assertRaises(teach.TeachingError):
                teach.load_playback(playback_path)

    def test_version_two_records_are_rejected_in_four_axis_mode(self):
        teach.configure_axes(5)
        capture = make_capture(axes=5)
        teaching = make_teaching(axes=5)
        frozen = teach.compile_playback(teaching, [0.0] * 5, FIVE_BOUNDS, 0.5,
                                        identity=IDENTITY)["frozen"]
        teach.configure_axes(4)
        with self.assertRaises(teach.TeachingError):
            teach.validate_capture(capture)
        with self.assertRaises(teach.TeachingError):
            teach.validate_teaching(teaching)
        with self.assertRaises(teach.TeachingError):
            teach.validate_frozen(frozen)


# ---------------------------------------------------------------- dimensions
class TeachingDimensions(AxesBase):
    def test_five_axis_recorder_records_five_dimension_samples(self):
        teach.configure_axes(5)
        recorder = teach.TeachingRecorder(FIVE_BOUNDS, FIVE_LIMITS_DEG)
        rows = five_frames(120)
        feed(recorder, rows)
        recorder.record(9999, rows[-1]["at"] + 0.02, stop_frame(5), IDENTITY)
        capture = recorder.finalize(True, None, "op-test", 1.0)
        self.assertTrue(capture["complete"], capture["reasons"])
        self.assertEqual(capture["schema"], teach.SCHEMA_CAPTURE)
        self.assertEqual(len(capture["samples"]), 120)
        self.assertEqual(len(capture["samples"][0]["q"]), 5)
        self.assertEqual(capture["samples"][0]["motor"], [1] * 5)
        self.assertEqual(len(capture["source"]["limits_deg"]), 5)
        teach.validate_capture(capture)

    def test_recorder_refuses_wrong_axis_bounds_and_ignores_wrong_axis_frames(self):
        teach.configure_axes(5)
        with self.assertRaises(teach.TeachingError):
            teach.TeachingRecorder(FOUR_BOUNDS, FIVE_LIMITS_DEG)
        with self.assertRaises(teach.TeachingError):
            teach.TeachingRecorder(FIVE_BOUNDS, FOUR_LIMITS_DEG)
        recorder = teach.TeachingRecorder(FIVE_BOUNDS, FIVE_LIMITS_DEG)
        four_frame = {"q": [0.0] * 4, "dq": [0.0] * 4, "age": [0.0] * 4, "valid": [1] * 4,
                      "motor": [1] * 4, "stage": 5, "float": True, "hold_ready": True,
                      "command_id": 7}
        recorder.record(1, 0.0, four_frame, IDENTITY)
        self.assertEqual(recorder.frames, [])
        self.assertEqual(recorder.display()["frames"], 0)
        # A truncated frame must close a running window instead of being padded.
        recorder = teach.TeachingRecorder(FIVE_BOUNDS, FIVE_LIMITS_DEG)
        feed(recorder, five_frames(60))
        recorder.record(999, 0.62, four_frame, IDENTITY)
        capture = recorder.finalize(True, None, "op-test", 1.0)
        self.assertFalse(capture["complete"])
        self.assertTrue(any("5 维" in reason or "维度" in reason for reason in capture["reasons"]))

    def test_four_axis_recorder_shape_is_unchanged(self):
        teach.configure_axes(4)
        recorder = teach.TeachingRecorder(FOUR_BOUNDS, FOUR_LIMITS_DEG)
        rows = [{"seq": row["seq"], "at": row["at"],
                 "frame": dict(row["frame"], q=row["frame"]["q"][:4],
                               dq=row["frame"]["dq"][:4], age=row["frame"]["age"][:4],
                               valid=row["frame"]["valid"][:4],
                               motor=row["frame"]["motor"][:4])}
                for row in five_frames(60)]
        feed(recorder, rows)
        recorder.record(9999, rows[-1]["at"] + 0.02, stop_frame(4), IDENTITY)
        capture = recorder.finalize(True, None, "op-test", 1.0)
        self.assertTrue(capture["complete"], capture["reasons"])
        self.assertEqual(len(capture["samples"][0]["q"]), 4)

    def test_version_two_capture_dimension_errors_are_refused(self):
        teach.configure_axes(5)
        for mutate, code in (
                (lambda row: row.update(q=row["q"][:4]), "SHAPE"),
                (lambda row: row.update(q=row["q"] + [0.0]), "SHAPE"),
                (lambda row: row.update(motor=[1] * 4), "SAMPLE"),
                (lambda row: row.update(age=[0.005] * 4), "SHAPE")):
            capture = make_capture(axes=5)
            mutate(capture["samples"][3])
            rehash(capture)
            with self.subTest(code=code), self.assertRaises(teach.TeachingError) as caught:
                teach.validate_capture(capture)
            self.assertEqual(caught.exception.code, code)
        capture = make_capture(axes=5)
        capture["source"]["limits_deg"] = list(FOUR_LIMITS_DEG)
        rehash(capture)
        with self.assertRaises(teach.TeachingError) as caught:
            teach.validate_capture(capture)
        self.assertEqual(caught.exception.code, "SOURCE")

    def test_five_axis_playback_compiles_and_never_pads(self):
        teach.configure_axes(5)
        start = [0.0] * 5
        teaching = make_teaching(axes=5)
        compiled = teach.compile_playback(teaching, start, FIVE_BOUNDS, 0.5,
                                          identity=IDENTITY)
        preview = compiled["preview"]
        self.assertEqual(len(preview["ranges_deg"]), 5)
        self.assertEqual(len(preview["approach_delta_deg"]), 5)
        self.assertEqual(len(compiled["frozen"]["samples"][0]), 6)
        self.assertEqual(teach.validate_frozen(compiled["frozen"])["schema"],
                         teach.SCHEMA_PLAYBACK)
        points, meta = teach.build_trajectory(compiled["frozen"], start, FIVE_BOUNDS)
        self.assertTrue(all(len(point["q"]) == 5 for point in points))
        self.assertEqual(len(meta["ranges_deg"]), 5)
        tracking = teach.tracking_error_deg(
            points, [(point["t"], list(point["q"])) for point in points], 0.0)
        self.assertEqual(len(tracking["max_tracking_error_deg"]), 5)
        for bad_start in ([0.0] * 4, [0.0] * 6):
            with self.subTest(bad_start=len(bad_start)), \
                    self.assertRaises(teach.TeachingError):
                teach.compile_playback(teaching, bad_start, FIVE_BOUNDS, 0.5,
                                       identity=IDENTITY)
            with self.assertRaises(teach.TeachingError):
                teach.build_trajectory(compiled["frozen"], bad_start, FIVE_BOUNDS)
        with self.assertRaises(teach.TeachingError):
            teach.compile_playback(teaching, start, FOUR_BOUNDS, 0.5, identity=IDENTITY)
        with self.assertRaises(teach.TeachingError):
            teach.build_trajectory(compiled["frozen"], start, FOUR_BOUNDS)

    def test_four_axis_playback_still_compiles(self):
        teach.configure_axes(4)
        compiled = teach.compile_playback(make_teaching(axes=4), [0.0] * 4, FOUR_BOUNDS,
                                          0.5, identity=IDENTITY)
        points, meta = teach.build_trajectory(compiled["frozen"], [0.0] * 4, FOUR_BOUNDS)
        self.assertTrue(all(len(point["q"]) == 4 for point in points))
        self.assertEqual(len(meta["ranges_deg"]), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
