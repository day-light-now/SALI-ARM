"""当前位置静态采样：纯协议/基线/一次性 ack 反例（无 ROS）。"""
import json
import math
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import supported_capture as sc  # noqa: E402
import drag_teaching as dt  # noqa: E402

HASH = "a" * 64
IDENTITY = {"schema": "slaihand_runtime_identity/1", "boot_id": "boot-A", "started_at": 1.0,
            "mapping_sha256": HASH, "model_sha256": HASH, "policy_sha256": HASH,
            "mass_profile_sha256": HASH, "policy_profile": "p", "load_id": "no_end_load",
            "gravity_base_m_s2": [0.0, 9.80665, 0.0], "j3_offset_rad": math.pi,
            "kp": 160.0, "kd": 5.0, "reference_speed_rad_s": 1.0,
            "gravity_ff_alpha": 0.0, "gravity_ff_ramp_s": 1.0}
IDENTITY_SHA = dt.identity_fingerprint(IDENTITY)
TESTS = []


def test(function):
    TESTS.append(function)
    return function


def expected(**overrides):
    base = {"operation_id": "op-1", "command_id": 4, "boot_id": "boot-A",
            "identity_sha256": IDENTITY_SHA, "panel_stop_generation": 2}
    base.update(overrides)
    return base


@test
def test_start_confirmations():
    full = {"supported": True, "clearance_and_local_power_cut": True,
            "native_output_not_hardware_clamp": True, "supported_static_capture": True}
    assert sc.validate_start_confirmations(full)["supported_static_capture"] is True
    for missing in ("supported", "clearance_and_local_power_cut",
                    "native_output_not_hardware_clamp", "supported_static_capture"):
        broken = dict(full)
        broken[missing] = False
        try:
            sc.validate_start_confirmations(broken)
            raise AssertionError("missing %s accepted" % missing)
        except sc.CaptureError as error:
            assert error.code == "CONFIRMATION"
    try:
        sc.validate_ack_confirmations({})
        raise AssertionError("ack without operator confirmation accepted")
    except sc.CaptureError as error:
        assert error.code == "DISENGAGE_ACK"


@test
def test_baseline_eligibility():
    zero = sc.fit_baseline(0.0)
    assert zero["satisfied"] is True
    other = sc.fit_baseline(0.1)
    assert other["satisfied"] is False and "诊断" in other["note"]
    unknown = sc.fit_baseline(None)
    assert unknown["satisfied"] is False


@test
def test_ack_roundtrip_and_validation():
    ack = sc.build_ack("op-1", 4, "boot-A", IDENTITY_SHA, 2, alpha=0.0, stage=5,
                       hold_ready=True, nonce="0123456789abcdef")
    assert sc.validate_ack(ack, expected())["nonce"] == "0123456789abcdef"
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / sc.ACK_FILENAME
        sc.write_ack_file(path, ack)
        loaded = sc.read_ack_file(path)
        assert loaded["command_id"] == 4
        # 再写一次只覆盖同一路径（面板只在未发送时写）；这里验证原子替换可用。
        sc.write_ack_file(path, ack)
        assert json.loads(path.read_text())["schema"] == sc.SCHEMA_ACK


@test
def test_ack_rejections():
    ack = sc.build_ack("op-1", 4, "boot-A", IDENTITY_SHA, 2, alpha=0.0, stage=5,
                       hold_ready=True)
    cases = [
        (dict(ack, operation_id="op-2"), "IDENTITY"),
        (dict(ack, command_id=5), "IDENTITY"),
        (dict(ack, boot_id="boot-B"), "IDENTITY"),
        (dict(ack, identity_sha256="b" * 64), "IDENTITY"),
        (dict(ack, panel_stop_generation=3), "IDENTITY"),
        (dict(ack, operator_disengaged=False), "DISENGAGE_ACK"),
        (dict(ack, hold_ready=False), "NOT_READY"),
        (dict(ack, stage=8), "STAGE"),
        (dict(ack, mode="gravity_float"), "MODE"),
        (dict(ack, support="supported"), "SUPPORT"),
        (dict(ack, requested_at=time.time() - 60), "STALE"),
        (dict(ack, nonce="zz"), "SCHEMA"),
        (dict(ack, alpha=None), "SCHEMA"),
    ]
    for payload, code in cases:
        try:
            sc.validate_ack(payload, expected())
            raise AssertionError("accepted invalid ack %r" % (payload,))
        except sc.CaptureError as error:
            assert error.code == code, (error.code, code)
    broken = dict(ack)
    broken.pop("nonce")
    try:
        sc.validate_ack(broken, expected())
        raise AssertionError("missing field accepted")
    except sc.CaptureError as error:
        assert error.code == "SCHEMA"


@test
def test_ack_corrupt_file():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / sc.ACK_FILENAME
        path.write_text("not json", encoding="utf-8")
        try:
            sc.read_ack_file(path)
            raise AssertionError("corrupt ack accepted")
        except sc.CaptureError as error:
            assert error.code == "CORRUPT"


@test
def test_preview_payload_and_bounds():
    preview = sc.preview_payload([0.1, 0.2, 0.3, 0.4], IDENTITY, 0.0,
                                 [[-80, 200], [-10, 190], [-90, 90], [-50, 90]], "urdf")
    assert preview["schema"] == sc.SCHEMA_PREVIEW
    assert preview["start_deg"] == [5.73, 11.459, 17.189, 22.918]
    assert preview["baseline"]["satisfied"] is True
    assert preview["identity"]["boot_id"] == "boot-A"
    assert preview["timing"]["ack_timeout_s"] == sc.ACK_TIMEOUT_S
    assert preview["identity"]["identity_sha256"] == IDENTITY_SHA
    diagnostic = sc.preview_payload([0, 0, 0, 0], IDENTITY, 0.1,
                                    [[-80, 200], [-10, 190], [-90, 90], [-50, 90]])
    assert diagnostic["baseline"]["satisfied"] is False


def main():
    failures = []
    for function in TESTS:
        try:
            function()
            print("PASS  %s" % function.__name__)
        except Exception as error:  # noqa: BLE001
            failures.append(function.__name__)
            print("FAIL  %s: %s: %s" % (function.__name__, type(error).__name__, error))
    print("RESULT: %s (%d/%d)" % ("FAIL" if failures else "PASS",
                                  len(TESTS) - len(failures), len(TESTS)))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
