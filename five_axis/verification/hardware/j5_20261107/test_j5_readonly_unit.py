"""Pure J5 codec/register/conflict tests; no serial descriptor opened."""
import math
import tempfile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
from j5_readonly import (J5ManualCalibration, J5ProtocolError, J5State,
                         REGS, feedback_frame, request_payload, usb_packet)


def reg_frame(rid, value):
    import struct
    data = bytes([5, 0, 0x33, rid]) + (struct.pack("<f", value)
                                      if rid in (21, 22, 23) else struct.pack("<I", int(value)))
    return feedback_frame(0, data)


_TICKS = [0]


def make_sample(pos, pmax=12.5, session="session-a"):
    _TICKS[0] += 1
    profile = {"can_id": 5, "master_id": 0x10, "mode": 1, "timeout": 2000,
               "pmax_rad": pmax, "vmax_rad_s": 25.0, "tmax_nm": 5.0,
               "model_declared": "DM-J4340-2EC", "model_verified": False}
    return {"fresh": True, "state": 0, "stage": 0, "session_id": session,
            "device_path": "/dev/pts/test", "sample_at": float(_TICKS[0]),
            "profile": profile,
            "registers": {}, "feedback": {"raw_position_rad": pos, "state": 0,
                                          "payload": "test-payload" * 3}}


def feedback(pos, can_id=0x10, state=0):
    pmax, vmax, tmax = 12.5, 25.0, 5.0
    p = int(round((pos + pmax) * 65535 / (2 * pmax)))
    data = bytes([(state << 4) | 5, p >> 8, p & 255, 0x80, 0x08, 0x00, 30, 28])
    return feedback_frame(can_id, data)


def main():
    state = J5State()
    assert request_payload(5, 7)[:4] == bytes([5, 0, 0x33, 7])
    for bad in ((4, None), (5, 11)):
        try:
            request_payload(*bad)
            raise AssertionError("invalid J5 request accepted")
        except J5ProtocolError:
            pass
    assert len(usb_packet(5, 7)) == 30 and usb_packet(5, 7)[21:25] == bytes([5, 0, 0x33, 7])
    values = {7: 0x10, 8: 5, 9: 2000, 10: 1, 21: 12.5, 22: 25.0, 23: 5.0}
    now = 100.0
    for rid in REGS:
        state.feed(0, reg_frame(rid, values[rid])[7:15], now)
    assert state.ready() and state.snapshot(now)["profile"]["model_verified"] is False
    assert state.snapshot(now)["hardware_sign"] is None
    assert state.snapshot(now)["zero_offset_rad"] is None
    # A second different reply for the same register is a duplicate-ID conflict.
    state.feed(0, reg_frame(21, 13.0)[7:15], now)
    assert not state.ready() and 21 in state.conflicts
    assert "conflicting" in state.notice

    clean = J5State()
    for rid in REGS:
        clean.feed(0, reg_frame(rid, values[rid])[7:15], now)
    clean.feed(0x10, feedback(math.radians(10))[7:15], now)
    assert clean.ready() and clean.feedback["raw_position_deg"] > 9
    clean.feed(0x11, feedback(math.radians(20), can_id=0x11)[7:15], now)
    assert clean.feedback["raw_position_deg"] > 9  # wrong master ID ignored

    calibration = J5ManualCalibration(Path("/tmp/j5-unit-cal.json"))
    sample = clean.snapshot(now)
    try:
        calibration.capture_zero(sample, confirm_disabled_support=False)
        raise AssertionError("missing disabled/support acknowledgement accepted")
    except J5ProtocolError:
        pass
    calibration.capture_zero(sample, confirm_disabled_support=True)
    try:
        calibration.capture_direction(sample, confirm_manual_rotation=True)
        raise AssertionError("direction without rotation accepted")
    except J5ProtocolError:
        pass
    # Single-turn boundary: 6.25 rad + actual +10deg wraps to .14135; reject.
    with tempfile.TemporaryDirectory() as folder:
        cal = J5ManualCalibration(Path(folder) / "cal.json")
        cal.capture_zero(make_sample(6.25), confirm_disabled_support=True)
        try:
            cal.capture_direction(make_sample(.14135), confirm_manual_rotation=True)
            raise AssertionError("single-turn boundary jump accepted as negative direction")
        except J5ProtocolError as error:
            assert "SINGLE_TURN_BOUNDARY_OR_JUMP" in str(error), error
        # Re-zero clears direction and both limits.
        cal.capture_zero(make_sample(0.0), confirm_disabled_support=True)
        cal.capture_direction(make_sample(.2), confirm_manual_rotation=True)
        cal.capture_limit("min", make_sample(-.5), confirm_cable_limit=True)
        cal.capture_limit("max", make_sample(.5), confirm_cable_limit=True)
        cal.capture_zero(make_sample(.3), confirm_disabled_support=True)
        assert cal.direction is None and cal.limit_min is None and cal.limit_max is None
        # Re-direction clears limits.
        cal.capture_direction(make_sample(.4), confirm_manual_rotation=True)
        assert cal.limit_min is None and cal.limit_max is None
        # Identity/profile change after zero is rejected.
        try:
            cal.capture_limit("min", make_sample(-.5, pmax=25.0), confirm_cable_limit=True)
            raise AssertionError("changed PMAX accepted within one calibration")
        except J5ProtocolError as error:
            assert "DEVICE_IDENTITY_CHANGED" in str(error), error
        # A complete same-identity sequence saves generation/profile identity.
        cal.capture_limit("min", make_sample(-.5), confirm_cable_limit=True)
        cal.capture_limit("max", make_sample(.5), confirm_cable_limit=True)
        saved = cal.save(confirm_save=True)
        assert saved["record"]["calibration_generation"] > 0
        assert saved["record"]["device_identity"]["session_id"] == "session-a"
        assert saved["record"]["decode_profile"]["pmax_rad"] == 12.5

    print("PASS pure J5 codec/register/conflict/calibration guards")


if __name__ == "__main__":
    main()
