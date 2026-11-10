"""拖动示教 E2E：真实 PanelBackend/HTTP + 真实 ROS/JTC + 新鲜 PTY（域 176）。

覆盖：自由拖动记录 → 停止 → 质量 → 命名保存 → 现有载入列表 → 载入 →
预览（接近段/时长/范围/峰值）→ 回放（正常位置控制，非零刚度）→ 停止门禁 →
心跳丢失 → 停止竞态 → 保存失败不影响停止。
"""
import json
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTROL = HERE.parent / "control_panel_20261104"
sys.path.insert(0, str(CONTROL))
sys.path.insert(0, str(HERE.parents[2] / "src" / "slaihand_bringup" / "scripts"))

os.environ.setdefault("ROS_DOMAIN_ID", "176")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
assert os.environ["ROS_DOMAIN_ID"] not in ("", "0")

import rclpy  # noqa: E402
import verify_control_panel as fixture  # noqa: E402

RESULTS = {"checks": [], "failures": []}


def check(name, condition, detail=None):
    entry = {"name": name, "passed": bool(condition)}
    if detail is not None:
        entry["detail"] = detail
    RESULTS["checks"].append(entry)
    print(("PASS  " if condition else "FAIL  ") + name
          + ("" if detail is None else "  " + json.dumps(detail, ensure_ascii=False, default=str)[:600]),
          flush=True)
    if not condition:
        RESULTS["failures"].append(name)
    return bool(condition)


def confirmations(**extra):
    return fixture.confirmations(**extra)


def wait_operation_finished(panel, timeout=40):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = panel.status()
        operation = last.get("operation") or {}
        if operation and not operation.get("running") and operation.get("phase") not in (
                "starting", "running", "stopping"):
            return last
        time.sleep(0.2)
    raise AssertionError("operation did not finish: %r" % (last and last.get("operation")))


def audit_rows(session):
    rows = []
    for path in sorted((session.dir / "audit").glob("*.jsonl")):
        try:
            with path.open(encoding="utf-8", errors="replace") as stream:
                for line in stream:
                    try:
                        rows.append(json.loads(line))
                    except ValueError:
                        continue
        except OSError:
            continue
    return rows


def kp_of(row):
    payload = bytes.fromhex(row["payload"])
    return ((((payload[3] & 15) << 8) | payload[4]) * 500.0 / 4095.0)


def mode_packets(session, start_index):
    rows = audit_rows(session)
    out = []
    for row in rows[start_index:]:
        if row.get("kind") == "TX_WRITTEN" and row.get("role") == 1:
            out.append(kp_of(row))
    return out, len(rows)


def reset_to_disarmed(panel, observer, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        code, body = panel.call("POST", "/api/reset", {})
        if code in (202, 200):
            break
        observer.spin(0.2)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = panel.status()
        if status["ros"]["stage"] == 0 and status["ros"]["stage_fresh"]:
            return status
        observer.spin(0.1)
    raise AssertionError("did not observe fresh DISARMED after reset")


def move_plant(session, observer, seconds=1.4, speed_deg_s=10.0):
    """在自由拖动中模拟操作员拖动：直接移动假夹具姿态。"""
    plant = session.plant
    steps = max(10, int(seconds / 0.02))
    delta = math.radians(speed_deg_s) * 0.02
    moved = False
    for index in range(steps):
        plant.raw[0] += delta
        plant.raw[3] += delta * 0.4
        observer.spin(0.02)
        moved = True
    return moved


def main():
    rclpy.init()
    observer = fixture.Observer()
    session = None
    work = Path(tempfile.mkdtemp(prefix="drag-teaching-e2e-"))
    try:
        session = fixture.Session("drag", work, observer)
        panel, plant = session.panel, session.plant
        status = panel.wait_status(
            lambda body: body.get("teaching", {}).get("available") is True, timeout=20)
        teaching = status["teaching"]
        check("候选驱动开放自由拖动", teaching["available"] is True, teaching.get("driver"))
        check("默认回放速度比例 0.5", abs(teaching["default_speed_factor"] - 0.5) < 1e-9,
              teaching["default_speed_factor"])
        check("回放峰值硬上限 1 rad/s", abs(teaching["speed_limit_rad_s"] - 1.0) < 1e-9,
              teaching["speed_limit_rad_s"])
        check("记录前未使能", status["ros"]["stage"] == 0, status["ros"]["stage"])

        # -- 确认门禁：缺拖动示教确认 / 缺自由拖动确认都被服务端拒绝 ----------
        code, body = panel.call("POST", "/api/teaching/record/start",
                                {"confirmations": confirmations()})
        check("缺少拖动示教确认被拒绝", code == 400 and body.get("code") == "TEACHING_ACK",
              {"code": code, "body": body})
        code, body = panel.call("POST", "/api/teaching/record/start",
                                {"confirmations": confirmations(drag_teaching=True)})
        check("缺少自由拖动确认被拒绝", code == 400 and body.get("code") == "FLOAT_ACK",
              {"code": code, "body": body})
        status = panel.status()
        check("被拒绝的请求没有启动操作", status.get("operation") is None)

        # -- 记录：自由拖动 + 模拟拖动 + 停止记录 ----------------------------
        audit_index = len(audit_rows(session))
        code, rejected = panel.call("POST", "/api/teaching/record/start",
            {"confirmations": confirmations(gravity_float=True, drag_teaching=True),
             "float_settings": {"kd": 5.1, "seconds": 20.0}})
        check("超出协议范围的 Kd 在启动前被拒绝", code == 400 and rejected.get("code") == "FLOAT_SETTINGS")
        code, body = panel.call("POST", "/api/teaching/record/start",
                                {"confirmations": confirmations(gravity_float=True,
                                                                drag_teaching=True),
                                 "float_settings": {"kd": 2.0, "seconds": 20.0}})
        check("开始记录受理", code == 202 and (body.get("operation") or {}).get("kind") == "drag_teaching",
              {"code": code, "body": body})
        argv = " ".join((body.get("operation") or {}).get("argv") or [])
        check("记录复用自由拖动客户端参数", "--gravity-float" in argv
              and "--ack-gravity-float" in argv, argv[-160:])
        status = panel.wait_status(
            lambda body: (body.get("teaching") or {}).get("phase") in ("ramp_in", "recording"),
            timeout=12)
        moved = False
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            status = panel.status()
            phase = (status.get("teaching") or {}).get("phase")
            if phase == "recording":
                moved = move_plant(session, observer, seconds=1.3, speed_deg_s=12.0) or moved
                break
            if phase in ("ready", "incomplete", "idle"):
                break
            observer.spin(0.05)
        check("进入自由拖动有效记录阶段", moved, (status.get("teaching") or {}).get("phase"))
        driver = status["teaching"]["driver"]
        check("驱动确认自定义 Kd 与持续时间", driver.get("float_kd") == 2.0
              and driver.get("float_seconds") == 20.0, driver)
        code, body = panel.call("POST", "/api/teaching/record/stop", {})
        check("停止记录走既有停止流程", code == 202 and body.get("phase") == "stop_requested",
              {"code": code, "phase": body.get("phase")})
        status = wait_operation_finished(panel, timeout=25)
        operation = status.get("operation") or {}
        check("记录操作结束且确认新失能", operation.get("exit_code") is not None
              and (operation.get("result") or {}).get("stop_observations_confirmed") is True,
              {"exit": operation.get("exit_code"),
               "result": (operation.get("result") or {}).get("reason")})
        status = panel.wait_status(
            lambda body: ((body.get("teaching") or {}).get("last") or {}).get("recording_id"),
            timeout=15)
        last = status["teaching"]["last"]
        check("记录完整并落盘", last["complete"] is True, last.get("reasons")[:4])
        quality = last.get("quality") or {}
        check("记录帧数/时长/间隙达标",
              int(quality.get("frames") or 0) >= 30 and float(quality.get("duration_s") or 0) >= 1.0
              and float(quality.get("max_gap_s") or 1) <= 0.05,
              {k: quality.get(k) for k in ("frames", "duration_s", "max_gap_s", "sample_rate_hz")})
        check("记录携带 boot_id 与 command_id",
              bool(last.get("boot_id")) and isinstance(last.get("command_id"), int),
              {"boot_id": last.get("boot_id"), "command_id": last.get("command_id")})
        kps, audit_index = mode_packets(session, audit_index)
        check("记录阶段全部零刚度报文", len(kps) > 100 and all(abs(kp) < 1e-6 for kp in kps),
              {"packets": len(kps), "max_kp": max(kps) if kps else None})
        damping = []
        for row in audit_rows(session):
            if row.get("kind") == "TX_WRITTEN" and row.get("role") == 1:
                payload = bytes.fromhex(row["payload"])
                damping.append(((payload[5] << 4) | (payload[6] >> 4)) * 5.0 / 4095.0)
        check("真实 MIT 报文达到 Kd=2", bool(damping) and abs(max(damping) - 2.0) < .002,
              {"peak_encoded_kd": max(damping) if damping else None})
        check("记录后处于已失能且停止门禁锁定",
              status["ros"]["stage"] == 8 and status["stop_gate"]["locked"] is True,
              {"stage": status["ros"]["stage"], "gate": status["stop_gate"]})

        # -- 保存 / 列表 / 载入 --------------------------------------------
        recording_id = last["recording_id"]
        code, body = panel.call("POST", "/api/teaching/save",
                                {"recording_id": recording_id, "name": ""})
        check("空名称保存被拒绝（停止已完成）", code == 400,
              {"code": code, "body": body.get("code")})
        code, body = panel.call("POST", "/api/teaching/save",
                                {"recording_id": recording_id, "name": "E2E 拖动示教"})
        saved = body.get("teaching") or {}
        teaching_id = saved.get("id")
        check("命名保存成功", code == 201 and teaching_id, {"code": code, "id": teaching_id})
        code, body = panel.call("GET", "/api/plans")
        entries = [row for row in body.get("plans") or [] if row.get("id") == teaching_id]
        check("示教出现在现有载入列表", code == 200 and entries
              and entries[0].get("kind") == "teaching" and entries[0].get("name") == "E2E 拖动示教",
              entries[:1])
        code, body = panel.call("GET", "/api/plans/" + teaching_id)
        check("现有载入接口可载入示教", code == 200 and body.get("kind") == "teaching"
              and body.get("id") == teaching_id)
        code, body = panel.call("GET", "/api/teachings")
        check("示教列表接口可用", code == 200 and any(
            row.get("id") == teaching_id for row in body.get("teachings") or []))

        # -- 停止门禁：未复位前拒绝预览；复位后预览 ------------------------
        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 0.5})
        check("停止门禁锁定回放预览", code == 409 and body.get("code") == "STOP_LOCKED",
              {"code": code, "body": body.get("code")})
        status = reset_to_disarmed(panel, observer)

        # -- 篡改 / 版本不一致：预览必须拒绝（不静默重映射） -----------------
        import drag_teaching as teach
        saved_path = session.dir / "archive" / "teachings" / (teaching_id + ".json")
        original = saved_path.read_text(encoding="utf-8")
        saved_payload = json.loads(original)
        tampered = json.loads(original)
        tampered["samples"][4]["q"][0] += 0.05
        saved_path.write_text(json.dumps(tampered, ensure_ascii=False), encoding="utf-8")
        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 0.5})
        check("篡改的教学文件被拒绝", code == 400 and body.get("code") == "INTEGRITY",
              {"code": code, "body": body.get("code")})
        mismatched = json.loads(original)
        mismatched["source"]["identity"]["mapping_sha256"] = "b" * 64
        mismatched["source"]["identity_sha256"] = teach.identity_fingerprint(
            mismatched["source"]["identity"])
        mismatched["content_sha256"] = teach.content_sha256(
            {"schema": teach.SCHEMA_CAPTURE, "source": mismatched["source"],
             "quality": mismatched["quality"], "samples": mismatched["samples"],
             "complete": mismatched["complete"], "reasons": mismatched["reasons"]})
        saved_path.write_text(json.dumps(mismatched, ensure_ascii=False), encoding="utf-8")
        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 0.5})
        check("映射哈希不一致的回放被拒绝", code == 409 and body.get("code") == "TEACHING_VERSION",
              {"code": code, "body": body.get("code")})
        saved_path.write_text(original, encoding="utf-8")
        check("原始教学文件已恢复", json.loads(saved_path.read_text(encoding="utf-8")) == saved_payload)

        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 0.5})
        preview = body.get("preview") or {}
        ticket = body.get("ticket") or {}
        check("回放预览生成", code == 200 and ticket.get("mode") == "teaching", {"code": code})
        check("预览含接近段/原时长/重定时总时长",
              float(preview.get("approach_s") or 0) > 0
              and float(preview.get("original_duration_s") or 0) > 0
              and abs(float(preview.get("retimed_duration_s") or 0)
                      - (float(preview.get("approach_s") or 0)
                         + float(preview.get("original_duration_s") or 0) / 0.5)) < 0.05,
              {k: preview.get(k) for k in ("approach_s", "original_duration_s",
                                           "retimed_duration_s")})
        check("预览含关节范围与峰值速度",
              len(preview.get("ranges_deg") or []) == 4
              and float(preview.get("peak_speed_rad_s") or 9) <= 1.0 + 1e-9,
              {"peak": preview.get("peak_speed_rad_s"), "ranges": preview.get("ranges_deg")})
        expected_identity = (preview.get("runtime_identity") or {}).get("identity_sha256")
        check("预览票根绑定运行身份", bool(expected_identity), expected_identity)
        check("预览未使能", panel.status()["ros"]["stage"] == 0)
        # 速度比例超限反例：记录峰值×比例不得超过 1 rad/s。
        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 1.0})
        if code == 200:
            check("比例 1.0 预览峰值仍 ≤1 rad/s",
                  float((body.get("preview") or {}).get("peak_speed_rad_s") or 9) <= 1.0 + 1e-9)
            panel.call("POST", "/api/teaching/preview",
                       {"teaching_id": teaching_id, "speed_factor": 0.5})
        else:
            check("比例 1.0 超速被拒绝", body.get("code") in ("SPEED", "SPEED_FACTOR"),
                  body.get("code"))
        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 0.5})
        ticket = body.get("ticket") or {}

        # -- 回放：正常位置控制（Kp=160），不是零刚度 ---------------------
        playback_index = len(audit_rows(session))
        code, body = panel.call("POST", "/api/teaching/execute",
                                {"ticket": ticket.get("id"), "confirmations": confirmations()})
        check("回放执行受理", code == 202, {"code": code, "body": body.get("code")})
        argv = " ".join((body.get("operation") or {}).get("argv") or [])
        check("回放不使用 --gravity-float", "--teaching-plan" in argv
              and "--gravity-float" not in argv, argv[-160:])
        status = wait_operation_finished(panel, timeout=60)
        operation = status.get("operation") or {}
        result = operation.get("result") or {}
        check("回放完成且确认新失能", operation.get("exit_code") == 0
              and result.get("stop_observations_confirmed") is True,
              {"exit": operation.get("exit_code"), "reason": result.get("reason")})
        check("回放结果模式为 drag_teaching", result.get("mode") == "drag_teaching",
              result.get("mode"))
        playback = result.get("teaching_playback") or {}
        check("回放记录速度比例与峰值",
              abs(float(playback.get("speed_factor") or 0) - 0.5) < 1e-9
              and float(playback.get("peak_speed_rad_s") or 9) <= 1.0 + 1e-9,
              {k: playback.get(k) for k in ("speed_factor", "peak_speed_rad_s",
                                            "retimed_duration_s")})
        check("客户端使能前核对了运行身份",
              result.get("runtime_identity_sha256") == expected_identity,
              {"client": result.get("runtime_identity_sha256"), "preview": expected_identity})
        tracking = result.get("playback_tracking") or {}
        check("回放跟踪误差已记录", isinstance(tracking.get("max_tracking_error_deg"), list),
              tracking.get("max_tracking_error_deg"))
        kps, _ = mode_packets(session, playback_index)
        median = sorted(kps)[len(kps) // 2] if kps else None
        check("回放阶段为正常位置控制（Kp≈160）",
              len(kps) > 100 and median is not None and median > 150
              and sum(1 for kp in kps if kp > 150) > 100,
              {"packets": len(kps), "median_kp": median,
               "position_packets": sum(1 for kp in kps if kp > 150),
               "zero_stiffness_packets": sum(1 for kp in kps if kp < 1e-6)})
        check("回放后已失能且未复位不能再次预览",
              status["ros"]["stage"] == 8 and status["ros"]["disabled_observed"] is True,
              {"stage": status["ros"]["stage"]})
        code, body = panel.call("POST", "/api/teaching/preview",
                                {"teaching_id": teaching_id, "speed_factor": 0.5})
        check("回放结束后新预览被拒绝", code == 409, {"code": code, "body": body.get("code")})

        reset_to_disarmed(panel, observer)
        code, body = panel.call("POST", "/api/gravity-float",
            {"confirmations": confirmations(gravity_float=True),
             "float_settings": {"kd": 1.5, "seconds": 2.0}})
        check("可调自由拖动计时体验受理", code == 202)
        running_id = (body.get("operation") or {}).get("id")
        code, rejected = panel.call("POST", "/api/teaching/record/start",
            {"confirmations": confirmations(gravity_float=True, drag_teaching=True),
             "float_settings": {"kd": 3.0, "seconds": 10.0}})
        check("活动期间不能替换本次拖动设置", code == 409 and rejected.get("code") == "BUSY"
              and panel.status()["operation"]["id"] == running_id)
        timed = wait_operation_finished(panel, timeout=20)["operation"]
        check("自定义持续时间结束后自动失能", timed.get("exit_code") == 0
              and (timed.get("result") or {}).get("stop_observations_confirmed") is True,
              timed.get("result"))
        check("客户端保存了驱动确认的设置", (timed.get("result") or {}).get("float_settings")
              == {"kd": 1.5, "seconds": 2.0, "confirmed": True})

        # -- 心跳丢失：记录被停止，不产生迟到启动 --------------------------
        reset_to_disarmed(panel, observer)
        code, body = panel.call("POST", "/api/teaching/record/start",
                                {"confirmations": confirmations(gravity_float=True,
                                                                drag_teaching=True)})
        check("心跳丢失场景记录已启动", code == 202, code)
        session.heartbeat.stop()
        deadline = time.monotonic() + 12
        status = None
        while time.monotonic() < deadline:
            status = panel.status()
            operation = status.get("operation") or {}
            if operation and not operation.get("running") and status["stop"]["reason"]:
                break
            observer.spin(0.1)
        stop_reason = (status or {}).get("stop", {}).get("reason") or ""
        check("心跳丢失触发有界停止", "心跳" in stop_reason, stop_reason)
        check("心跳丢失后停止门禁锁定", (status or {}).get("stop_gate", {}).get("locked") is True)
        session.heartbeat = fixture.Heartbeat(panel)
        session.heartbeat.start()
        panel.heartbeat()
        time.sleep(0.3)

        # -- 停止竞态：启动与停止并发，不产生停止后的迟到启动 --------------
        reset_to_disarmed(panel, observer)
        race = {}

        def start_request():
            race["start"] = panel.call("POST", "/api/teaching/record/start",
                                       {"confirmations": confirmations(gravity_float=True,
                                                                       drag_teaching=True)})

        def stop_request():
            race["stop"] = panel.call("POST", "/api/teaching/record/stop", {})

        thread_start = threading.Thread(target=start_request)
        thread_stop = threading.Thread(target=stop_request)
        thread_start.start()
        thread_stop.start()
        thread_start.join(10)
        thread_stop.join(10)
        time.sleep(0.6)
        status = panel.status()
        operation = status.get("operation") or {}
        check("并发启动/停止被安全处理",
              status["stop_gate"]["locked"] is True and not operation.get("running"),
              {"start": race.get("start", (None,))[0] if isinstance(race.get("start"), tuple) else None,
               "operation": bool(operation), "gate": status["stop_gate"]["locked"]})
        check("停止后没有新的记录操作在运行",
              operation.get("kind") != "drag_teaching" or operation.get("finished") is not False,
              operation.get("kind"))

        # -- 保存失败不影响已完成的停止 ------------------------------------
        code, body = panel.call("POST", "/api/teaching/save",
                                {"recording_id": last["recording_id"], "name": ""})
        status = panel.status()
        operation = status.get("operation") or {}
        check("保存失败仍保持停止语义", code == 400
              and status["ros"]["stage"] in (0, 8)
              and operation.get("running") is not True
              and (status.get("stop") or {}).get("state") in ("confirmed", "unknown", "requested"),
              {"code": code, "stop": (status.get("stop") or {}).get("state"),
               "stage": status["ros"]["stage"], "running": operation.get("running")})

        RESULTS["ok"] = not RESULTS["failures"]
        return 0 if RESULTS["ok"] else 1
    finally:
        try:
            if session is not None:
                session.close()
        except Exception as error:  # noqa: BLE001
            print("cleanup session error:", error)
        try:
            fixture.wait_domain_clean(observer)
        except Exception:
            pass
        observer.close()
        rclpy.shutdown()


if __name__ == "__main__":
    code = main()
    print("RESULT: %s (%d/%d checks)" % ("PASS" if code == 0 else "FAIL",
                                          sum(1 for item in RESULTS["checks"] if item["passed"]),
                                          len(RESULTS["checks"])))
    raise SystemExit(code)
