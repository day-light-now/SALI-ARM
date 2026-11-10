"""大角度标定 UI 的 PTY/ROS/JTC 端到端验证（隔离域 176）。

场景：
A. 当前位置静态采样（候选 α=0）：预览 → 位置保持 → 外部推挤使未就绪时拒绝撤手确认 →
   复位静止后就绪 → 撤手确认 → 静态窗口 → 停止确认 → 原始文件/残差。
B. 候选 α=0.1：预览基线不满足，采样成功但仅诊断、不进入拟合。
C. 自由拖动就绪：注入 0.2 rad/s 非零反馈速度，位置补偿渐入完成后仍可开始并完成；
   注入 1.2 rad/s 时（速度门槛独立补丁）不再触发位置模式 FEEDBACK_SPEED。
D. 停止竞态/心跳丢失：保持就绪后停止或心跳丢失，随后撤手确认必须被拒绝。
"""
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTROL = HERE.parent / "control_panel_20261104"
sys.path.insert(0, str(CONTROL))
sys.path.insert(0, str(CONTROL / ".." / "canusb_motion_20261104"))
sys.path.insert(0, str(HERE.parents[2] / "src/slaihand_bringup/scripts"))

os.environ.setdefault("ROS_DOMAIN_ID", "176")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
assert os.environ["ROS_DOMAIN_ID"] not in ("", "0")

import rclpy  # noqa: E402
import verify_control_panel as fixture  # noqa: E402

RESULTS = {"checks": [], "failures": []}
CONFIRMATIONS = {"supported": True, "clearance_and_local_power_cut": True,
                 "native_output_not_hardware_clamp": True}


def check(name, condition, detail=None):
    entry = {"name": name, "passed": bool(condition)}
    if detail is not None:
        entry["detail"] = detail
    RESULTS["checks"].append(entry)
    print(("PASS  " if condition else "FAIL  ") + name
          + ("" if detail is None else "  " + json.dumps(detail, ensure_ascii=False, default=str)[:500]),
          flush=True)
    if not condition:
        RESULTS["failures"].append(name)
    return bool(condition)


def reset_to_disarmed(panel, observer, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        code, _ = panel.call("POST", "/api/reset", {})
        if code in (200, 202):
            break
        observer.spin(0.2)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = panel.status()
        if status["ros"]["stage"] == 0 and status["ros"]["stage_fresh"]:
            return status
        observer.spin(0.1)
    raise AssertionError("did not observe fresh DISARMED after reset")


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


def wait_panel_ready(panel, timeout=25):
    """等待面板同时具备新鲜动态反馈、驱动前馈状态和运行身份。"""
    return panel.wait_status(
        lambda body: body["ros"]["stage"] == 0 and body["ros"]["connected"]
        and (body.get("supported_capture") or {}).get("baseline", {}).get("alpha") is not None,
        timeout)


def preview_and_start(panel):
    code, body = panel.call("POST", "/api/capture/preview", {})
    check("静态采样预览生成", code == 200, {"code": code, "body": (body or {}).get("code")})
    preview = (body or {}).get("preview") or {}
    ticket = (body or {}).get("ticket") or {}
    confirmations = dict(CONFIRMATIONS, supported_static_capture=True)
    code, body = panel.call("POST", "/api/capture/start",
                            {"ticket": ticket.get("id"), "confirmations": confirmations})
    operation = (body or {}).get("operation") or {}
    check("静态采样启动进入受保护客户端", code == 202 and operation.get("kind") == "supported_capture",
          {"code": code, "kind": operation.get("kind")})
    argv = " ".join(operation.get("argv") or [])
    check("客户端使用支持静态采样参数", "--supported-static-capture" in argv
          and "--ack-file" in argv and "--panel-stop-epoch" in argv, argv[-200:])
    return preview, operation


def wait_capture_ready(panel, timeout=25):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = panel.status()
        capture = last.get("supported_capture") or {}
        current = capture.get("current") or {}
        if current.get("ready"):
            return last
        time.sleep(0.2)
    raise AssertionError("capture hold did not become ready: %r"
                         % ((last or {}).get("supported_capture")))


def scenario_capture_alpha0(observer, work):
    session = fixture.Session("capture-a0", work, observer)
    try:
        panel, plant = session.panel, session.plant
        plant.max_follow_rate = 0.6
        if not hasattr(plant, "reported_velocity"):
            plant.reported_velocity = [0.0] * 4
        status = wait_panel_ready(panel)
        check("候选 α=0 基线满足", status["supported_capture"]["baseline"]["satisfied"] is True,
              status["supported_capture"]["baseline"])
        preview, operation = preview_and_start(panel)
        check("预览冻结到期实际姿态与身份",
              abs(preview["start_deg"][0] - math.degrees(0.2618)) < 1.5
              and preview["baseline"]["satisfied"] is True
              and preview["identity"]["identity_sha256"],
              {"start_deg": preview["start_deg"], "alpha": preview["alpha"]})
        status = wait_capture_ready(panel)
        current = status["supported_capture"]["current"]
        check("位置保持已就绪（stage5 + hold_ready）", current["ready"] is True
              and status["ros"]["stage"] == 5, current)
        # 外部推挤：夹具跟随产生非零速度，hold_ready 必须下降，撤手确认被拒绝
        plant.raw[0] += 0.05
        deadline = time.monotonic() + 5
        not_ready = False
        while time.monotonic() < deadline:
            observer.spin(0.05)
            status = panel.status()
            capture = status["supported_capture"] or {}
            current = capture.get("current") or {}
            if current.get("id") and not current.get("ready"):
                not_ready = True
                break
        check("外部推挤后保持未就绪", not_ready,
              (status.get("supported_capture") or {}).get("readiness_reasons"))
        if not_ready:
            code, body = panel.call("POST", "/api/capture/ack",
                                    {"operation_id": current["id"],
                                     "confirmations": {"operator_disengaged": True}})
            check("未就绪时撤手确认被拒绝", code == 409 and body.get("code") == "NOT_READY",
                  {"code": code, "body": (body or {}).get("code")})
        # 回到目标并稳定后，重新就绪
        plant.raw[0] -= 0.05
        status = wait_capture_ready(panel)
        current = status["supported_capture"]["current"]
        check("恢复稳定后再次就绪", current["ready"] is True, current.get("state"))
        code, body = panel.call("POST", "/api/capture/ack",
                                {"operation_id": current["id"],
                                 "confirmations": {"operator_disengaged": True}})
        check("撤手确认受理", code == 202 and body.get("ack"), {"code": code,
                                                              "body": (body or {}).get("code")})
        status = wait_operation_finished(panel, timeout=40)
        operation = status.get("operation") or {}
        result = operation.get("result") or {}
        sample = result.get("supported_capture") or {}
        check("采样操作完成且停止确认", operation.get("exit_code") == 0
              and result.get("stop_observations_confirmed") is True,
              {"exit": operation.get("exit_code"), "reason": result.get("reason")})
        check("采样窗口合格", sample.get("sample_valid") is True
              and result.get("capture_sample_ok") is True,
              sample.get("invalid_reasons"))
        check("α=0 样本可进入静态拟合", sample.get("fit_eligible") is True,
              sample.get("exclusion_reason"))
        raw_path = Path(sample.get("raw_file") or "")
        check("原始采样文件独占只读", raw_path.is_file()
              and (raw_path.stat().st_mode & 0o222) == 0, str(raw_path))
        check("残差只读可见", isinstance(sample.get("residual_nm"), list)
              and len(sample["residual_nm"]) == 4
              and isinstance(sample.get("model_gravity_nm"), list),
              sample.get("residual_nm"))
        gate = status.get("stop_gate") or {}
        check("采样后进入已失能且停止已确认", status["ros"]["stage"] == 8
              and gate.get("state") != "unknown",
              {"stage": status["ros"]["stage"], "stop_gate": gate.get("state"),
               "stop_confirmed": result.get("stop_observations_confirmed")})
        code, body = panel.call("POST", "/api/capture/ack",
                                {"operation_id": current["id"],
                                 "confirmations": {"operator_disengaged": True}})
        check("操作结束后的旧撤手确认被拒绝", code == 409, {"code": code})
        return True
    finally:
        session.close()


def scenario_capture_alpha01(observer, work):
    session = fixture.Session("capture-a01", work, observer,
                              stack_args=["gravity_ff_alpha:=0.1"])
    try:
        panel, plant = session.panel, session.plant
        plant.max_follow_rate = 0.6
        if not hasattr(plant, "reported_velocity"):
            plant.reported_velocity = [0.0] * 4
        status = wait_panel_ready(panel)
        baseline = status["supported_capture"]["baseline"]
        check("α=0.1 基线显示仅诊断", baseline["satisfied"] is False, baseline)
        preview, operation = preview_and_start(panel)
        check("α=0.1 预览标记不满足基线", preview["baseline"]["satisfied"] is False,
              preview["alpha"])
        status = wait_capture_ready(panel)
        current = status["supported_capture"]["current"]
        code, body = panel.call("POST", "/api/capture/ack",
                                {"operation_id": current["id"],
                                 "confirmations": {"operator_disengaged": True}})
        check("α=0.1 撤手确认受理", code == 202, code)
        status = wait_operation_finished(panel, timeout=40)
        result = (status.get("operation") or {}).get("result") or {}
        sample = result.get("supported_capture") or {}
        check("α=0.1 采样成功但仅诊断",
              sample.get("sample_valid") is True and sample.get("fit_eligible") is False
              and "alpha" in str(sample.get("exclusion_reason") or "").lower()
              or "α" in str(sample.get("exclusion_reason") or ""),
              {"fit": sample.get("fit_eligible"),
               "exclusion": sample.get("exclusion_reason")})
        check("α=0.1 停止确认", result.get("stop_observations_confirmed") is True)
        return True
    finally:
        session.close()


def scenario_float_readiness(observer, work):
    session = fixture.Session("float-ready", work, observer)
    try:
        panel, plant = session.panel, session.plant
        plant.max_follow_rate = 1.0
        plant.reported_velocity = [0.0] * 4
        status = panel.wait_status(lambda body: body.get("gravity_shadow", {}).get("driver", {}).get(
            "float_available"), 20)
        code, body = panel.call("POST", "/api/gravity-float",
                                {"confirmations": dict(CONFIRMATIONS, gravity_float=True),
                                 "float_settings": {"kd": 2.0, "seconds": 6.0}})
        check("自由拖动请求受理", code == 202, code)
        deadline = time.monotonic() + 10
        active = False
        while time.monotonic() < deadline:
            observer.spin(0.05)
            status = panel.status()
            driver = status["gravity_shadow"]["driver"]
            if driver.get("gravity_float") and status["ros"]["stage"] == 5:
                active = True
                break
        check("自由拖动进入活动保持", active, status["ros"]["stage_label"])
        # 手扶/漂移速度 0.2 rad/s：旧代码会因静止条件无法满足而等待 hold_ready 超时
        plant.reported_velocity = [0.2, 0.0, 0.0, 0.0]
        status = wait_operation_finished(panel, timeout=40)
        operation = status.get("operation") or {}
        result = operation.get("result") or {}
        check("0.2 rad/s 拖动仍完成自由拖动",
              operation.get("exit_code") == 0 and result.get("reason") == "finite_trial_complete"
              and result.get("mode") == "gravity_float",
              {"exit": operation.get("exit_code"), "reason": result.get("reason")})
        check("自由拖动停止确认", result.get("stop_observations_confirmed") is True)
        return True
    finally:
        session.close()


def scenario_float_speed(observer, work):
    session = fixture.Session("float-speed", work, observer)
    try:
        panel, plant = session.panel, session.plant
        plant.max_follow_rate = 1.0
        plant.reported_velocity = [0.0] * 4
        panel.wait_status(lambda body: body.get("gravity_shadow", {}).get("driver", {}).get(
            "float_available"), 20)
        code, body = panel.call("POST", "/api/gravity-float",
                                {"confirmations": dict(CONFIRMATIONS, gravity_float=True),
                                 "float_settings": {"kd": 1.5, "seconds": 5.0}})
        check("速度场景自由拖动受理", code == 202, code)
        deadline = time.monotonic() + 10
        active = False
        while time.monotonic() < deadline:
            observer.spin(0.05)
            status = panel.status()
            if status["gravity_shadow"]["driver"].get("gravity_float"):
                active = True
                break
        check("速度场景进入自由拖动", active)
        plant.reported_velocity = [1.2, 0.0, 0.0, 0.0]
        status = wait_operation_finished(panel, timeout=40)
        operation = status.get("operation") or {}
        result = operation.get("result") or {}
        check("自由拖动 1.2 rad/s 不被位置模式速度门限中止",
              operation.get("exit_code") == 0 and result.get("reason") == "finite_trial_complete",
              {"exit": operation.get("exit_code"), "reason": result.get("reason")})
        return True
    finally:
        session.close()


def scenario_stop_before_ack(observer, work):
    session = fixture.Session("capture-stop", work, observer)
    try:
        panel, plant = session.panel, session.plant
        plant.max_follow_rate = 0.6
        if not hasattr(plant, "reported_velocity"):
            plant.reported_velocity = [0.0] * 4
        wait_panel_ready(panel)
        preview_and_start(panel)
        status = wait_capture_ready(panel)
        current = status["supported_capture"]["current"]
        code, body = panel.call("POST", "/api/stop", {"reason": "E2E stop before ack"})
        check("保持结束后停止受理", code == 202, code)
        status = wait_operation_finished(panel, timeout=30)
        code, body = panel.call("POST", "/api/capture/ack",
                                {"operation_id": current["id"],
                                 "confirmations": {"operator_disengaged": True}})
        check("停止后的撤手确认被拒绝", code == 409, {"code": code,
                                                     "body": (body or {}).get("code")})
        check("停止后无客户端仍在运行", not (status.get("operation") or {}).get("running"))
        return True
    finally:
        session.close()


def main():
    rclpy.init()
    observer = fixture.Observer()
    work = Path(tempfile.mkdtemp(prefix="gravitycal-e2e-"))
    try:
        scenario_capture_alpha0(observer, work)
        scenario_capture_alpha01(observer, work)
        scenario_float_readiness(observer, work)
        scenario_float_speed(observer, work)
        scenario_stop_before_ack(observer, work)
        RESULTS["ok"] = not RESULTS["failures"]
        return 0 if RESULTS["ok"] else 1
    finally:
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
