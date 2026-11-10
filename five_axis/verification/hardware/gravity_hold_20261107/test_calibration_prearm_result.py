"""Pre-arm failure must leave a diagnostic result file, never a silent exit.

Runs the real protected client with no ROS driver on isolated domain 176. The
client cannot obtain four-axis feedback, so it must refuse before arm and still
write a result JSON that distinguishes "未请求使能 / 未开始轨迹 / 未发送停止".
"""
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CLIENT = ROOT / "src/slaihand_bringup/scripts/commission_supported_trial.py"
PLAN = {"coordinates": "absolute_deg",
        "waypoints": [{"degrees": [0.0, 0.0, 0.0, 0.0], "move_s": 7.5, "dwell_s": 6.0}]}


def main():
    assert os.environ.get("ROS_DOMAIN_ID", "176") not in ("", "0")
    directory = Path(tempfile.mkdtemp(prefix="prearm-result-"))
    plan = directory / "plan.json"
    output = directory / "client"
    plan.write_text(json.dumps(PLAN), encoding="utf-8")
    command = [sys.executable, str(CLIENT), "--execute", "--ack-supported",
               "--ack-clearance", "--ack-native-output", "--plan", str(plan),
               "--calibrate", "--ack-free-static", "--output-directory", str(output)]
    started = time.monotonic()
    result = subprocess.run(command, text=True, capture_output=True, timeout=40,
                            cwd=str(ROOT), env=dict(os.environ))
    files = sorted(output.glob("*.json")) if output.is_dir() else []
    assert files, ("client exited without a result file", result.returncode,
                   result.stdout[-2000:], result.stderr[-2000:])
    payload = json.loads(files[-1].read_text(encoding="utf-8"))
    assert result.returncode != 0, payload
    assert payload["arm_requested"] is False, payload
    assert payload["trajectory_requested"] is False, payload
    assert payload["stop_requested"] is False, payload
    assert payload["phase"] == "pre_arm", payload
    assert payload["phase_result"] == "未请求使能", payload
    assert payload["stop_result"] == "未请求停止", payload
    assert payload["prearm_reasons"], payload
    assert isinstance(payload.get("prearm_observations"), dict), payload
    evidence = {"passed": True, "returncode": result.returncode,
                "elapsed_s": round(time.monotonic() - started, 2),
                "result_file": str(files[-1]),
                "phase": payload["phase"], "phase_result": payload["phase_result"],
                "stop_result": payload["stop_result"],
                "prearm_reasons": payload["prearm_reasons"],
                "prearm_observations": payload.get("prearm_observations"),
                "domain": os.environ.get("ROS_DOMAIN_ID"), "physical_port_opened": False}
    path = Path(__file__).resolve().parent / "evidence/calibration_prearm_result.json"
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(evidence, indent=2), encoding="utf-8")
    print("PASS", json.dumps(evidence, ensure_ascii=False))


if __name__ == "__main__":
    main()
