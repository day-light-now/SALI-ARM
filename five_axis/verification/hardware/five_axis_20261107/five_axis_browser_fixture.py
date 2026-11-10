"""Serve the real five-axis page with a transport-only fake backend (no ROS/serial)."""
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
from test_five_axis_panel_http import FakeBackend  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402

TOKEN = "<redacted-panel-token>"
URL = HERE / "five_axis_browser_url.txt"


class UiBackend(FakeBackend):
    def status(self):
        data = super().status()
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        data["axis_calibration"] = {
            "schema": "slaihand_axis_calibration_workflow/1", "formal": False,
            "joints": [{"name": name, "label": "J%d" % index}
                       for index, name in enumerate(names, 1)],
            "phases": [{"name": "zero", "label": "零位读数"},
                       {"name": "positive", "label": "正方向移动"},
                       {"name": "lower", "label": "下限端点"},
                       {"name": "upper", "label": "上限端点"}],
            "phase": "idle", "phase_label": "未开始", "selected_joint": None,
            "selected_label": None, "pending": [], "captures": {},
            "candidate": None, "session_identity": {}, "session_id": None,
            "live": {"disarmed": True}, "identity": {"fresh": False},
            "start_reasons": ["离线 UI 夹具：未连接驱动"],
            "capabilities": {"start": {"ok": False, "message": "离线夹具"},
                             "capture": {"ok": False, "message": "离线夹具"},
                             "abort": {"ok": True, "message": "始终可用"}},
        }
        return data


def main():
    backend = UiBackend()
    server = PanelServer(backend, TOKEN, HERE.parents[2] / "src/slaihand_bringup/web/control_panel", 18897)
    server.start()
    URL.write_text("http://127.0.0.1:%d/?token=%s" % (server.port, TOKEN) + "\n")
    print("READY", URL.read_text().strip(), flush=True)
    try:
        while True:
            time.sleep(1)
    finally:
        server.stop()


if __name__ == "__main__":
    main()
