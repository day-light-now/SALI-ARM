#!/usr/bin/env bash
# Manual smoke check: panel starts, serves auth-protected status against an
# isolated domain (176, no driver here). Never touches domain 0 or a real port.
# Only the panel PID this script started is ever signalled.
set -e
cd "$(dirname "$0")"
source ./test_env.sh
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1
PANEL=/home/xuyu/slaihand_control_panel_ws_20261104/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py
PORT=${SLAIHAND_SMOKE_PORT:-18999}
rm -rf /tmp/panel-smoke
python3 "$PANEL" --port "$PORT" --archive-dir /tmp/panel-smoke >/tmp/panel-smoke.log 2>&1 &
PANEL_PID=$!
trap 'kill "$PANEL_PID" 2>/dev/null || true' EXIT
sleep 5
echo "--- panel log"
cat /tmp/panel-smoke.log
URL=$(sed -n 's/.*\(http:\/\/127\.0\.0\.1:[0-9]*\/?token=[A-Za-z0-9_-]*\).*/\1/p' /tmp/panel-smoke.log | head -1)
echo "URL=$URL"
PORT_FOUND=$(printf '%s' "$URL" | sed -n 's#.*:\([0-9]*\)/?token.*#\1#p')
TOKEN=$(printf '%s' "$URL" | sed -n 's#.*token=##p')
PORT="$PORT_FOUND" TOKEN="$TOKEN" python3 - <<'PY'
import json, os, urllib.request
port, token = os.environ["PORT"], os.environ["TOKEN"]
base = "http://127.0.0.1:%s" % port

def get(path, headers):
    request = urllib.request.Request(base + path, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as error:
        return error.code, error.read().decode()

print("no-token status:", get("/api/status", {})[0])
status, text = get("/api/status", {"X-Panel-Token": token})
payload = json.loads(text)
print(json.dumps({"ros": payload["ros"], "server": {k: payload["server"][k] for k in
                  ("ros_domain_id", "ros_thread_alive", "closing")},
                  "axes": [{k: axis[k] for k in ("label", "actual_deg", "target_deg", "feedback_valid")}
                           for axis in payload["axes"]]}, ensure_ascii=False)[:1400])
print("page:", get("/?token=" + token, {})[0])
PY
echo SMOKE_DONE
