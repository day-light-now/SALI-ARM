#!/usr/bin/env bash
# Stop only the browser fixture this repository started.
# Identity is proven from the recorded PID/PGID plus /proc cmdline+environ;
# no pkill/pattern matching is used, so other panels or fake stacks survive.
set -e
STATE=/tmp/panel-browser-fixture/fixture.json
if [ ! -f "$STATE" ]; then
  echo "no fixture state file; nothing to stop"
  exit 0
fi
read -r PID PGID < <(python3 - "$STATE" <<'PY'
import json, sys
state = json.load(open(sys.argv[1]))
print(state.get("pid"), state.get("pgid"))
PY
)
if [ -z "${PID:-}" ] || [ -z "${PGID:-}" ]; then
  echo "state file incomplete; refusing to kill anything"
  exit 2
fi
if [ ! -d "/proc/$PID" ]; then
  echo "recorded fixture pid $PID is gone; removing stale state"
  rm -f "$STATE"
  exit 0
fi
CMDLINE=$(tr '\0' ' ' < "/proc/$PID/cmdline")
case "$CMDLINE" in
  *browser_fixture.py*) ;;
  *) echo "pid $PID is not the browser fixture ($CMDLINE); refusing" >&2; exit 3 ;;
esac
if ! tr '\0' '\n' < "/proc/$PID/environ" | grep -qx 'ROS_DOMAIN_ID=176'; then
  echo "pid $PID is not running in ROS_DOMAIN_ID=176; refusing" >&2
  exit 4
fi
ACTUAL_PGID=$(ps -o pgid= -p "$PID" | tr -d ' ')
if [ "$ACTUAL_PGID" != "$PGID" ]; then
  echo "pgid mismatch (recorded $PGID, actual $ACTUAL_PGID); refusing" >&2
  exit 5
fi
echo "stopping verified fixture pid=$PID pgid=$PGID"
kill -TERM -- "-$PGID" 2>/dev/null || kill -TERM "$PID"
for _ in $(seq 1 40); do
  [ -d "/proc/$PID" ] || break
  sleep 0.25
done
if [ -d "/proc/$PID" ]; then
  echo "fixture did not exit; killing verified process group"
  kill -KILL -- "-$PGID" 2>/dev/null || kill -KILL "$PID"
fi
rm -f "$STATE"
echo "fixture stopped"
