#!/usr/bin/env bash
# Start the browser-check fixture detached (isolated domain 176, PTY drive).
# No pattern killing: an existing fixture is stopped only after PID/PGID and
# identity verification by stop_browser_fixture.sh.
set -e
cd "$(dirname "$0")"
source ./test_env.sh
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1
if [ -f /tmp/panel-browser-fixture/fixture.json ]; then
  echo "existing fixture state found; stopping the verified instance first"
  bash ./stop_browser_fixture.sh || true
fi
rm -f browser_url.txt
setsid nohup python3 browser_fixture.py >/tmp/browser_fixture.log 2>&1 < /dev/null &
sleep 1
echo FIXTURE_STARTED
