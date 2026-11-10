#!/bin/bash
# Verify the shared environment sources cleanly and resolves relative paths.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
echo "pwd=$(pwd)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "buffered=$RCUTILS_LOGGING_BUFFERED_STREAM"
ros2 pkg prefix slaihand_bringup
test -f src/slaihand_bringup/scripts/commission_supported_trial.py && echo SRC_PATHS_OK
