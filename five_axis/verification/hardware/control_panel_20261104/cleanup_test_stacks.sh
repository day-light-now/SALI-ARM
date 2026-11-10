#!/usr/bin/env bash
# Clean up ROS processes that belong to THIS task's isolated test domains.
# A process is killed only when both hold:
#   - exact ROS_DOMAIN_ID=176/177 plus this panel harness's session marker
#   - its command line is a ROS node/launch/spawner, not a test script
# This never touches the real stack (domain 0) or another domain's panels.
set -u
echo "=== candidates on isolated domains"
mapfile -t CANDIDATES < <(
  for pid in /proc/[0-9]*; do
    p=${pid#/proc/}
    [ "$p" = "$$" ] && continue
    env=$({ tr '\0' '\n' < "$pid/environ"; } 2>/dev/null || true)
    printf '%s\n' "$env" | grep -Eq '^ROS_DOMAIN_ID=(176|177)$' || continue
    printf '%s\n' "$env" | grep -Eq '^SLAIHAND_PANEL_TEST_SESSION=[0-9a-f]{32}$' || continue
    cmd=$({ tr '\0' ' ' < "$pid/cmdline"; } 2>/dev/null || true)
    case "$cmd" in
      *ros2*|*ros2_control_node*|*robot_state_publisher*|*controller_manager*|*spawner*) ;;
      *) continue ;;
    esac
    echo "$p|$cmd"
  done
)
for item in "${CANDIDATES[@]:-}"; do
  [ -z "$item" ] && continue
  echo "${item%%|*}: ${item#*|}" | cut -c1-180
done
echo "=== terminating"
for item in "${CANDIDATES[@]:-}"; do
  [ -z "$item" ] && continue
  pid="${item%%|*}"
  echo "TERM $pid"
  kill -TERM "$pid" 2>/dev/null || true
done
sleep 2
for item in "${CANDIDATES[@]:-}"; do
  [ -z "$item" ] && continue
  pid="${item%%|*}"
  if [ -d "/proc/$pid" ]; then
    echo "KILL $pid"
    kill -KILL "$pid" 2>/dev/null || true
  fi
done
sleep 1
echo "=== remaining on 176/177"
for pid in /proc/[0-9]*; do
  p=${pid#/proc/}
  env=$({ tr '\0' '\n' < "$pid/environ"; } 2>/dev/null || true)
  printf '%s\n' "$env" | grep -Eq '^ROS_DOMAIN_ID=(176|177)$' || continue
  printf '%s\n' "$env" | grep -Eq '^SLAIHAND_PANEL_TEST_SESSION=[0-9a-f]{32}$' || continue
  echo "$p: $(tr '\0' ' ' < "$pid/cmdline" 2>/dev/null | cut -c1-120)"
done
echo "=== real serial stack (must remain)"
ps -eo pid,etime,cmd | grep "dev/serial/by-id" | grep -v grep || echo "REAL_STACK_NOT_FOUND"
