#!/bin/bash
# One-click static-compensation pipeline (WSL side).
#
# Usage: compensate.sh <plan.json> <TAG> [run|dry|plan] [hybrid|model|iter]
#   hybrid (default): model-compensated run -> one measured-residual correction
#   model:            model-compensated run only
#   iter:             legacy baseline -> iteration 1 -> iteration 2
#   plan:             offline only: validate + emit the model-compensated plan
# Runs: read-only TIMEOUT admission -> truthful artifact -> candidate driver ->
# selected strategy -> comparison/report -> stop driver.  Never enables gravity FF
# or touches any safety gate; every motion plan starts/ends at absolute zero.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"

if [ "$#" -lt 2 ]; then
  echo "USAGE: compensate.sh <plan.json> <TAG> [run|dry|plan] [hybrid|model|iter]"
  exit 2
fi
PLAN_IN="$1"
TAG="$2"
MODE="${3:-run}"
STRATEGY="${4:-hybrid}"
case "$STRATEGY" in hybrid|model|iter) ;; *) echo "invalid strategy: $STRATEGY"; exit 2 ;; esac
WORK="$HERE/runs/$TAG"
mkdir -p "$WORK" "$HERE/plans"
LOGDIR="$HERE/logs"
mkdir -p "$LOGDIR"

stop_driver_quiet() {
  bash "$HERE/stop_driver.sh" >/dev/null 2>&1 || true
}
trap 'stop_driver_quiet' EXIT

fail() {
  echo "ABORT: $*"
  echo "日志目录: $LOGDIR"
  exit 1
}

# ---- 1. plan validation and staging -------------------------------------
PLAN_ABS="$(cd "$(dirname "$PLAN_IN")" && pwd)/$(basename "$PLAN_IN")" || fail "plan path"
[ -f "$PLAN_ABS" ] || fail "plan not found: $PLAN_IN"
python3 "$HERE/validate_plan.py" "$PLAN_ABS" || fail "invalid plan"
BASE_NAME="${TAG}_base"
BASE_PLAN="$HERE/plans/${BASE_NAME}_20261110.json"
cp -f "$PLAN_ABS" "$BASE_PLAN"
DUR="$(python3 "$HERE/plan_duration.py" "$BASE_PLAN")" || fail "duration"
echo "plan=$PLAN_ABS tag=$TAG mode=$MODE strategy=$STRATEGY recorder_duration=$DUR"

# ---- 1b. offline plan-only mode -----------------------------------------
if [ "$MODE" = "plan" ]; then
  if [ "$STRATEGY" = "iter" ]; then
    echo "PLAN_ONLY: iter strategy needs measured residual; no plan generated"
    exit 0
  fi
  python3 "$HERE/compensate_plan.py" "$BASE_PLAN" "${TAG}_modelcomp" \
    > "$LOGDIR/${TAG}_model_plan.log" 2>&1 \
    || fail "model compensation rejected: $(tail -3 "$LOGDIR/${TAG}_model_plan.log")"
  echo "PLAN_ONLY_OK: plans/${TAG}_modelcomp_20261110.json"
  exit 0
fi

# ---- 2. read-only TIMEOUT admission, then truthful artifact -------------
EVI="$HERE/evidence/comp_${TAG}.json"
python3 src/slaihand_bringup/scripts/j5_timeout_setup.py \
  --port /dev/ttyACM0 --apply-timeout 2000 --confirm EXCLUSIVE_J5_SETUP \
  --audit "$EVI" --verify-existing > "$LOGDIR/${TAG}_verify.log" 2>&1 || true
STATUS="$(python3 "$HERE/parse_verify.py" "$EVI")"
if [ "$STATUS" = "write" ]; then
  echo "TIMEOUT 回退为 0（断电），执行一次写设置 0->2000"
  python3 src/slaihand_bringup/scripts/j5_timeout_setup.py \
    --port /dev/ttyACM0 --apply-timeout 2000 --confirm EXCLUSIVE_J5_SETUP \
    --audit "$EVI" > "$LOGDIR/${TAG}_timeout_write.log" 2>&1 || true
  STATUS="$(python3 "$HERE/parse_verify.py" "$EVI")"
fi
[ "$STATUS" = "ok" ] || fail "TIMEOUT/失能准入失败（$STATUS），见 ${TAG}_verify.log"
cp -f "$EVI" "$HERE/evidence/current_timeout_evidence.json"

BUILD_OUT="$(python3 src/slaihand_bringup/scripts/j5_motion_artifact.py build \
  --mapping src/slaihand_bringup/config/hardware_mapping.yaml \
  --model src/slaihand_description/urdf/slaihand_five_axis.urdf \
  --policy src/slaihand_bringup/config/canusb_five_axis_motion_policy.yaml \
  --load src/slaihand_bringup/config/load_3p5kg_provisional.yaml \
  --evidence "$HERE/evidence/current_timeout_evidence.json" \
  --output "$HERE/artifacts/j5_motion_calibration_artifact_20261110.json" 2>&1)"
echo "$BUILD_OUT" > "$LOGDIR/${TAG}_artifact.log"
echo "$BUILD_OUT" | grep -q '"ok": true' || fail "artifact build failed"

# ---- 3. driver with admission -------------------------------------------
ready_driver() {
  local log count
  log="$(ls -t "$HERE"/logs/driver_launch_*.log 2>/dev/null | head -1)"
  count="$(pgrep -f 'ros2_control_node --ros-args' 2>/dev/null | wc -l)"
  [ "$count" = "1" ] || fail "driver count=$count"
  grep -q '"j5_qualified": true' "$log" 2>/dev/null || fail "admission not J5_QUALIFIED"
}
restart_driver() {
  local count tries
  stop_driver_quiet
  # Wait until both the driver process and the serial port are actually free;
  # launching into a still-held port can abort the node at startup.
  tries=0
  while [ "$tries" -lt 30 ]; do
    count="$(pgrep -f 'ros2_control_node --ros-args' 2>/dev/null | wc -l)"
    if [ "$count" = "0" ] && ! fuser -s /dev/ttyACM0 2>/dev/null; then
      break
    fi
    tries=$((tries + 1))
    sleep 0.5
  done
  count="$(pgrep -f 'ros2_control_node --ros-args' 2>/dev/null | wc -l)"
  [ "$count" = "0" ] || fail "previous driver did not exit (count=$count)"
  fuser -s /dev/ttyACM0 2>/dev/null && fail "serial port still busy before launch"
  bash "$HERE/04_start_driver_bg.sh" > "$LOGDIR/${TAG}_launch.log" 2>&1 || fail "launch failed"
  sleep 12
  ready_driver
}
restart_driver
echo "driver ready (J5_QUALIFIED)"

if [ "$MODE" = "dry" ]; then
  echo "DRY_RUN_OK: admission/artifact/driver verified, no motion performed"
  exit 0
fi

reset_between_runs() {
  # Preferred: software reset through the running panel (does not reopen the
  # USB serial port, which reduces the chance of the known write-fault tail).
  if python3 "$HERE/panel_reset.py" > "$LOGDIR/${TAG}_panel_reset.log" 2>&1; then
    if [ "$(python3 "$HERE/panel_stage.py" 2>/dev/null)" = "0" ]; then
      sleep 1
      return 0
    fi
  fi
  echo "panel reset unavailable; restarting driver"
  restart_driver
}

# ---- 4. baseline + iterations ------------------------------------------
run_one() {
  local plan="$1" tag="$2" dur
  dur="$(python3 "$HERE/plan_duration.py" "$plan")" || fail "$tag: duration"
  echo "run $tag ... (recorder ${dur}s)"
  bash "$HERE/run_plan.sh" "$plan" "$tag" "$dur" > "$LOGDIR/${tag}_run.log" 2>&1 || true
  pgrep -f 'ros2_control_node --ros-args' >/dev/null 2>&1 || \
    fail "$tag: driver died (see ${tag}_run.log)"
  python3 "$HERE/extract_windows.py" "$tag" "$plan" > "$LOGDIR/${tag}_extract.log" 2>&1 || \
    fail "$tag: extraction failed"
  python3 "$HERE/check_windows.py" "$tag" 6 > "$LOGDIR/${tag}_windows.log" 2>&1 || \
    fail "$tag: insufficient windows: $(tail -1 "$LOGDIR/${tag}_windows.log")"
}

if [ "$STRATEGY" != "iter" ]; then
  MODEL_NAME="${TAG}_modelcomp"
  python3 "$HERE/compensate_plan.py" "$BASE_PLAN" "$MODEL_NAME" \
    > "$LOGDIR/${TAG}_model_plan.log" 2>&1 \
    || fail "model compensation rejected: $(tail -3 "$LOGDIR/${TAG}_model_plan.log")"
  MODEL_PLAN="$HERE/plans/${MODEL_NAME}_20261110.json"
  [ -f "$MODEL_PLAN" ] || fail "model-compensated plan missing"
  run_one "$MODEL_PLAN" "${TAG}_model"
  python3 "$HERE/compare_plan_runs.py" "$BASE_NAME" \
    "$WORK/${TAG}_model_compare.json" "${TAG}_model" \
    > "$LOGDIR/${TAG}_model_compare.log" 2>&1 || fail "model comparison failed"
  if [ "$STRATEGY" = "model" ]; then
    python3 "$HERE/summarize_model_comp.py" "$TAG" || fail "model summary failed"
    echo "REPORT: $WORK/${TAG}_REPORT.md"
    echo "DONE tag=$TAG strategy=model"
    exit 0
  fi
  reset_between_runs
  python3 "$HERE/make_iter_comp.py" "${TAG}_model" "$MODEL_NAME" "${TAG}_iter1" \
    > "$LOGDIR/${TAG}_iter1_plan.log" 2>&1 || fail "model+iter1 plan generation failed"
  run_one "$HERE/plans/${TAG}_iter1_20261110.json" "${TAG}_iter1"
  python3 "$HERE/compare_plan_runs.py" "$BASE_NAME" \
    "$WORK/${TAG}_iter1_compare.json" "${TAG}_iter1" \
    > "$LOGDIR/${TAG}_iter1_compare.log" 2>&1 || fail "model+iter1 comparison failed"
  python3 "$HERE/summarize_model_comp.py" "$TAG" || fail "model+iter1 summary failed"
  echo "REPORT: $WORK/${TAG}_REPORT.md"
  echo "DONE tag=$TAG strategy=$STRATEGY"
  exit 0
fi

run_one "$BASE_PLAN" "${TAG}_base"

reset_between_runs
python3 "$HERE/make_iter_comp.py" "${TAG}_base" "${BASE_NAME}" "${TAG}_iter1" \
  > "$LOGDIR/${TAG}_iter1_plan.log" 2>&1 || fail "iteration 1 plan generation failed"
run_one "$HERE/plans/${TAG}_iter1_20261110.json" "${TAG}_iter1"

reset_between_runs
python3 "$HERE/make_iter2.py" "${BASE_NAME}" "${TAG}_iter1" "${TAG}_iter1" "${TAG}_iter2" \
  > "$LOGDIR/${TAG}_iter2_plan.log" 2>&1 || fail "iteration 2 plan generation failed"
run_one "$HERE/plans/${TAG}_iter2_20261110.json" "${TAG}_iter2"

# ---- 5. comparison and report ------------------------------------------
python3 "$HERE/compare_any.py" "${BASE_NAME}" "${TAG}_base" "${TAG}_iter1" \
  "$WORK/${TAG}_iter1_compare.json" > "$LOGDIR/${TAG}_compare1.log" 2>&1 || \
  fail "iteration 1 comparison failed"
python3 "$HERE/compare_any.py" "${BASE_NAME}" "${TAG}_base" "${TAG}_iter2" \
  "$WORK/${TAG}_iter2_compare.json" > "$LOGDIR/${TAG}_compare2.log" 2>&1 || \
  fail "iteration 2 comparison failed"
python3 "$HERE/summarize_comp.py" "$TAG" || fail "summary failed"
echo "REPORT: $WORK/${TAG}_REPORT.md"
echo "DONE tag=$TAG"
