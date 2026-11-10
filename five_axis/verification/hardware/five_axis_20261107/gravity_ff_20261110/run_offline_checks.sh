#!/bin/bash
# Offline regression checks: compile, unit tests, suite dry validation, and a
# synthetic end-to-end run of the analysis pipeline. No driver, no motion.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
fail=0

echo "=== 1. python compile ==="
python3 -m py_compile compensate_plan.py verify_comp_plans.py analyze_model_comp.py \
  prep_test_suite.py run_test_suite.py summarize_suite.py analyze_stiction.py \
  compare_plan_runs.py summarize_model_comp.py test_offline_comp.py simulate_suite.py \
  || fail=1

echo "=== 2. unit tests ==="
python3 -m unittest -v test_offline_comp.py || fail=1

echo "=== 3. test-suite dry validation ==="
bash run_test_suite.sh --dry || fail=1

echo "=== 4. synthetic end-to-end analysis ==="
python3 simulate_suite.py || fail=1

if [ "$fail" = "0" ]; then
  echo "OFFLINE_CHECKS_OK"
  exit 0
fi
echo "OFFLINE_CHECKS_FAILED"
exit 1
