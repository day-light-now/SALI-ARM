#!/usr/bin/env bash
# Run the panel verification suites in the isolated domain (176).
#   1) production-callback counterexamples (real backend + HTTP, no ROS spin)
#   2) end-to-end with a fresh PTY fake stack per scenario
set -o pipefail
cd "$(dirname "$0")"
source ./test_env.sh
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1

REPO=$(cd "$(dirname "$0")/../../.." && pwd)
echo "=== C++ gravity feedforward model test"
g++ -std=c++17 -Wall -Wextra -Wpedantic -I "$REPO/src/slaihand_hardware/include" \
  "$REPO/src/slaihand_hardware/test/gravity_ff_test.cpp" \
  "$REPO/src/slaihand_hardware/src/gravity_ff.cpp" -o /tmp/gravity_ff_test
/tmp/gravity_ff_test
model_status=$?

echo "=== gravity model cross-check (C++ versus Python)"
python3 verify_gravity_model.py
cross_status=$?

echo "=== J3 fixed-offset chain proof"
python3 j3_offset_proof.py > /tmp/j3_proof.log
j3_status=$?

echo "=== calibration schema / exclusion / insufficiency counterexamples"
python3 test_calibration_core.py > /tmp/calibration_core.log 2>&1
calibration_status=$?

echo "=== driver runtime identity (latched + periodic refresh)"
python3 verify_runtime_identity.py > /tmp/runtime_identity.log 2>&1
identity_status=$?

echo "=== calibration import/preview CLI (fake data, read-only)"
python3 fake_calibration_check.py > /tmp/calibration_cli.log 2>&1
cli_status=$?

echo "=== production callback counterexamples"
python3 test_control_panel_callbacks.py 2>&1 | tee /tmp/panel_callbacks.log
callback_status="${PIPESTATUS[0]}"
cp /tmp/panel_callbacks.log panel_callbacks.log 2>/dev/null || echo "WARNING: could not copy panel_callbacks.log"

echo "=== end-to-end (per-scenario fake stacks)"
python3 verify_control_panel.py 2>&1 | tee /tmp/panel_verify.log
verify_status="${PIPESTATUS[0]}"
cp /tmp/panel_verify.log panel_verify.log 2>/dev/null || echo "WARNING: could not copy panel_verify.log"

echo "=== driver gravity feedforward PTY scenario"
python3 verify_gravity_ff_pty.py 2>&1 | tee /tmp/panel_ff.log
ff_status="${PIPESTATUS[0]}"
cp /tmp/panel_ff.log panel_ff.log 2>/dev/null || echo "WARNING: could not copy panel_ff.log"

if [ "$callback_status" -ne 0 ] || [ "$verify_status" -ne 0 ] || [ "$model_status" -ne 0 ] \
   || [ "$cross_status" -ne 0 ] || [ "$ff_status" -ne 0 ] || [ "$j3_status" -ne 0 ] \
   || [ "$calibration_status" -ne 0 ] || [ "$cli_status" -ne 0 ] || [ "$identity_status" -ne 0 ]; then
  echo "RESULT: FAIL (callbacks=$callback_status e2e=$verify_status model=$model_status cross=$cross_status ff=$ff_status j3=$j3_status calib=$calibration_status cli=$cli_status identity=$identity_status)"
  exit 1
fi
echo "RESULT: PASS (callbacks=0 e2e=0 model=0 cross=0 ff=0 j3=0 calib=0 cli=0 identity=0)"
