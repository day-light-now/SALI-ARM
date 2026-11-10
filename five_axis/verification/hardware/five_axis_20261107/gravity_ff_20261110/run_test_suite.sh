#!/bin/bash
# Wrapper: source the gravity_ff environment, then delegate to the Python runner.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
exec python3 "$HERE/run_test_suite.py" "$@"
