#! /bin/bash

# regenerate planning golden test log data.

set -x

D=/apollo/bazel-bin/modules/planning/

for T in integration_tests/garage_test \
  integration_tests/sunnyvale_loop_test \
  integration_tests/sunnyvale_big_loop_test; do
  $D/$T --test_update_golden_log
done
