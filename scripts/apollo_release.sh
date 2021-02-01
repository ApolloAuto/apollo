#! /usr/bin/env bash

STAGE_DIR="/tmp/apollo"

# TODO(storypku): revisit cpu/gpu release later.
bazel run //:install --config=gpu \
  -- \
  --pre_clean \
  ${STAGE_DIR}

find "${STAGE_DIR}" -name "*.dag" -exec \
  sed -i 's@/apollo/bazel-bin@/apollo@g' {} +
