#!/usr/bin/env bash

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

${TOP_DIR}/bazel-bin/modules/contrib/cyber_bridge/cyber_bridge
