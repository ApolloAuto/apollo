#!/usr/bin/env bash

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

BRIDGE_PORT=9090
HOST_BRIDGE_PORT="${HOST_BRIDGE_PORT:-$BRIDGE_PORT}"
info "Bridge port: ${HOST_BRIDGE_PORT}"

${TOP_DIR}/bazel-bin/modules/contrib/cyber_bridge/cyber_bridge
