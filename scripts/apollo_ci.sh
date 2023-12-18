#! /usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"
ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

APOLLO_BUILD_SH="${APOLLO_ROOT_DIR}/scripts/apollo_build.sh"
APOLLO_TEST_SH="${APOLLO_ROOT_DIR}/scripts/apollo_test.sh"
APOLLO_LINT_SH="${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh"

function run_ci_build() {
  [[ -e "${TOP_DIR}/output" ]] && error "${TOP_DIR}/output is detected" && return -1
  mkdir -p "${TOP_DIR}/output"
  cp -rv ${TOP_DIR}/`ls -A ${TOP_DIR} | grep -vE "output|\.git*|\.cache"` ${TOP_DIR}/output/
}

function run_ci_test() {
  env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_TEST_SH}" --config=unit_test
}

function run_ci_lint() {
  env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_LINT_SH}" --cpp
}

function main() {
  local cmd="$1"
  if [ -z "${cmd}" ]; then
    cmd="ALL"
    info "Running ALL ..."
    run_ci_lint
    run_ci_build
    run_ci_test
  elif [ "${cmd}" == "test" ]; then
    info "Running CI Test ..."
    run_ci_test
  elif [ "${cmd}" == "build" ]; then
    info "Running CI Build ..."
    run_ci_build
  elif [ "${cmd}" == "lint" ]; then
    info "Running CI Lint ..."
    run_ci_lint
  fi
  success "ci ${cmd} finished."
}

main "$@"
