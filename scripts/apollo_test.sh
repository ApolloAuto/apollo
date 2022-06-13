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

function run_bazel_test() {
  if ${USE_ESD_CAN}; then
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define USE_ESD_CAN=${USE_ESD_CAN}"
  fi

  CMDLINE_OPTIONS="$(echo ${CMDLINE_OPTIONS} | xargs)"

  local test_targets
  test_targets="$(determine_targets ${SHORTHAND_TARGETS})"

  local disabled_targets
  disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"
  disabled_targets="$(echo ${disabled_targets} | xargs)"

  # Note(storypku): Workaround for "/usr/bin/bazel: Argument list too long"
  # bazel test ${CMDLINE_OPTIONS} ${job_args} $(bazel query ${test_targets} ${disabled_targets})
  local formatted_targets="$(format_bazel_targets ${test_targets} ${disabled_targets})"

  info "Test Overview: "
  info "${TAB}USE_GPU: ${USE_GPU}  [ 0 for CPU, 1 for GPU ]"
  info "${TAB}Test Options: ${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
  info "${TAB}Test Targets: ${GREEN}${test_targets}${NO_COLOR}"
  info "${TAB}Disabled:     ${YELLOW}${disabled_targets}${NO_COLOR}"

  local job_args="--jobs=$(nproc) --local_ram_resources=HOST_RAM*0.7"
  bazel test ${CMDLINE_OPTIONS} ${job_args} -- ${formatted_targets}
}

function main() {
  if ! "${APOLLO_IN_DOCKER}"; then
    error "This test operation must be run from within docker container"
    exit 1
  fi

  parse_cmdline_arguments "$@"
  determine_cpu_or_gpu "tests"

  run_bazel_test
  success "Done testing ${SHORTHAND_TARGETS:-Apollo}. Enjoy"
}

main "$@"
