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
source "${TOP_DIR}/scripts/apollo_base.sh"

function run_bazel_build() {
  if ${USE_ESD_CAN}; then
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define USE_ESD_CAN=${USE_ESD_CAN}"
  fi

  CMDLINE_OPTIONS="$(echo ${CMDLINE_OPTIONS} | xargs)"

  local build_targets
  build_targets="$(determine_targets ${SHORTHAND_TARGETS})"

  local disabled_targets
  disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"
  disabled_targets="$(echo ${disabled_targets} | xargs)"

  # Note(storypku): Workaround for in case "/usr/bin/bazel: Argument list too long"
  # bazel build ${CMDLINE_OPTIONS} ${job_args} $(bazel query ${build_targets})
  local formatted_targets="$(format_bazel_targets ${build_targets} ${disabled_targets})"

  info "Build Overview: "
  info "${TAB}USE_GPU:       ${GREEN}${USE_GPU}${NO_COLOR}  [ 0 for CPU, 1 for GPU ]"
  if [ "${USE_GPU}" -eq 1 ]; then
    info "${TAB}GPU arch:      ${GREEN}${GPU_PLATFORM}${NO_COLOR}"
  else
    info "${TAB}CPU arch:      ${GREEN}${ARCH}${NO_COLOR}"
  fi
  info "${TAB}Bazel Options: ${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
  info "${TAB}Build Targets: ${GREEN}${build_targets}${NO_COLOR}"
  info "${TAB}Disabled:      ${YELLOW}${disabled_targets}${NO_COLOR}"

  local job_args="--jobs=$(nproc) --local_ram_resources=HOST_RAM*0.7"
  bazel build ${CMDLINE_OPTIONS} ${job_args} -- ${formatted_targets}
}

function main() {
  parse_cmdline_arguments "$@"
  if [ "${APOLLO_OUTSIDE_DOCKER}" -eq 1 ]; then
    warning "Assembling outside the docker can cause errors,"
    warning "  we recommend using a ready-made container."
    warning "Make sure that all dependencies are installed,"
    warning "  if errors, try running <apollo_path>/docker/build/installers/install.sh"
  elif ! "${APOLLO_IN_DOCKER}"; then
    error "The build operation must be run from within docker container"
    error "Use -o flag to force build"
    exit 1
  fi
  determine_cpu_or_gpu "build"

  run_bazel_build

  if [ -z "${SHORTHAND_TARGETS}" ]; then
    SHORTHAND_TARGETS="apollo"
  fi

  success "Done building ${SHORTHAND_TARGETS}. Enjoy!"
}

main "$@"
