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
  run_bazel "Build"
  if [ -z "${SHORTHAND_TARGETS}" ]; then
    SHORTHAND_TARGETS="apollo"
  fi
  success "Done building ${SHORTHAND_TARGETS}. Enjoy!"
}

main "$@"
