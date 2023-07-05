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

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

function buildify() {
  local stage="${STAGE}"

  local buildifier_cmd="$(command -v buildifier)"
  if [ -z "${buildifier_cmd}" ]; then
    local download_link="https://github.com/bazelbuild/buildtools/releases"
    error "Command buildifier not found. You can download and install" \
      "(or build) it from:"
    error "${TAB}${download_link}"
    exit 1
  fi

  buildifier_cmd="${buildifier_cmd} -lint=fix"

  local build_dirs="cyber third_party tools"
  if [ "${stage}" == "dev" ]; then
    build_dirs="modules ${build_dirs}"
  fi
  build_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " $build_dirs)

  set -x
  find ${build_dirs} -type f \
    \( -name "BUILD" -or -name "*.BUILD" -or -name "*.bzl" -or -name "*.bazel" \) \
    -exec ${buildifier_cmd} {} +
  set +x

  success "buildifier run finished successfully."
  if [ -f "${APOLLO_ROOT_DIR}/BUILD" ]; then
    ${buildifier_cmd} "${APOLLO_ROOT_DIR}/BUILD"
  fi
  if [ -f "${APOLLO_ROOT_DIR}/WORKSPACE.in" ]; then
    ${buildifier_cmd} "${APOLLO_ROOT_DIR}/WORKSPACE.in"
  fi
}

function main() {
  buildify "$@"
}

main "$@"
