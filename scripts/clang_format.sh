#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

# Usage:
#   clang-format.sh <path/to/src/dir/or/files>

# Fail on error
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

CLANG_FORMAT_CMD="clang-format"

function check_clang_format() {
  if [[ -z "$(command -v clang-format)" ]]; then
    error "Command \"clang-format\" not found."
    error "Please make sure clang-format is installed and check your PATH" \
      "settings. For Debian/Ubuntu, you can run the following command:"
    error "  sudo apt-get -y update && sudo apt-get -y install clang-format"
    exit 1
  fi
}

function clang_format_run() {
  # Note: `${CLANG_FORMAT_CMD} -i -style=Google "$@"`
  # will override the settings in .clang-format
  ${CLANG_FORMAT_CMD} -i "$@"
}

function run_clang_format() {
  for target in "$@"; do
    if [[ -f "${target}" ]]; then
      if c_family_ext "${target}" || proto_ext "${target}"; then
        clang_format_run "${target}"
        info "Done formatting ${target}"
      else
        warning "Do nothing. ${target} is not a c/c++/cuda header/source or proto file."
      fi
    else
      local srcs
      srcs="$(find_proto_srcs ${target})"
      if [[ -n "${srcs}" ]]; then
        clang_format_run ${srcs}
      fi
      srcs="$(find_c_cpp_srcs ${target})"
      if [[ -n "${srcs}" ]]; then
        clang_format_run ${srcs}
      fi
      ok "Done formatting c/cpp/cuda/proto source files under ${target}"
    fi
  done
}

function main() {
  check_clang_format

  if [ "$#" -eq 0 ]; then
    error "Usage: $0 <path/to/dirs/or/files>"
    exit 1
  fi

  run_clang_format "$@"
}

main "$@"
