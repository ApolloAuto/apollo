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
#   clang-format.sh <path/to/src/dir/or/file>

# Fail on error
set -euo pipefail

TOP_DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
. "${TOP_DIR}/scripts/apollo.bashrc"

TARGET=$1
if [[ -z "${TARGET}" ]]; then
  echo "No path specified. Exiting..."
  exit 1
fi

# Check tool.
CLANG_FORMAT_CMD="$(command -v clang-format)"

if [[ -z "${CLANG_FORMAT_CMD}" ]]; then
  echo "Installing clang-format..."
  sudo apt-get -y update && \
    sudo apt-get -y install clang-format
  CLANG_FORMAT_CMD="$(command -v clang-format)"
fi

function clang_format_run() {
  ${CLANG_FORMAT_CMD} -i -style=Google "$@"
}

# Format.
if [ -f "${TARGET}" ]; then
  if c_family_ext "${TARGET}"; then
    clang_format_run "${TARGET}"
    info "Done formatting ${TARGET}"
  else
    warning "Do nothing. ${TARGET} " \
            "is not c/c++/cuda header/source file."
  fi
else
  srcs="$(find_c_cpp_srcs ${TARGET})"
  if [[ -z "${srcs}" ]]; then
      warning "Do nothing. No c/c++/cuda header/source" \
              "files found under ${TARGET} ."
      exit 0
  fi
  clang_format_run ${srcs}
  info "Done formatting c/cpp/cuda source files under ${TARGET}"
fi
