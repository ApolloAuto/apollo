#!/usr/bin/env bash

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

# Usage:
#   autopep8.sh <path/to/src/dir/or/files>

# Fail on error
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

AUTOPEP8_CMD="autopep8"

function check_autopep8() {
  if [ -z "$(command -v autopep8)" ]; then
    error "Oops, autopep8 missing..."
    error "Please make sure autopep8 is installed and check your PATH" \
      "settings. For Debian/Ubuntu, you can run the following command:"
    error "  sudo pip install --upgrade --no-cache-dir autopep8"
    exit 1
  fi
}

function autopep8_run() {
  ${AUTOPEP8_CMD} -i "$@"
}

function run_autopep8() {
  for target in "$@"; do
    if [ -f "${target}" ]; then
      if py_ext "${target}"; then
        autopep8_run "${target}"
        info "Done formatting ${target}"
      else
        warning "Do nothing. ${target} is not a Python file."
      fi
    else
      local srcs
      srcs="$(find_py_srcs ${target})"
      if [ -z "${srcs}" ]; then
        warning "Do nothing. No Python files found under ${target} ."
        continue
      fi
      autopep8_run ${srcs}
      ok "Done formatting Python files under ${target}"
    fi
  done
}

function main() {
  check_autopep8

  if [ "$#" -eq 0 ]; then
    error "Usage: $0 <path/to/dirs/or/files>"
    exit 1
  fi

  run_autopep8 "$@"
}

main "$@"
