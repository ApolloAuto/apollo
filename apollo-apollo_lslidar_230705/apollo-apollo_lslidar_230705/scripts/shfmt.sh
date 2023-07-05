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
#   shfmt.sh <path/to/src/dir/or/files>

# Fail on error
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

SHELL_FORMAT_CMD="shfmt"

function _find_shell_srcs() {
  find "$@" -type f -name "*.sh" \
    -o -name "*.bash" \
    -o -name "*.bashrc"
}

function _shell_ext() {
  local __ext
  __ext="$(file_ext $1)"
  for ext in "sh" "bash" "bashrc"; do
    if [ "${ext}" == "${__ext}" ]; then
      return 0
    fi
  done
  return 1
}

function check_shfmt() {
  SHELL_FORMAT_CMD="$(command -v shfmt)"
  if [ -z "${SHELL_FORMAT_CMD}" ]; then
    error "Oops, shfmt missing..."
    error "Please make sure shfmt is installed and check your PATH settings."
    exit 1
  fi
}

function shell_format_run() {
  # Use settings in .editorconfig
  ${SHELL_FORMAT_CMD} -w "$@"
}

function run_shfmt() {
  for target in "$@"; do
    if [ -f "${target}" ]; then
      if _shell_ext "${target}"; then
        shell_format_run "${target}"
        info "Done formatting ${target}"
      else
        warning "Do nothing. ${target} is not a bash scripts."
      fi
    else
      local srcs
      srcs="$(_find_shell_srcs ${target})"
      if [ -z "${srcs}" ]; then
        ok "No need to format shell scripts under ${target} as none found"
        continue
      fi
      shell_format_run ${srcs}
      ok "Done formatting shell scripts under ${target}"
    fi
  done
}

function main() {
  check_shfmt

  if [ "$#" -eq 0 ]; then
    error "Usage: $0 <path/to/dirs/or/files>"
    exit 1
  fi

  run_shfmt "$@"
}

main "$@"
