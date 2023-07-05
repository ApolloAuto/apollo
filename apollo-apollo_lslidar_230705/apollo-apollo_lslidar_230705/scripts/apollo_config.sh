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
source "${TOP_DIR}/cyber/setup.bash"

BAZEL_CONF="${TOP_DIR}/.apollo.bazelrc"

function run_bootstrap() {
  py3_bin="$(which python3 || true)"
  # Set all env variables
  "${py3_bin}" "${TOP_DIR}/tools/bootstrap.py" "$@"
}

function print_usage() {
  info "Usage: $0 [Options]"
  info "Options:"
  info "${TAB}-i|--interactive      Run in interactive mode"
  info "${TAB}-n|--noninteractive   Run in non-interactive mode"
  info "${TAB}-h|--help             Show this message and exit"
}

function main() {
  local mycfg="$(basename ${BAZEL_CONF})"
  if [[ "$#" -eq 0 ]]; then
    print_usage
    exit 1
  fi

  case "$1" in
    --clean)
      rm -f "${BAZEL_CONF}"
      exit 0
      ;;
    -h | --help)
      print_usage
      exit 0
      ;;
    -i | --interactive)
      info "Configure ${GREEN}${mycfg}${NO_COLOR} in interactive mode"
      run_bootstrap --interactive
      ok "Successfully configured ${GREEN}${mycfg}${NO_COLOR} in interactive mode."
      exit 0
      ;;
    -n | --noninteractive)
      info "Configure ${GREEN}${mycfg}${NO_COLOR} in non-interactive mode"
      run_bootstrap --interactive false
      ok "Successfully configured ${GREEN}${mycfg}${NO_COLOR} in non-interactive mode."
      exit 0
      ;;
    *)
      print_usage
      exit 1
      ;;
  esac
}

main "$@"
