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
#   apollo_format.sh [options] <path/to/src/dir/or/files>

# Fail on error
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

function print_usage() {
  info "Usage: $0 [options] <path/to/src/dir/or/files> "
  info "Options:"
  info "${TAB}-p|--python      Format Python code"
  info "${TAB}-b|--bazel       Format Bazel code"
  info "${TAB}-c|--cpp         Format cpp code"
  info "${TAB}-s|--shell       Format Shell code"
  info "${TAB}-a|--all         Format all (C++/Python/Bazel/Shell)"
  info "${TAB}-h|--help        # Show this message"
}

function run_clang_format() {
  bash "${TOP_DIR}/scripts/clang_format.sh" "$@"
}

function run_buildifier() {
  bash "${TOP_DIR}/scripts/buildifier.sh" "$@"
}

function run_autopep8() {
  bash "${TOP_DIR}/scripts/autopep8.sh" "$@"
}

function run_shfmt() {
  bash "${TOP_DIR}/scripts/shfmt.sh" "$@"
}

function run_format_all() {
  run_clang_format "$@"
  run_buildifier "$@"
  run_autopep8 "$@"
  run_shfmt "$@"
}

function main() {
  if [ "$#" -eq 0 ]; then
    print_usage
    exit 1
  fi

  local option="$1"
  shift
  case "${option}" in
  -p|--python)
    run_autopep8 "$@"
    ;;
  -c|--cpp)
    run_clang_format "$@"
    ;;
  -b|--bazel)
    run_buildifier "$@"
    ;;
  -s|--shell)
    run_shfmt "$@"
    ;;
  -a|--all)
    run_format_all "$@"
    ;;
  -h|--help)
    print_usage
    exit 1
    ;;
  *)
    echo "Unknown option: ${option}"
    print_usage
    exit 1
    ;;
  esac
}

main "$@"
