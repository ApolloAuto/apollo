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
#   apollo_format.sh <path/to/src/dir/or/files> [Options]

# Fail on error
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

function print_usage() {
    info "Usage: $0 <path/to/src/dir/or/files> [Options]"
    info "Options:"
    info "${TAB}py|python   Format Python code"
    info "${TAB}bazel       Lint Bazel code"
    info "${TAB}cpp         Format cpp code"
    info "${TAB}all         Lint all (C++/Python/Bazel)"
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

function run_format_all() {
    run_clang_format "$@"
    run_buildifier "$@"
    run_autopep8 "$@"
}

function main() {
    if [ "$#" -eq 0 ]; then
        print_usage
        exit 1
    fi

    local path="$1"
    shift
    local option="$1"
    case "${option}" in
    py | python)
        run_autopep8 "${path}"
        ;;
    cpp)
        run_clang_format "${path}"
        ;;
    bazel)
        run_buildifier "${path}"
        ;;
    all)
        run_format_all "${path}"
        ;;
    *)
        run_format_all "${path}"
        ;;
    esac
}

main "$@"
