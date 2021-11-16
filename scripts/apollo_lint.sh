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
# shellcheck source=./apollo.bashrc
source "${TOP_DIR}/scripts/apollo.bashrc"

: ${STAGE:=dev}

PYTHON_LINT_FLAG=0
CPP_LINT_FLAG=0
SHELL_LINT_FLAG=0

function _cpp_lint_impl() {
  bazel test --config=cpplint "$@"
}

function run_cpp_lint() {
  pushd "${APOLLO_ROOT_DIR}" >/dev/null
  local cpp_dirs="cyber"
  if [[ "${STAGE}" == "dev" ]]; then
    cpp_dirs="${cpp_dirs} modules"
  fi
  for prey in $(find ${cpp_dirs} -name BUILD \
    | xargs grep -l -E 'cc_library|cc_test|cc_binary|cuda_library' \
    | xargs grep -L 'cpplint()'); do
    warning "unattended BUILD file found: ${prey}. Add cpplint() automatically."
    sed -i '1i\load("//tools:cpplint.bzl", "cpplint")\n' "${prey}"
    sed -i -e '$a\\ncpplint()' "${prey}"
    local buidifier
    buidifier="$(command -v buildifier)"
    if [ ! -z "${buidifier}" ]; then
      ${buidifier} -lint=fix "${prey}"
    fi
  done
  popd >/dev/null

  local targets="//cyber/..."
  _cpp_lint_impl "${targets}"

  if [[ "${STAGE}" == "dev" ]]; then
    _cpp_lint_impl "//modules/..."
  fi

}

function run_sh_lint() {
  local shellcheck_cmd
  shellcheck_cmd="$(command -v shellcheck)"
  if [ -z "${shellcheck_cmd}" ]; then
    warning "Command 'shellcheck' not found. For Debian/Ubuntu systems," \
      "please run the following command to install it: "
    warning "  sudo apt-get -y update"
    warning "  sudo apt-get -y install shellcheck"
    exit 1
  fi
  local sh_dirs="cyber scripts docker tools"
  if [[ "${STAGE}" == "dev" ]]; then
    sh_dirs="modules ${sh_dirs}"
  fi

  sh_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " ${sh_dirs})
  run find ${sh_dirs} -type f \( -name "*.sh" -or -name "*.bashrc" \) -exec \
    shellcheck -x --shell=bash {} +

  for script in ${APOLLO_ROOT_DIR}/*.sh; do
    run shellcheck -x --shell=bash "${script}"
  done
}

function run_py_lint() {
  local flake8_cmd
  flake8_cmd="$(command -v flake8)"
  if [ -z "${flake8_cmd}" ]; then
    warning "Command flake8 not found. You can install it manually via:"
    warning "  '[sudo -H] python3 -m pip install flake8'"
    exit 1
  fi

  local py_dirs="cyber docker tools"
  if [[ "${STAGE}" == "dev" ]]; then
    py_dirs="modules ${py_dirs}"
  fi

  py_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " ${py_dirs})
  run find ${py_dirs} -type f \( -name "*.py" \) -exec \
    flake8 {} \;
}

function print_usage() {
  info "Usage: $0 [Options]"
  info "Options:"
  info "${TAB}--py        Lint Python files"
  info "${TAB}--sh        Lint Bash scripts"
  info "${TAB}--cpp       Lint cpp source files"
  info "${TAB}-a|--all    Lint all. Equivalent to \"--py --sh --cpp\""
  info "${TAB}-h|--help   Show this message and exit"
}

function parse_cmdline_args() {
  if [[ "$#" -eq 0 ]]; then
    CPP_LINT_FLAG=1
    return 0
  fi

  while [[ "$#" -gt 0 ]]; do
    local opt="$1"
    shift
    case "${opt}" in
      --py)
        PYTHON_LINT_FLAG=1
        ;;
      --cpp)
        CPP_LINT_FLAG=1
        ;;
      --sh)
        SHELL_LINT_FLAG=1
        ;;
      -a | --all)
        PYTHON_LINT_FLAG=1
        CPP_LINT_FLAG=1
        SHELL_LINT_FLAG=1
        ;;
      -h | --help)
        print_usage
        exit 0
        ;;
      *)
        warning "Unknown option: ${opt}"
        print_usage
        exit 1
        ;;
    esac
  done
}

function main() {
  parse_cmdline_args "$@"
  if [[ "${CPP_LINT_FLAG}" -eq 1 ]]; then
    run_cpp_lint
  fi
  if [[ "${PYTHON_LINT_FLAG}" -eq 1 ]]; then
    run_py_lint
  fi

  if [[ "${SHELL_LINT_FLAG}" -eq 1 ]]; then
    run_sh_lint
  fi
}

main "$@"
