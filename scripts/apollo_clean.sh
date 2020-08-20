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

CORE_DIR="${TOP_DIR}/data/core"
LOG_DIR="${TOP_DIR}/data/log"

function _clean_cache() {
  local opt="--async"
  if [ "$1" == "--expunge" ]; then
    opt="--expunge_async"
  fi

  bazel clean ${opt}

  # Remove bazel cache in associated directories
  if [ -d /apollo-simulator ]; then
    pushd /apollo-simulator > /dev/null
    bazel clean ${opt}
    popd > /dev/null
  fi
}

function _clean_config() {
  # Remove local bazel config.
  bash "${TOP_DIR}/scripts/apollo_config.sh" --clean
}

function _clean_core() {
  if [ -d "${CORE_DIR}" ]; then
    rm -f ${CORE_DIR}/core_*
  fi
}

function _clean_log() {
  if [ -d "${LOG_DIR}" ]; then
    rm -rf ${LOG_DIR}/*
  fi
}

function _clean_docs() {
  local docs_sh="${TOP_DIR}/scripts/apollo_docs.sh"
  if [ -f "${docs_sh}" ]; then
    bash "${docs_sh}" clean
  fi
}

function _print_usage() {
  echo -e "Usage:\n  $0 [options]\nOptions:"
  echo "${TAB}--expunge   Run \"bazel clean --expunge\""
  echo "${TAB}-h, --help  Show this message"
}

function main() {
  if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    _print_usage
    exit 1
  fi

  if ! "${APOLLO_IN_DOCKER}"; then
    error "The clean operation should be run from within docker container"
    exit 1
  fi

  _clean_cache "$@"
  _clean_config
  _clean_docs

  local answer
  typeset -l answer
  warning "Do you want to remove all log files under \"${LOG_DIR}\"" \
    "and core dump files under \"${CORE_DIR}\" (Y/n)?"
  answer=$(read_one_char_from_stdin)
  info "Your choice is: ${answer}"
  if [ "${answer}" == "y" ]; then
    _clean_core
    _clean_log
  fi

  success "Apollo cleanup done."
}

main "$@"
