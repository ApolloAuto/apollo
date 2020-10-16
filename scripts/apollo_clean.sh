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

BAZEL_CLEAN_EXPUNGE=0
CORE_DUMP_CLEANUP=0
LOG_FILES_CLEANUP=0

function _clean_bazel_cache() {
  local opt="--async"
  if [ "$1" == "--expunge" ]; then
    opt="--expunge_async"
  fi

  bazel clean ${opt}

  # Remove bazel cache in associated directories
  if [ -d /apollo-simulator ]; then
    pushd /apollo-simulator >/dev/null
    bazel clean ${opt}
    popd >/dev/null
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
  echo "${TAB}--bazel     Remove bazel output"
  echo "${TAB}--core      Remove coredump files"
  echo "${TAB}--log       Remove log files"
  echo "${TAB}-a, --all   Equivalent to \"--bazel --core --log\""
  echo "${TAB}--expunge   Run \"bazel clean --expunge\""
  echo "${TAB}-h, --help  Show this message and exit"
}

function parse_arguments() {
  if [[ $# -eq 0 ]]; then
    _print_usage
    exit 1
  fi

  while [ $# -gt 0 ]; do
    local opt="$1"
    shift
    case "${opt}" in
      --bazel)
        [ "${BAZEL_CLEAN_EXPUNGE}" -eq 0 ] && BAZEL_CLEAN_EXPUNGE=1
        ;;
      --core)
        CORE_DUMP_CLEANUP=1
        ;;
      --log)
        LOG_FILES_CLEANUP=1
        ;;
      --expunge)
        BAZEL_CLEAN_EXPUNGE=2
        ;;
      -a | --all)
        CORE_DUMP_CLEANUP=1
        LOG_FILES_CLEANUP=1
        [ "${BAZEL_CLEAN_EXPUNGE}" -eq 0 ] && BAZEL_CLEAN_EXPUNGE=1
        ;;
      -h | --help)
        _print_usage
        exit 1
        ;;
      *)
        _print_usage
        exit 1
        ;;
    esac
  done
}

function main() {
  parse_arguments "$@"

  if ! "${APOLLO_IN_DOCKER}"; then
    error "The clean operation should be run from within docker container"
    exit 1
  fi

  if [ "${BAZEL_CLEAN_EXPUNGE}" -eq 2 ]; then
    info "Clean bazel cache, opcode=${BAZEL_CLEAN_EXPUNGE}"
    _clean_bazel_cache "--expunge"
  elif [ "${BAZEL_CLEAN_EXPUNGE}" -eq 1 ]; then
    info "Clean bazel cache, opcode=${BAZEL_CLEAN_EXPUNGE}"
    _clean_bazel_cache
  fi

  _clean_config
  _clean_docs

  if [ "${CORE_DUMP_CLEANUP}" -eq 1 ]; then
    info "Cleanup core dump files under data/core/ ..."
    _clean_core
  fi

  if [ "${LOG_FILES_CLEANUP}" -eq 1 ]; then
    info "Cleanup log files under data/log/ ..."
    _clean_log
  fi

  success "Apollo cleanup done."
}

main "$@"
