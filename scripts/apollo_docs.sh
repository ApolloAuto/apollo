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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

set -e

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

APOLLO_DOCS_CFG="${APOLLO_ROOT_DIR}/apollo.doxygen"
APOLLO_DOCS_DIR="${APOLLO_ROOT_DIR}/.cache/docs"
# APOLLO_DOCS_PORT=9527 # Unused for now

function determine_docs_dir() {
  local doxygen_cfg="${APOLLO_DOCS_CFG}"
  local output_dir="$(awk -F '[= ]' \
    '/^OUTPUT_DIRECTORY/ {print $NF}' ${doxygen_cfg})"

  if [ -z "${output_dir}" ]; then
    error "Oops, OUTPUT_DIRECTORY not set in ${doxygen_cfg}"
    exit 1
  fi

  if [[ "${output_dir}" != /* ]]; then
    output_dir="${APOLLO_ROOT_DIR}/${output_dir}"
  fi

  APOLLO_DOCS_DIR="${output_dir}"
}

function generate_docs() {
  local doxygen_cfg="${APOLLO_DOCS_CFG}"
  local output_dir="${APOLLO_DOCS_DIR}"

  local gendoc=true
  if [ -d "${output_dir}" ]; then
    local answer
    echo -n "Docs directory ${output_dir} already exists. Do you want to keep it (Y/n)? "
    answer=$(read_one_char_from_stdin)
    echo
    if [ "${answer}" == "n" ]; then
      rm -rf "${output_dir}"
    else
      gendoc=false
    fi
  fi
  if ! $gendoc; then
    return
  fi
  info "Generating Apollo docs..."
  local doxygen_cmd="$(command -v doxygen)"
  if [ -z "${doxygen_cmd}" ]; then
    error "Command 'doxygen' not found. Please install it manually."
    error "On Ubuntu 18.04, this can be done by running: "
    error "${TAB}sudo apt-get -y update"
    error "${TAB}sudo apt-get -y install doxygen"
    exit 1
  fi

  if [ ! -d "${output_dir}" ]; then
    mkdir -p "${output_dir}"
  fi

  local start_time="$(get_now)"
  pushd "${APOLLO_ROOT_DIR}" > /dev/null
  run "${doxygen_cmd}" "${doxygen_cfg}" > /dev/null
  popd > /dev/null

  local elapsed="$(time_elapsed_s ${start_time})"
  success "Apollo docs generated. Time taken: ${elapsed}s"
}

function clean_docs() {
  if [ -d "${APOLLO_DOCS_DIR}" ]; then
    rm -rf "${APOLLO_DOCS_DIR}"
    success "Done cleanup apollo docs in ${APOLLO_DOCS_DIR}"
  else
    success "Nothing to do for empty directory '${APOLLO_DOCS_DIR}'."
  fi
}

function _usage() {
  info "Usage:"
  info "${TAB}$0 [Options]"
  info "Options:"
  info "${TAB}-h|--help   Show this help message and exit"
  info "${TAB}clean       Delete generated docs"
  info "${TAB}generate    Generate apollo docs"
  #local doclink="http://0.0.0.0:${APOLLO_DOCS_PORT}"
  #info "${TAB}start       Start local apollo docs server at ${doclink}"
  #info "${TAB}shutdown    Shutdown local apollo docs server at ${doclink}"
  exit 1
}

# TODO(all): cyber/doxy-docs

function main() {
  local cmd="$1"
  determine_docs_dir

  case "${cmd}" in
    generate)
      generate_docs
      ;;
    clean)
      clean_docs
      ;;
      #  start)
      #      start_doc_server
      #      ;;
      #  shutdown)
      #      shutdown_doc_server
      #      ;;
    -h | --help)
      _usage
      ;;

    *)
      _usage
      ;;
  esac
}

main "${@}"
