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

# eg.
#   perf.sh gen_data -m planning
#   perf.sh gen_svg

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"

OUTPUT_DIR="${TOP_DIR}/.cache/perf"

PERF_DATA="perf.data"
PERF_DATA_FULL_PATH="${OUTPUT_DIR}/${PERF_DATA}"
PERF_UNFOLD="perf.unfold"
PERF_FOLDED="perf.folded"
PERF_SVG="perf.svg"
PERF_CMD="perf"

FLAME_GRAPH_DIR="${TOP_DIR}/tools/FlameGraph"
FLAME_GRAPH_VERSION="1.0"

function print_usage() {
  info "Usage: $0 COMMAND [ARGS] "
  info "COMMANDS:"
  info "${TAB}help          # Show this message"
  info "${TAB}gen_data      generate perf.data by running specified command or module"
  info "${TAB}gen_svg       generate perf.svg from perf.data"
  info "${TAB}clean         cleanup .cache/perf and tools/FlameGraph"
}

function _check_perf() {
  if [ -z "$(which perf)" ]; then
    error "Seems that perf has not been installed..."
    error "You can install it manually by running:"
    error "${TAB}sudo apt-get -y update"
    error "${TAB}sudo apt-get -y install linux-tools-$(uname -r) linux-tools-common"
    exit 1
  fi
}

function _check_out_dir() {
  if [ ! -d "${OUTPUT_DIR}" ]; then
    mkdir -p "${OUTPUT_DIR}"
  fi

  if [ -f "${PERF_DATA_FULL_PATH}" ]; then
    rm -f "${PERF_DATA_FULL_PATH}"
  fi
}

# Note(All): if permission denied, run `echo 1 > /proc/sys/kernel/perf_event_paranoid` as root user out of docker
function gen_perf_data() {
  _check_perf
  _check_out_dir

  local cmd=$1
  if [ "${cmd}" == "-m" ]; then
    shift
    local module=$1
    if [ ! -z "${module}" ]; then
      ${PERF_CMD} record -e cpu-clock -o "${PERF_DATA_FULL_PATH}" -g mainboard -d \
        /apollo/modules/${module}/dag/${module}.dag \
        --flagfile=/apollo/modules/${module}/conf/${module}.conf \
        --log_dir=/apollo/data/log
    else
      error "Please specify which module to run."
      exit 1
    fi

  else
    ${PERF_CMD} record -e cpu-clock -o "${PERF_DATA_FULL_PATH}" -g "$@"
  fi
}

function _check_flame_graph() {
  if [ ! -d "${FLAME_GRAPH_DIR}" ]; then
    local answer
    typeset -l answer
    echo -n "FlameGraph does not exist. Do you want to download it (Y/n)? "
    answer=$(read_one_char_from_stdin)
    echo
    if [ "${answer}" == "y" ]; then
      local download_link="https://github.com/brendangregg/FlameGraph/archive/v${FLAME_GRAPH_VERSION}.tar.gz"
      local pkg_name="v${FLAME_GRAPH_VERSION}.tar.gz"
      wget "${download_link}"
      tar -xvf ${pkg_name}
      mv -f "FlameGraph-${FLAME_GRAPH_VERSION}" "${FLAME_GRAPH_DIR}"
      rm -f ${pkg_name}
    else
      exit 1
    fi
  fi
}

function gen_perf_svg() {
  cd "${OUTPUT_DIR}"
  if [ ! -f "${PERF_DATA}" ]; then
    error "There is no perf.data under ${OUTPUT_DIR}"
    exit 1
  else
    _check_flame_graph
    ${PERF_CMD} script -i "${PERF_DATA}" > "${PERF_UNFOLD}"
    ${FLAME_GRAPH_DIR}/stackcollapse-perf.pl ${PERF_UNFOLD} > ${PERF_FOLDED}
    ${FLAME_GRAPH_DIR}/flamegraph.pl ${PERF_FOLDED} > ${PERF_SVG}
  fi

  success "perf.svg generated under ${OUTPUT_DIR}"
}

function _clean_up() {
  for folder in "$@"; do
    if [ -d "${folder}" ]; then
      rm -rf "${folder}"
      success "Done cleanup ${folder}"
    else
      success "Nothing to do for empty directory '${folder}'."
    fi
  done
}

function clean_all() {
  _clean_up "${OUTPUT_DIR}" "${FLAME_GRAPH_DIR}"
}

function main() {
  if [ "$#" -eq 0 ]; then
    print_usage
    exit 1
  fi

  local opt="$1"
  shift
  case "${opt}" in
    help)
      print_usage
      exit 1
      ;;
    gen_data)
      gen_perf_data "$@"
      ;;
    gen_svg)
      gen_perf_svg
      ;;
    clean)
      clean_all
      ;;
    *)
      echo "Unknown command: ${opt}"
      print_usage
      exit 1
      ;;
  esac
}

main "$@"
