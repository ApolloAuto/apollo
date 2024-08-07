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

BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[1;34;48m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
PROCESS_NAME="mainboard"

INPUT_FILES=

function usage() {
  echo -e "\n${RED}Usage${NO_COLOR}:
    .${BOLD}/sampling.sh${NO_COLOR} [OPTION] [ORIGIN SAMPLING FILES]"
  echo -e "\n${RED}Options${NO_COLOR}:
    ${BLUE}-p  [options]${NO_COLOR}: specify sampling process name, default is ${PROCESS_NAME}
    "
}

function parse_args() {
  if [[ "$#" -eq 0 ]]; then
    usage
    exit 1
  fi

  while [[ "$#" -gt 0 ]]; do
    local opt="$1"
    shift
    case "${opt}" in
      -p)
        PROCESS_NAME=$1
        shift
        ;;
      -h | --help)
        usage
        exit 0
        ;;
      *)
        INPUT_FILES="${opt} ${INPUT_FILES}"  
        ;;
    esac
  done
}

parse_args "$@"

[[ -z $INPUT_FILES ]] && error "sampling files not specified. Exiting ..." && exit 1

CBT_FILE="profile.cbt"
SVG_FILE="profile.svg"

[[ ! `which pprof` ]] && error "please build or install cyber first. Exiting ..." && exit 1

[[ ! `which flamegraph.pl` ]] && \
    git clone --progress https://github.com/brendangregg/FlameGraph.git ~/FlameGraph && \
    sudo ln -snf ~/FlameGraph/flamegraph.pl /usr/bin/flamegraph.pl

# [[ -z $(which mainboard) ]] && error "please install or compile apollo first. Exiting ..." && exit 1
pprof --collapsed $(which ${PROCESS_NAME}) $INPUT_FILES > ${CBT_FILE}

flamegraph.pl ${CBT_FILE} > ${SVG_FILE}

rm -f ${CBT_FILE}
