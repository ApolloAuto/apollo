#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

set -ue

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'
  echo -e "\n${RED}Usage${NONE}:
  ${BOLD}bash client.sh${NONE} --stage STAGE [--cmd COMMAND, default is start] [--record_path PATH, only record_check requied]"

  echo -e "\n${RED}Stages${NONE}:
  ${BLUE}record_check${NONE}: check data integrity
  ${BLUE}static_align${NONE}: static alignment
  ${BLUE}eight_route${NONE}: figure eight
  ${BLUE}data_collect${NONE}: data collection
  ${BLUE}loops_check${NONE}: check loops
  ${BLUE}clean${NONE}: end this collection
  "
  echo -e "${RED}Commands${NONE}:
  ${BLUE}start${NONE}: start corresponding stage
  ${BLUE}stop${NONE}: stop corresponding stage
  "
}

function set_global_var() {
  SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  APOLLO_ROOT_PATH="${SCRIPT_PATH}/../../.."
  MAP_DATACHECKER_CLIENT=${APOLLO_ROOT_PATH}/bazel-bin/modules/map/tools/map_datachecker/client/map_datachecker_client
  CONF=${APOLLO_ROOT_PATH}/modules/map/tools/map_datachecker/conf/map-datachecker.conf
  LOG_DIR=${APOLLO_ROOT_PATH}/modules/tools/map_datachecker/log
}

function examine_2_params() {
    if [[ $1 != "--stage" ]];then
      return -1
    fi
    stage=$2
    if [[ ${stage} != "record_check" ]] &&
       [[ ${stage} != "static_align" ]] &&
       [[ ${stage} != "eight_route" ]] &&
       [[ ${stage} != "data_collect" ]] &&
       [[ ${stage} != "loops_check" ]] &&
       [[ ${stage} != "clean" ]];then
      return -1
    fi
    return 0
}

function examine_4_params() {
  examine_2_params $@
  if [[ $? -ne 0 ]];then
    return -1
  fi
  if [[ $3 == "--cmd" ]];then
    cmd=$4
    if [[ ${cmd} != "start" ]] &&
       [[ ${cmd} != "stop" ]];then
      return -1
    fi
  elif [[ $3 == "--record_path" ]];then
    record_path=$4
  else
    return -1
  fi
  return 0
}

function examine_6_params() {
  examine_4_params $@
  if [[ $? -ne 0 ]];then
    return -1
  fi
  if [[ $5 == "--record_path" ]];then
    record_path=$4
  else
    return -1
  fi
  return 0
}

function examine_params() {
  if [[ $# -eq 2 ]];then
    examine_2_params $@
  elif [[ $# -eq 4 ]];then
    examine_4_params $@
  elif [[ $# -eq 6 ]];then
    examine_6_params $@
  else
    print_usage
    return -1
  fi
  if [[ $? -ne 0 ]];then
    print_usage
    return -1
  fi
  stage=$2
  if [[ ${stage} == "record_check" ]];then
    if [[ $# -eq 2 ]];then
      print_usage
      return -1
    elif [[ $# -eq 4 ]];then
      if [[ $3 != "--record_path" ]];then
        print_usage
        return -1
      fi
    else
      if [[ $3 != "--record_path" ]] &&
         [[ $5 != "--record_path" ]];then
        print_usage
        return -1
      fi
    fi
  fi
  return 0
}

function trap_ctrlc() {
  PID=$!
  echo kill ${PID}
  kill -INT ${PID}
}

function main() {
  examine_params $@
  if [ $? -ne 0 ];then
    exit -1
  fi
  set_global_var
  ${MAP_DATACHECKER_CLIENT} $@ 2>> ${LOG_DIR}/client.log
  # PID=$!
  # trap "trap_ctrlc" INT
}

main $@
