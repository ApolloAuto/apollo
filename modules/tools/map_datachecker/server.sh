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

set -e

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'
  echo -e "\n${RED}Usage${NONE}:
  ${BOLD}bash server.sh${NONE} COMMAND"

  echo -e "\n${RED}Commands${NONE}:
  ${BLUE}start${NONE}: start server
  ${BLUE}stop${NONE}: stop server
  "
}

function set_global_var() {
  SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  APOLLO_ROOT_PATH="${SCRIPT_PATH}/../../.."
  MAP_DATACHECKER_SERVER=${APOLLO_ROOT_PATH}/bazel-bin/modules/map/tools/map_datachecker/server/map_datachecker_server
  CONF=${APOLLO_ROOT_PATH}/modules/map/tools/map_datachecker/server/conf/map-datachecker.conf
  LOG_DIR=${SCRIPT_PATH}/log
}

function start_server() {
  if [ ! -e ${MAP_DATACHECKER_SERVER} ];then
    echo "/apollo/apollo.sh build should be executed before run this script"
    exit -1
  fi
  if [ ! -e "${LOG_DIR}" ];then
    mkdir -p ${LOG_DIR}
  fi
  server_count=`ps -ef | grep map_datachecker_server | grep -v grep | wc -l`
  if [ ${server_count} -ne 0 ];then
    echo 'Start server failed, there is already a process called map_datachecker_server'
    echo 'You can kill the preceding map_datachecker_server and rerun this command'
    exit -1
  fi
  server_log=server_`date '+%Y%m%d%H%M%S'`.log
  ${MAP_DATACHECKER_SERVER} --flagfile=${CONF} > ${LOG_DIR}/${server_log} 2>&1 &
  if [ $? -ne 0 ];then
    echo 'Start server failed'
    exit -1
  fi
  echo 'Server has been started successfully'
}

function stop_server() {
  kill_cmd="kill -INT $(ps -ef | grep map_datachecker_server | grep -v grep | awk '{print $2}')"
  server_count=`ps -ef | grep map_datachecker_server | grep -v grep | wc -l`

  if [ ${server_count} -eq 1 ];then
    ${kill_cmd}
    echo "stop server done"
  elif [ ${server_count} -eq 0 ];then
    echo "System has no server to stop"
  else
    read -p "System has more than one server, stop all server?[Y/N]" is_stop
    case ${is_stop} in
      [Yy]*)
        ${kill_cmd}
        echo "Stop server done"
        ;;
      [Nn]*)
        ;;
      esac
  fi
}

function main() {
  set_global_var
  local cmd=$1
  case $cmd in
    start)
      start_server
      ;;
    stop)
      stop_server
      ;;
    usage)
      print_usage
      ;;
    *)
      print_usage
      ;;
  esac
}

main $@


