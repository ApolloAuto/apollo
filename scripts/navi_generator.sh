#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "${DIR}/apollo_base.sh"

function start() {
  local MODULE_PATH=$1
  local MODULE=$1
  shift 2
  # Start monitor.
  supervisorctl start monitor > /dev/null
  LOG="${APOLLO_ROOT_DIR}/data/log/${MODULE}.out"
  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    eval "nohup ${APOLLO_BIN_PREFIX}/modules/tools/${MODULE_PATH}/${MODULE} \
        --flagfile=modules/tools/${MODULE_PATH}/conf/${MODULE}.conf \
        --log_dir=${APOLLO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
    sleep 0.5
    is_stopped_navi_path "${MODULE_PATH}" "${MODULE}"
    if [ $? -eq 0 ]; then
      echo "Launched module ${MODULE}."
      echo "${MODULE} is running at http://localhost:9888"
      return 0
    else
      echo "Could not launch module ${MODULE}. Is it already built?"
      return 1
    fi
  else
    echo "Module ${MODULE} is already running - skipping."
    return 2
  fi
}

function is_stopped_navi_path() {
  MODULE_PATH=$1
  MODULE=$2
  NUM_PROCESSES="$(pgrep -c -f "modules/tools/${MODULE_PATH}/${MODULE}")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    return 1
  else
    return 0
  fi
}

function stop() {
  MODULE=$1

  pkill -SIGKILL -f "modules/tools/${MODULE}/${MODULE}"
  if [ $? -eq 0 ]; then
    echo "Successfully stopped module ${MODULE}."
  else
    echo "Module ${MODULE} is not running - skipping."
  fi
  supervisorctl stop monitor
}

# run function from apollo_base.sh
# run command_name module_name
case $1 in
  start)
    start navi_generator "$@"
    ;;
  stop)
    stop navi_generator "$@"
    ;;
  *)
    start navi_generator "$@"
    ;;
esac