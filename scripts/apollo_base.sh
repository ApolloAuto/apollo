#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

RED='\033[0;31m'
YELLOW='\e[33m'
NO_COLOR='\033[0m'

function info() {
  (>&2 echo -e "[\e[34m\e[1mINFO\e[0m] $*")
}

function error() {
  (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
  (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
  (>&2 echo -e "[\e[32m\e[1m OK \e[0m] $*")
}

function print_delim() {
  echo '============================'
}

function get_now() {
  echo $(date +%s)
}

function print_time() {
  END_TIME=$(get_now)
  ELAPSED_TIME=$(echo "$END_TIME - $START_TIME" | bc -l)
  MESSAGE="Took ${ELAPSED_TIME} seconds"
  info "${MESSAGE}"
}

function success() {
  print_delim
  ok "$1"
  print_time
  print_delim
}

function fail() {
  print_delim
  error "$1"
  print_time
  print_delim
  exit -1
}

# Check whether user has agreed license agreement
function check_agreement() {
  agreement_record="$HOME/.cache/.apollo_agreement.txt"
  if [ ! -e "$agreement_record" ]; then
     AGREEMENT_FILE="$APOLLO_ROOT_DIR/scripts/AGREEMENT.txt"
     if [ ! -e "$AGREEMENT_FILE" ]; then
          error "AGREEMENT $AGREEMENT_FILE does not exist."
          exit 0
     fi
     cat $AGREEMENT_FILE
     tip="Type 'y' or 'Y' to agree to the license agreement above, or type any other key to exit"
     echo $tip
     read -n 1 user_agreed
     if [ "$user_agreed" == "y" ] || [ "$user_agreed" == "Y" ]; then
         rm -rf $agreement_record
         cat $AGREEMENT_FILE >> $agreement_record
         echo "$tip" >> $agreement_record
         echo "$user_agreed" >> $agreement_record
     else
         exit 0
     fi
  fi
}

function check_in_docker() {
  if [ -f /.dockerenv ]; then
      APOLLO_IN_DOCKER=true
  else
      APOLLO_IN_DOCKER=false
  fi
  export APOLLO_IN_DOCKER
}

function set_lib_path() {
  if [ "$RELEASE_DOCKER" == 1 ];then
      source /apollo/ros/setup.bash
      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/lib
      PY_LIB_PATH=/apollo/lib
      PY_TOOLS_PATH=/apollo/modules/tools
  else
      local MD5=`echo -n $APOLLO_ROOT_DIR | md5sum | cut -d' ' -f1`
      local ROS_SETUP="${HOME}/.cache/bazel/_bazel_${USER}/${MD5}/external/ros/setup.bash"
      if [ -e "${ROS_SETUP}" ]; then
         source "${ROS_SETUP}"
      fi
      PY_LIB_PATH=${APOLLO_ROOT_DIR}/py_proto
      PY_TOOLS_PATH=${APOLLO_ROOT_DIR}/modules/tools
      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/lib:/apollo/bazel-genfiles/external/caffe/lib
  fi
  export PYTHONPATH=${PY_LIB_PATH}:${PY_TOOLS_PATH}:${PYTHONPATH}
}

function create_data_dir() {
  if [ ! -e "${APOLLO_ROOT_DIR}/data/log" ]; then
      mkdir -p "${APOLLO_ROOT_DIR}/data/log"
  fi

  if [ ! -e "${APOLLO_ROOT_DIR}/data/bag" ]; then
      mkdir -p "${APOLLO_ROOT_DIR}/data/bag"
  fi

  if [ ! -e "${APOLLO_ROOT_DIR}/data/core" ]; then
      mkdir -p "${APOLLO_ROOT_DIR}/data/core"
  fi
}

function determine_bin_prefix() {
  APOLLO_BIN_PREFIX=$APOLLO_ROOT_DIR
  if [ -e "${APOLLO_ROOT_DIR}/bazel-bin" ]; then
      APOLLO_BIN_PREFIX="${APOLLO_ROOT_DIR}/bazel-bin"
  fi
  export APOLLO_BIN_PREFIX
}

function is_stopped() {
    MODULE=${1}
    NUM_PROCESSES="$(pgrep -c -f "modules/${MODULE}/${MODULE}")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        return 1
    else
        return 0
    fi
}

function start() {
    MODULE=$1
    shift

    LOG="${APOLLO_ROOT_DIR}/data/log/${MODULE}.out"
    is_stopped "${MODULE}"
    if [ $? -eq 1 ]; then
        eval "nohup ${APOLLO_BIN_PREFIX}/modules/${MODULE}/${MODULE} \
            --flagfile=modules/${MODULE}/conf/${MODULE}.conf \
            --log_dir=${APOLLO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
        sleep 0.5
        is_stopped "${MODULE}"
        if [ $? -eq 0 ]; then
            echo "Launched module ${MODULE}."
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

function start_fe() {
    MODULE=$1
    shift

    eval "${APOLLO_BIN_PREFIX}/modules/${MODULE}/${MODULE} \
        --flagfile=modules/${MODULE}/conf/${MODULE}.conf \
        --log_dir=${APOLLO_ROOT_DIR}/data/log $@"
}

function stop() {
    MODULE=$1
    pkill -SIGINT -f "modules/${MODULE}/${MODULE}"
    if [ $? -eq 0 ]; then
        echo "Successfully stopped module ${MODULE}."
    else
        echo "Module ${MODULE} is not running - skipping."
    fi
}

function help() {
  echo "Usage:
  ./$0 [COMMAND]"
  echo "COMMAND:
  help: this help message
  start: start the module in background
  start_fe: start the module without putting in background
  stop: stop the module
  "
}

# run command_name module_name
function run() {
    local module=$1
    local cmd=$2
    shift 2
    case $cmd in
        start)
            start $module "$@"
            ;;
        start_fe)
            start_fe $module "$@"
            ;;
        stop)
            stop $module
            ;;
        help)
            help
            ;;
        *)
            start $module "$@"
            ;;
    esac
}

check_agreement
check_in_docker
create_data_dir
set_lib_path
determine_bin_prefix
