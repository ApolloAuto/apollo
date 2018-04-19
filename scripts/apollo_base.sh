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
    check_agreement
  else
    APOLLO_IN_DOCKER=false
  fi
  export APOLLO_IN_DOCKER
}

function set_lib_path() {
  if [ "$RELEASE_DOCKER" == 1 ];then
    source /apollo/ros/setup.bash
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/lib:/usr/local/apollo/local_integ/lib:/home/caros/secure_upgrade/depend_lib
    PY_LIB_PATH=/apollo/lib
    PY_TOOLS_PATH=/apollo/modules/tools
  else
    local ROS_SETUP="/home/tmp/ros/setup.bash"
    if [ -e "${ROS_SETUP}" ]; then
      source "${ROS_SETUP}"
    fi
    PY_LIB_PATH=${APOLLO_ROOT_DIR}/py_proto
    PY_TOOLS_PATH=${APOLLO_ROOT_DIR}/modules/tools
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/lib:/apollo/bazel-genfiles/external/caffe/lib:/home/caros/secure_upgrade/depend_lib
  fi
  PY_LIB_PATH=${PY_LIB_PATH}:/usr/local/apollo/snowboy/Python
  export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:${PY_LIB_PATH}:${PY_TOOLS_PATH}:${PYTHONPATH}
  if [ -e /usr/local/cuda-8.0/ ];then
    export PATH=/usr/local/cuda-8.0/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64:$LD_LIBRARY_PATH
    export C_INCLUDE_PATH=/usr/local/cuda-8.0/include:$C_INCLUDE_PATH
    export CPLUS_INCLUDE_PATH=/usr/local/cuda-8.0/include:$CPLUS_INCLUDE_PATH
  fi
}

function create_data_dir() {
  local DATA_DIR=""
  if [ "$RELEASE_DOCKER" != "1" ];then
    DATA_DIR="${APOLLO_ROOT_DIR}/data"
  else
    DATA_DIR="${HOME}/data"
  fi

  mkdir -p "${DATA_DIR}/log"
  mkdir -p "${DATA_DIR}/bag"
  mkdir -p "${DATA_DIR}/core"
}

function determine_bin_prefix() {
  APOLLO_BIN_PREFIX=$APOLLO_ROOT_DIR
  if [ -e "${APOLLO_ROOT_DIR}/bazel-bin" ]; then
    APOLLO_BIN_PREFIX="${APOLLO_ROOT_DIR}/bazel-bin"
  fi
  export APOLLO_BIN_PREFIX
}

function find_device() {
  # ${1} = device pattern
  local device_list=$(find /dev -name "${1}")
  if [ -z "${device_list}" ]; then
    warning "Failed to find device with pattern \"${1}\" ..."
  else
    local devices=""
    for device in $(find /dev -name "${1}"); do
      ok "Found device: ${device}."
      devices="${devices} --device ${device}:${device}"
    done
    echo "${devices}"
  fi
}

function setup_device() {
  if [ $(uname -s) != "Linux" ]; then
    echo "Not on Linux, skip mapping devices."
    return
  fi

  # setup CAN device
  for INDEX in `seq 0 3`
  do
    if [ ! -e /dev/can${INDEX} ]; then
      sudo mknod --mode=a+rw /dev/can${INDEX} c 52 $INDEX
    fi
  done

  MACHINE_ARCH=$(uname -m)
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
  fi

  # setup nvidia device
  sudo /sbin/modprobe nvidia
  sudo /sbin/modprobe nvidia-uvm
  if [ ! -e /dev/nvidia0 ];then
    sudo mknod -m 666 /dev/nvidia0 c 195 0
  fi
  if [ ! -e /dev/nvidiactl ];then
    sudo mknod -m 666 /dev/nvidiactl c 195 255
  fi
  if [ ! -e /dev/nvidia-uvm ];then
    sudo mknod -m 666 /dev/nvidia-uvm c 243 0
  fi
  if [ ! -e /dev/nvidia-uvm-tools ];then
    sudo mknod -m 666 /dev/nvidia-uvm-tools c 243 1
  fi

  if [ ! -e /dev/nvidia-uvm-tools ];then
    sudo mknod -m 666 /dev/nvidia-uvm-tools c 243 1
  fi
}

function decide_task_dir() {
  DISK=""
  if [ "$1" = "--portable-disk" ]; then
    # Try to find largest NVMe drive.
    DISK="$(df | grep "^/dev/nvme" | sort -nr -k 4 | \
        awk '{print substr($0, index($0, $6))}')"

    # Try to find largest external drive.
    if [ -z "${DISK}" ]; then
      DISK="$(df | grep "/media/${DOCKER_USER}" | sort -nr -k 4 | \
          awk '{print substr($0, index($0, $6))}')"
    fi

    if [ -z "${DISK}" ]; then
      echo "Cannot find portable disk."
      echo "Please make sure your container was started AFTER inserting the disk."
    fi
  fi

  # Default disk.
  if [ -z "${DISK}" ]; then
    DISK="/apollo"
  fi

  # Create task dir.
  BAG_PATH="${DISK}/data/bag"
  TASK_ID=$(date +%Y-%m-%d-%H-%M-%S)
  TASK_DIR="${BAG_PATH}/${TASK_ID}"
  mkdir -p "${TASK_DIR}"

  echo "Record bag to ${TASK_DIR}..."
  export TASK_DIR="${TASK_DIR}"
}

function is_stopped_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  NUM_PROCESSES="$(pgrep -c -f "modules/${MODULE_PATH}/${MODULE}")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    return 1
  else
    return 0
  fi
}

function start_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  LOG="${APOLLO_ROOT_DIR}/data/log/${MODULE}.out"
  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    eval "nohup ${APOLLO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE} \
        --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
        --log_dir=${APOLLO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
    sleep 0.5
    is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
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

function start() {
  MODULE=$1
  shift

  start_customized_path $MODULE $MODULE "$@"
}

function start_prof_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  echo "Make sure you have built with 'bash apollo.sh build_prof'"
  LOG="${APOLLO_ROOT_DIR}/data/log/${MODULE}.out"
  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    PROF_FILE="/tmp/$MODULE.prof"
    rm -rf $PROF_FILE
    BINARY=${APOLLO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE}
    eval "CPUPROFILE=$PROF_FILE $BINARY \
        --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
        --log_dir=${APOLLO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
    sleep 0.5
    is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
    if [ $? -eq 0 ]; then
      echo -e "Launched module ${MODULE} in prof mode. \nExport profile by command:"
      echo -e "${YELLOW}google-pprof --pdf $BINARY $PROF_FILE > ${MODULE}_prof.pdf${NO_COLOR}"
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

function start_prof() {
  MODULE=$1
  shift

  start_prof_customized_path $MODULE $MODULE "$@"
}

function start_fe_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  eval "${APOLLO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE} \
      --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
      --alsologtostderr --log_dir=${APOLLO_ROOT_DIR}/data/log $@"
}

function start_fe() {
  MODULE=$1
  shift

  start_fe_customized_path $MODULE $MODULE "$@"
}

function start_gdb_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  shift 2

  eval "gdb --args ${APOLLO_BIN_PREFIX}/modules/${MODULE_PATH}/${MODULE} \
      --flagfile=modules/${MODULE_PATH}/conf/${MODULE}.conf \
      --log_dir=${APOLLO_ROOT_DIR}/data/log $@"
}

function start_gdb() {
  MODULE=$1
  shift

  start_gdb_customized_path $MODULE $MODULE "$@"
}

function stop_customized_path() {
  MODULE_PATH=$1
  MODULE=$2

  pkill -f "modules/${MODULE_PATH}/${MODULE}"
  if [ $? -eq 0 ]; then
    echo "Successfully stopped module ${MODULE}."
  else
    echo "Module ${MODULE} is not running - skipping."
  fi
}

function stop() {
  MODULE=$1
  stop_customized_path $MODULE $MODULE
}

function help() {
  echo "Usage:
  ./$0 [COMMAND]"
  echo "COMMAND:
  help: this help message
  start: start the module in background
  start_fe: start the module without putting in background
  start_gdb: start the module with gdb
  stop: stop the module
  "
}

function run_customized_path() {
  local module_path=$1
  local module=$2
  local cmd=$3
  shift 3
  case $cmd in
    start)
      start_customized_path $module_path $module "$@"
      ;;
    start_fe)
      start_fe_customized_path $module_path $module "$@"
      ;;
    start_gdb)
      start_gdb_customized_path $module_path $module "$@"
      ;;
    start_prof)
      start_prof_customized_path $module_path $module "$@"
      ;;
    stop)
      stop_customized_path $module_path $module
      ;;
    help)
      help
      ;;
    *)
      start_customized_path $module_path $module $cmd "$@"
    ;;
  esac
}

# run command_name module_name
function run() {
  local module=$1
  shift
  run_customized_path $module $module "$@"
}

if [ -z $APOLLO_BASE_SOURCED ]; then
  check_in_docker
  create_data_dir
  set_lib_path
  determine_bin_prefix
  export APOLLO_BASE_SOURCED=1
fi
