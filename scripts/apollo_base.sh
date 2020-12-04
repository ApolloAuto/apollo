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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source ${TOP_DIR}/scripts/apollo.bashrc

HOST_ARCH="$(uname -m)"

function set_lib_path() {
  local CYBER_SETUP="${APOLLO_ROOT_DIR}/cyber/setup.bash"
  [ -e "${CYBER_SETUP}" ] && . "${CYBER_SETUP}"

  # TODO(storypku):
  # /usr/local/apollo/local_integ/lib

  # FIXME(all): remove PYTHONPATH settings
  export PYTHONPATH="${APOLLO_ROOT_DIR}/modules/tools:${PYTHONPATH}"
  # Set teleop paths
  export PYTHONPATH="${APOLLO_ROOT_DIR}/modules/teleop/common:${PYTHONPATH}"
  add_to_path "/apollo/modules/teleop/common/scripts"
}

function create_data_dir() {
  local DATA_DIR="${APOLLO_ROOT_DIR}/data"
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

function setup_device_for_aarch64() {
  local can_dev="/dev/can0"
  if [ ! -e "${can_dev}" ]; then
    warning "No CAN device named ${can_dev}. "
    return
  fi

  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
}

function setup_device_for_amd64() {
  # setup CAN device
  local NUM_PORTS=8
  for i in $(seq 0 $((${NUM_PORTS} - 1))); do
    if [[ -e /dev/can${i} ]]; then
      continue
    elif [[ -e /dev/zynq_can${i} ]]; then
      # soft link if sensorbox exist
      sudo ln -s /dev/zynq_can${i} /dev/can${i}
    else
      break
      # sudo mknod --mode=a+rw /dev/can${i} c 52 ${i}
    fi
  done

  # Check Nvidia device
  if [[ ! -e /dev/nvidia0 ]]; then
    warning "No device named /dev/nvidia0"
  fi
  if [[ ! -e /dev/nvidiactl ]]; then
    warning "No device named /dev/nvidiactl"
  fi
  if [[ ! -e /dev/nvidia-uvm ]]; then
    warning "No device named /dev/nvidia-uvm"
  fi
  if [[ ! -e /dev/nvidia-uvm-tools ]]; then
    warning "No device named /dev/nvidia-uvm-tools"
  fi
  if [[ ! -e /dev/nvidia-modeset ]]; then
    warning "No device named /dev/nvidia-modeset"
  fi
}

function setup_device() {
  if [ "$(uname -s)" != "Linux" ]; then
    info "Not on Linux, skip mapping devices."
    return
  fi
  if [[ "${HOST_ARCH}" == "x86_64" ]]; then
    setup_device_for_amd64
  else
    setup_device_for_aarch64
  fi
}

function decide_task_dir() {
  # Try to find largest NVMe drive.
  DISK="$(df | grep "^/dev/nvme" | sort -nr -k 4 \
    | awk '{print substr($0, index($0, $6))}')"

  # Try to find largest external drive.
  if [ -z "${DISK}" ]; then
    DISK="$(df | grep "/media/${DOCKER_USER}" | sort -nr -k 4 \
      | awk '{print substr($0, index($0, $6))}')"
  fi

  if [ -z "${DISK}" ]; then
    echo "Cannot find portable disk. Fallback to apollo data dir."
    DISK="/apollo"
  fi

  # Create task dir.
  BAG_PATH="${DISK}/data/bag"
  TASK_ID=$(date +%Y-%m-%d-%H-%M-%S)
  TASK_DIR="${BAG_PATH}/${TASK_ID}"
  mkdir -p "${TASK_DIR}"

  echo "Record bag to ${TASK_DIR}..."
  export TASK_ID="${TASK_ID}"
  export TASK_DIR="${TASK_DIR}"
}

function is_stopped_customized_path() {
  MODULE_PATH=$1
  MODULE=$2
  NUM_PROCESSES="$(pgrep -f "modules/${MODULE_PATH}/launch/${MODULE}.launch" | grep -cv '^1$')"
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

  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    eval "nohup cyber_launch start ${APOLLO_ROOT_DIR}/modules/${MODULE_PATH}/launch/${MODULE}.launch &"
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

  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    eval "cyber_launch start ${APOLLO_ROOT_DIR}/modules/${MODULE_PATH}/launch/${MODULE}.launch"
  else
    echo "Module ${MODULE} is already running - skipping."
    return 2
  fi
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

  is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
  if [ $? -eq 1 ]; then
    echo "${MODULE} process is not running!"
    return
  fi

  cyber_launch stop "${APOLLO_ROOT_DIR}/modules/${MODULE_PATH}/launch/${MODULE}.launch"
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

# Note: This 'help' function here will overwrite the bash builtin command 'help'.
# TODO: add a command to query known modules.
function help() {
  cat << EOF
Invoke ". scripts/apollo_base.sh" within docker to add the following commands to the environment:
Usage: COMMAND [<module_name>]

COMMANDS:
  help:      show this help message
  start:     start the module in background
  start_fe:  start the module without putting in background
  start_gdb: start the module with gdb
  stop:      stop the module
EOF
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

# Write log to a file about the env when record a bag.
function record_bag_env_log() {
  if [ -z "${TASK_ID}" ]; then
    TASK_ID=$(date +%Y-%m-%d-%H-%M)
  fi

  git status > /dev/null 2>&1
  if [ $? -ne 0 ]; then
    echo "Not in Git repo, maybe because you are in release container."
    echo "Skip log environment."
    return
  fi

  commit=$(git log -1)
  echo -e "Date:$(date)\n" >> Bag_Env_$TASK_ID.log
  git branch | awk '/\*/ { print "current branch: " $2; }' >> Bag_Env_$TASK_ID.log
  echo -e "\nNewest commit:\n$commit" >> Bag_Env_$TASK_ID.log
  echo -e "\ngit diff:" >> Bag_Env_$TASK_ID.log
  git diff >> Bag_Env_$TASK_ID.log
  echo -e "\n\n\n\n" >> Bag_Env_$TASK_ID.log
  echo -e "git diff --staged:" >> Bag_Env_$TASK_ID.log
  git diff --staged >> Bag_Env_$TASK_ID.log
}

# run command_name module_name
function run_module() {
  local module=$1
  shift
  run_customized_path $module $module "$@"
}

unset OMP_NUM_THREADS

if [ $APOLLO_IN_DOCKER = "true" ]; then
  create_data_dir
  set_lib_path $1
  if [ -z $APOLLO_BASE_SOURCED ]; then
    determine_bin_prefix
    export APOLLO_BASE_SOURCED=1
  fi
fi
