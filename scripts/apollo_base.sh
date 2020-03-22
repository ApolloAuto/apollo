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

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function info() {
  (>&2 echo -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
  (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
  (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
  (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
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

function check_in_docker() {
  if [ -f /.dockerenv ]; then
    APOLLO_IN_DOCKER=true
  else
    APOLLO_IN_DOCKER=false
  fi
  export APOLLO_IN_DOCKER
}

function set_lib_path() {
  export LD_LIBRARY_PATH=/usr/lib:/usr/lib/x86_64-linux-gnu

  if [ "$RELEASE_DOCKER" == 1 ]; then
    local CYBER_SETUP="/apollo/cyber/setup.bash"
    if [ -e "${CYBER_SETUP}" ]; then
      source "${CYBER_SETUP}"
    fi
    PY_LIB_PATH=/apollo/lib
    PY_TOOLS_PATH=/apollo/modules/tools
  else
    local CYBER_SETUP="/apollo/cyber/setup.bash"
    if [ -e "${CYBER_SETUP}" ]; then
      source "${CYBER_SETUP}"
    fi
    PY_LIB_PATH=${APOLLO_ROOT_DIR}/py_proto
    PY_TOOLS_PATH=${APOLLO_ROOT_DIR}/modules/tools
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/lib:/apollo/bazel-genfiles/external/caffe/lib
  fi
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/lib:/usr/local/apollo/local_integ/lib
  export LD_LIBRARY_PATH=/usr/local/adolc/lib64:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/Qt5.5.1/5.5/gcc_64/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/fast-rtps/lib:$LD_LIBRARY_PATH
  if [ "$USE_GPU" != "1" ];then
    export LD_LIBRARY_PATH=/usr/local/apollo/libtorch/lib:$LD_LIBRARY_PATH
  else
    export LD_LIBRARY_PATH=/usr/local/apollo/libtorch_gpu/lib:$LD_LIBRARY_PATH
  fi
  export LD_LIBRARY_PATH=/usr/local/apollo/boost/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/usr/local/apollo/paddlepaddle_dep/mkldnn/lib/:$LD_LIBRARY_PATH
  export PYTHONPATH=${PY_LIB_PATH}:${PY_TOOLS_PATH}:${PYTHONPATH}

  # Set teleop paths
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export PYTHONPATH=/apollo/modules/teleop/common:${PYTHONPATH}
  export PATH=/apollo/modules/teleop/common/scripts:${PATH}

  if [ -e /usr/local/cuda/ ];then
    export PATH=/usr/local/cuda/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    export C_INCLUDE_PATH=/usr/local/cuda/include:$C_INCLUDE_PATH
    export CPLUS_INCLUDE_PATH=/usr/local/cuda/include:$CPLUS_INCLUDE_PATH
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
    # soft link if sensorbox exist
    if [ -e /dev/zynq_can${INDEX} ] &&  [ ! -e /dev/can${INDEX} ]; then
      sudo ln -s /dev/zynq_can${INDEX} /dev/can${INDEX}
    fi
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
  # Try to find largest NVMe drive.
  DISK="$(df | grep "^/dev/nvme" | sort -nr -k 4 | \
      awk '{print substr($0, index($0, $6))}')"

  # Try to find largest external drive.
  if [ -z "${DISK}" ]; then
    DISK="$(df | grep "/media/${DOCKER_USER}" | sort -nr -k 4 | \
        awk '{print substr($0, index($0, $6))}')"
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
  NUM_PROCESSES="$(pgrep -c -f "modules/${MODULE_PATH}/launch/${MODULE}.launch")"
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
    eval "nohup cyber_launch start /apollo/modules/${MODULE_PATH}/launch/${MODULE}.launch &"
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
    eval "cyber_launch start /apollo/modules/${MODULE_PATH}/launch/${MODULE}.launch"
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

  cyber_launch stop "/apollo/modules/${MODULE_PATH}/launch/${MODULE}.launch"
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
cat <<EOF
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

  git status >/dev/null 2>&1
  if [ $? -ne 0 ]; then
    echo "Not in Git repo, maybe because you are in release container."
    echo "Skip log environment."
    return
  fi

  commit=$(git log -1)
  echo -e "Date:$(date)\n" >> Bag_Env_$TASK_ID.log
  git branch | awk '/\*/ { print "current branch: " $2; }'  >> Bag_Env_$TASK_ID.log
  echo -e "\nNewest commit:\n$commit"  >> Bag_Env_$TASK_ID.log
  echo -e "\ngit diff:" >> Bag_Env_$TASK_ID.log
  git diff >> Bag_Env_$TASK_ID.log
  echo -e "\n\n\n\n" >> Bag_Env_$TASK_ID.log
  echo -e "git diff --staged:" >> Bag_Env_$TASK_ID.log
  git diff --staged >> Bag_Env_$TASK_ID.log
}

# run command_name module_name
function run() {
  local module=$1
  shift
  run_customized_path $module $module "$@"
}

check_in_docker
unset OMP_NUM_THREADS
if [ $APOLLO_IN_DOCKER = "true" ]; then
  CYBER_SETUP="/apollo/cyber/setup.bash"
  if [ -e "${CYBER_SETUP}" ]; then
    source "${CYBER_SETUP}"
  fi
  create_data_dir
  set_lib_path $1
  if [ -z $APOLLO_BASE_SOURCED ]; then
    determine_bin_prefix
    export APOLLO_BASE_SOURCED=1
  fi
fi
