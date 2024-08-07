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

ARCH="$(uname -m)"

# components="$(echo -e "${@// /\\n}" | sort -u)"
# if [ ${PIPESTATUS[0]} -ne 0 ]; then ... ; fi

APOLLO_OUTSIDE_DOCKER=0
CMDLINE_OPTIONS=
SHORTHAND_TARGETS=
DISABLED_TARGETS=

: ${CROSSTOOL_VERBOSE:=0}
: ${NVCC_VERBOSE:=0}
: ${HIPCC_VERBOSE:=0}

: ${USE_ESD_CAN:=false}
USE_GPU=-1

use_cpu=-1
use_gpu=-1
use_nvidia=-1
use_amd=-1

ENABLE_PROFILER=true

function set_lib_path() {
  local CYBER_SETUP="${APOLLO_ROOT_DIR}/cyber/setup.bash"
  [ -e "${CYBER_SETUP}" ] && . "${CYBER_SETUP}"
  pathprepend ${APOLLO_ROOT_DIR}/modules/tools PYTHONPATH
  pathprepend ${APOLLO_ROOT_DIR}/modules/teleop/common PYTHONPATH
  pathprepend /apollo/modules/teleop/common/scripts
}

function site_restore() {
  [[ -e "${TOP_DIR}/WORKSPACE.source" ]] && rm -f "${TOP_DIR}/WORKSPACE" && cp "${TOP_DIR}/WORKSPACE.source" "${TOP_DIR}/WORKSPACE"
  echo "" > "${TOP_DIR}/tools/package/rules_cc.patch"
  [[ -e "${TOP_DIR}/tools/proto/proto.bzl.tpl" ]] && rm -f "${TOP_DIR}/tools/proto/proto.bzl" && cp "${TOP_DIR}/tools/proto/proto.bzl.tpl" "${TOP_DIR}/tools/proto/proto.bzl"
  if [[ -e "${TOP_DIR}/tools/package/dynamic_deps.bzl" ]]; then
    echo "STATUS = 0" > "${TOP_DIR}/tools/package/dynamic_deps.bzl"
    echo "SOURCE = {}" >> "${TOP_DIR}/tools/package/dynamic_deps.bzl"
    echo "BINARY = {}" >> "${TOP_DIR}/tools/package/dynamic_deps.bzl"
  fi
  if which buildtool > /dev/null 2>&1; then
    sudo apt remove -y apollo-neo-buildtool
  fi
  # switch back to standalone mode to increase building speed
  sed -i 's/build --spawn_strategy=sandboxed/build --spawn_strategy=standalone/' "${TOP_DIR}/tools/bazel.rc"
  # recover ld cache
  sudo bash -c "echo '/opt/apollo/sysroot/lib' > /etc/ld.so.conf.d/apollo.conf"
  sudo bash -c "echo '/usr/local/fast-rtps/lib' >> /etc/ld.so.conf.d/apollo.conf"
  sudo bash -c "echo '/opt/apollo/absl/lib' >> /etc/ld.so.conf.d/apollo.conf"
  sudo bash -c "echo '/opt/apollo/pkgs/adv_plat/lib' >> /etc/ld.so.conf.d/apollo.conf"
  return 0
}

function env_prepare() {
  set +e
  mkdir -p /opt/apollo/neo/src
  dpkg -l apollo-neo-buildtool > /dev/null 2>&1
  [[ $? -ne 0 ]] && set -e && sudo apt-get install -y ca-certificates curl gnupg && sudo install -m 0755 -d /etc/apt/keyrings &&
    curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg &&
    sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg && echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core" \
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | sudo tee /etc/apt/sources.list.d/apolloauto.list &&
    sudo apt-get update && sudo apt-get install -y apollo-neo-buildtool apollo-neo-env-manager-dev &&
    sudo touch /.installed && sudo sed -i 's/#include "flann\/general\.h"/#include <\/usr\/include\/flann\/general\.h>/g' /usr/include/flann/util/params.h
  source /opt/apollo/neo/setup.sh
  # currently, only sandboxed available in package managerment env
  if cat "${TOP_DIR}/tools/bazel.rc" | grep standalone; then
    sed -i 's/build --spawn_strategy=standalone/build --spawn_strategy=sandboxed/' "${TOP_DIR}/tools/bazel.rc"
    rm -rf "${TOP_DIR}/.cache"
  fi
  rm -rf "${TOP_DIR}/dev"
  return 0
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
  local socket_can_dev="can0"
  if [ ! -e "${can_dev}" ]; then
    warning "No CAN device named ${can_dev}. "
  fi

  if [[ -x "$(command -v ip)" ]]; then
    if ! ip link show type can | grep "${socket_can_dev}" &> /dev/null; then
      warning "No SocketCAN device named ${socket_can_dev}."
    else
      sudo modprobe can
      sudo modprobe can_raw
      sudo modprobe mttcan
      sudo ip link set "${socket_can_dev}" type can bitrate 500000 sjw 4 berr-reporting on loopback off
      sudo ip link set up "${socket_can_dev}"
    fi
  else
    warning "ip command not found."
  fi
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
  if [[ "${ARCH}" == "x86_64" ]]; then
    setup_device_for_amd64
  else
    setup_device_for_aarch64
  fi
}

function decide_task_dir() {
  # Try to find largest NVMe drive.
  DISK="$(df | grep "^/dev/nvme" | sort -nr -k 4 |
    awk '{print substr($0, index($0, $6))}')"

  # Try to find largest external drive.
  if [ -z "${DISK}" ]; then
    DISK="$(df | grep "/media/${DOCKER_USER}" | sort -nr -k 4 |
      awk '{print substr($0, index($0, $6))}')"
  fi

  if [ -z "${DISK}" ]; then
    info "Cannot find portable disk. Fallback to apollo data dir."
    DISK="/apollo"
  fi

  # Create task dir.
  BAG_PATH="${DISK}/data/bag"
  TASK_ID=$(date +%Y-%m-%d-%H-%M-%S)
  TASK_DIR="${BAG_PATH}/${TASK_ID}"
  mkdir -p "${TASK_DIR}"

  info "Record bag to ${TASK_DIR}..."
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
    # todo(zero): Better to move nohup.out to data/log/nohup.out
    eval "nohup cyber_launch start ${APOLLO_ROOT_DIR}/modules/${MODULE_PATH}/launch/${MODULE}.launch &"
    sleep 0.5
    is_stopped_customized_path "${MODULE_PATH}" "${MODULE}"
    if [ $? -eq 0 ]; then
      ok "Launched module ${MODULE}."
      return 0
    else
      error "Could not launch module ${MODULE}. Is it already built?"
      return 1
    fi
  else
    info "Module ${MODULE} is already running - skipping."
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

  info "Make sure you have built with 'bash apollo.sh build_prof'"
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
      ok "Launched module ${MODULE} in prof mode."
      echo -e " Export profile by command:\n\t${YELLOW}google-pprof --pdf $BINARY $PROF_FILE > ${MODULE}_prof.pdf${NO_COLOR}"
      return 0
    else
      error "Could not launch module ${MODULE}. Is it already built?"
      return 1
    fi
  else
    info "Module ${MODULE} is already running - skipping."
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
    info "Module ${MODULE} is already running - skipping."
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
    info "${MODULE} process is not running!"
    return
  fi

  cyber_launch stop "${APOLLO_ROOT_DIR}/modules/${MODULE_PATH}/launch/${MODULE}.launch"
  if [ $? -eq 0 ]; then
    ok "Successfully stopped module ${MODULE}."
  else
    info "Module ${MODULE} is not running - skipping."
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
    warning "Not in Git repo, maybe because you are in release container."
    info "Skip log environment."
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

function _chk_n_set_gpu_arg() {
  local arg="$1"
  if [ "${arg}" = "cpu" ]; then
    use_cpu=1
  elif [ "${arg}" = "gpu" ]; then
    use_gpu=1
  elif [ "${arg}" = "nvidia" ]; then
    use_nvidia=1
  elif [ "${arg}" = "amd" ]; then
    use_amd=1
  else
    return 0
  fi

  if (($use_cpu == 1)) && (($use_gpu == 1)); then
    error "${RED}Mixed use of '--config=cpu' and '--config=gpu' may" \
      "lead to unexpected behavior. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (($use_cpu == 1)) && (($use_nvidia == 1)); then
    error "${RED}Mixed use of '--config=cpu' and '--config=nvidia' may" \
      "lead to unexpected behavior. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (($use_cpu == 1)) && (($use_amd == 1)); then
    error "${RED}Mixed use of '--config=cpu' and '--config=amd' may" \
      "lead to unexpected behavior. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (($use_nvidia == 1)) && (($use_amd == 1)); then
    error "${RED}Mixed use of '--config=amd' and '--config=nvidia':" \
      "please specify only one GPU target. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (($use_nvidia == 1)) && (($use_amd == -1)) && [ "$GPU_PLATFORM" == "AMD" ]; then
    error "${RED}Cross-compilation for NVIDIA GPU target is not supported on AMD GPU device':" \
      "please specify AMD or skip its specification to compile for AMD GPU target." \
      "To compile for NVIDIA GPU target NVIDIA GPU device should be installed. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (($use_amd == 1)) && (($use_nvidia == -1)) && [ "$GPU_PLATFORM" == "NVIDIA" ]; then
    error "${RED}Cross-compilation for AMD GPU target is not supported on NVIDIA GPU device':" \
      "please specify NVIDIA or skip its specification to compile for NVIDIA GPU target." \
      "To compile for AMD GPU target AMD GPU device should be installed. Exiting...${NO_COLOR}"
    exit 1
  fi

  return 0
}

function parse_cmdline_arguments() {
  local known_options=""
  local bazel_option=""
  local remained_args=""
  local bazel=0

  for ((pos = 1; pos <= $#; pos++)); do #do echo "$#" "$i" "${!i}"; done
    local opt="${!pos}"
    local optarg
    local known_bazel_opt=0
    if ((${bazel} == 1)); then
      ((++bazel))
    fi
    case "${opt}" in
      --bazel)
        ((++pos))
        bazel_option="${!pos}"
        bazel=1
        ((--pos))
        ;;
      --config=*)
        optarg="${opt#*=}"
        known_options="${known_options} ${opt}"
        _chk_n_set_gpu_arg "${optarg}"
        known_bazel_opt=1
        ;;
      --config)
        ((++pos))
        optarg="${!pos}"
        known_options="${known_options} ${opt} ${optarg}"
        _chk_n_set_gpu_arg "${optarg}"
        known_bazel_opt=1
        ;;
      -o)
        ((++pos))
        optarg="${!pos}"
        known_options="${known_options} ${opt}"
        APOLLO_OUTSIDE_DOCKER=1
        ;;
      -c)
        ((++pos))
        optarg="${!pos}"
        known_options="${known_options} ${opt} ${optarg}"
        ;;
      *)
        if ((${bazel} == 0)); then
          remained_args="${remained_args} ${opt}"
        elif ((${bazel} == 2)); then
          if ((${known_bazel_opt} == 0)); then
            known_options="${known_options} ${bazel_option}"
          fi
          bazel=0
        fi
        ;;
    esac
  done
  if ((${bazel} == 1)); then
    warning "Bazel option is not specified. Skipping..."
  fi
  # Strip leading whitespaces
  known_options="$(echo "${known_options}" | sed -e 's/^[[:space:]]*//')"
  remained_args="$(echo "${remained_args}" | sed -e 's/^[[:space:]]*//')"

  CMDLINE_OPTIONS="${known_options}"
  SHORTHAND_TARGETS="${remained_args}"
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

function _determine_drivers_disabled() {
  if ! ${USE_ESD_CAN}; then
    warning "ESD CAN library supplied by ESD Electronics doesn't exist."
    warning "If you need ESD CAN, please refer to third_party/can_card_library/esd_can/README.md"
    # DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/drivers/canbus/can_client/esd/..."
    DISABLED_TARGETS="${DISABLED_TARGETS}"
  fi
}

function _determine_perception_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/perception/..."
  elif [ "$GPU_PLATFORM" == "AMD" ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/perception/common/inference/tensorrt/..."
  elif [ "$GPU_PLATFORM" == "NVIDIA" ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/perception/common/inference/migraphx/..."
  fi
}

function _determine_localization_disabled() {
  if [ "${ARCH}" != "x86_64" ]; then
    # Skip msf for non-x86_64 platforms
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/localization/msf/..."
  fi
}

function _determine_planning_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} \
        except //modules/planning/planning_open_space:planning_block"
  fi
}

function determine_disabled_targets() {
  if [[ -n "${CUSTOM_DISABLED_TARGETS}" ]]; then
    disabled_targets=($(IFS=' ' echo "${CUSTOM_DISABLED_TARGETS}"))
    for target in "${disabled_targets[@]}"; do
      DISABLED_TARGETS="${DISABLED_TARGETS} except ${target}"
    done
  fi
  if [[ "$#" -eq 0 ]]; then
    _determine_drivers_disabled
    _determine_localization_disabled
    _determine_perception_disabled
    _determine_planning_disabled
    echo "${DISABLED_TARGETS}"
    return
  fi

  for component in $@; do
    case "${component}" in
      drivers*)
        _determine_drivers_disabled
        ;;
      localization*)
        _determine_localization_disabled
        ;;
      perception*)
        _determine_perception_disabled
        ;;
      planning*)
        _determine_planning_disabled
        ;;
    esac
  done

  echo "${DISABLED_TARGETS}"
}

function determine_targets() {
  local targets_all
  if [[ "$#" -eq 0 ]]; then
    targets_all="//modules/... union //cyber/..."
    echo "${targets_all}"
    return
  fi

  for component in $@; do
    local targets
    if [ "${component}" = "cyber" ]; then
      if [[ "${HOST_OS}" == "Linux" ]]; then
        targets="//cyber/... union //modules/tools/visualizer/..."
      else
        targets="//cyber/..."
      fi
    elif [[ -d "${APOLLO_ROOT_DIR}/modules/${component}" ]]; then
      targets="//modules/${component}/..."
    else
      error "Directory <APOLLO_ROOT_DIR>/modules/${component} not found. Exiting ..."
      exit 1
    fi
    if [ -z "${targets_all}" ]; then
      targets_all="${targets}"
    else
      targets_all="${targets_all} union ${targets}"
    fi
  done
  echo "${targets_all}"
}

function format_bazel_targets() {
  local targets="$(echo $@ | xargs)"
  targets="${targets// union / }"   # replace all matches of "A union B" to "A B"
  targets="${targets// except / -}" # replaces all matches of "A except B" to "A-B"
  echo "${targets}"
}

function determine_cpu_or_gpu() {
  USE_GPU="${USE_GPU_TARGET}"
  if [ "${USE_GPU_TARGET}" -eq 0 ]; then
    if [ "${use_gpu}" -eq 1 ]; then
      error "Can't compile for GPU: no GPU found. Exiting ..."
      exit 1
    elif [ "${use_cpu}" -lt 0 ]; then
      CMDLINE_OPTIONS="--config=cpu ${CMDLINE_OPTIONS}"
    fi
    USE_GPU="0"
  else
    if [ "${use_cpu}" -eq 1 ]; then
      USE_GPU="0"
    else
      USE_GPU="1"
      if [ "${use_gpu}" -lt 0 ]; then
        CMDLINE_OPTIONS="--config=gpu ${CMDLINE_OPTIONS}"
      fi
      if [ "${use_amd}" -lt 0 ] && [ "$GPU_PLATFORM" == "AMD" ]; then
        CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --config=amd"
      elif [ "${use_nvidia}" -lt 0 ] && [ "$GPU_PLATFORM" == "NVIDIA" ]; then
        CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --config=nvidia"
      fi
    fi
  fi

  if [ "${USE_GPU}" -eq 1 ]; then
    ok "Running ${GREEN}${GPU_PLATFORM} GPU${NO_COLOR} $1 on ${GREEN}${ARCH}${NO_COLOR} platform."
  else
    ok "Running ${GREEN}CPU${NO_COLOR} $1 on ${GREEN}${ARCH}${NO_COLOR} platform."
  fi
}

function run_bazel() {
  if [ "${APOLLO_OUTSIDE_DOCKER}" -eq 1 ]; then
    warning "Assembling outside the docker can cause errors,"
    warning "  we recommend using a ready-made container."
    warning "Make sure that all dependencies are installed,"
    warning "  if errors, try running <apollo_path>/docker/build/installers/install.sh"
  elif ! "${APOLLO_IN_DOCKER}"; then
    error "The build operation must be run from within docker container"
    error "Use -o flag to force build"
    exit 1
  fi

  determine_cpu_or_gpu "${1,,}"

  if ${USE_ESD_CAN}; then
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define USE_ESD_CAN=${USE_ESD_CAN}"
  fi

  if $ENABLE_PROFILER; then
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define ENABLE_PROFILER=${ENABLE_PROFILER}"
  fi

  CMDLINE_OPTIONS="$(echo ${CMDLINE_OPTIONS} | xargs)"

  local build_targets="$(determine_targets ${SHORTHAND_TARGETS})"

  local disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"
  disabled_targets="$(echo ${disabled_targets} | xargs)"

  # Note(storypku): Workaround for in case "/usr/bin/bazel: Argument list too long"
  # bazel build ${CMDLINE_OPTIONS} ${job_args} $(bazel query ${build_targets})
  local formatted_targets="$(format_bazel_targets ${build_targets} ${disabled_targets})"

  local sp="    "
  local spaces="    "
  local count=$(nproc)
  if [ "$1" == "Coverage" ]; then
    count="$(($(nproc) / 2))"
    spaces="       "
  elif [ "$1" == "Test" ]; then
    sp="     "
  fi

  info "${BLUE}$1 Overview:${NO_COLOR}"
  info "${TAB}USE_GPU:       ${spaces}${GREEN}${USE_GPU}${NO_COLOR}  [ 0 for CPU, 1 for GPU ]"
  if [ "${USE_GPU}" -eq 1 ]; then
    info "${TAB}GPU arch:      ${spaces}${GREEN}${GPU_PLATFORM}${NO_COLOR}"
    info "${TAB}CROSSTOOL_VERBOSE: ${GREEN}${CROSSTOOL_VERBOSE}${NO_COLOR}  [ 0 for no verbose, 1 for verbose]"
    if [ "$GPU_PLATFORM" == "AMD" ]; then
      info "${TAB}HIPCC_VERBOSE: ${spaces}${GREEN}${HIPCC_VERBOSE}${NO_COLOR}  [ 0 for no verbose, 1 for cmd, 2 for env, 4 for args, 3,5,6,7 for combinations of 1,2,4]"
    elif [ "$GPU_PLATFORM" == "NVIDIA" ]; then
      info "${TAB}NVCC_VERBOSE:  ${spaces}${GREEN}${NVCC_VERBOSE}${NO_COLOR}  [ 0 for no verbose, 1 for verbose]"
    fi
  else
    info "${TAB}CPU arch:      ${spaces}${GREEN}${ARCH}${NO_COLOR}"
  fi
  info "${TAB}Bazel Options: ${spaces}${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
  info "${TAB}$1 Targets: ${sp}${GREEN}${build_targets}${NO_COLOR}"
  info "${TAB}Disabled:      ${spaces}${YELLOW}${disabled_targets}${NO_COLOR}"

  if [[ -n "${CUSTOM_JOB_ARGS}" ]]; then
    job_args="${CUSTOM_JOB_ARGS}"
  else
    if [[ $(uname -m) == "x86_64" ]]; then
      job_args="--copt=-mavx2 --host_copt=-mavx2 --jobs=${count} --local_ram_resources=HOST_RAM*0.7"
    else
      job_args="--copt=-march=native --host_copt=-march=native --jobs=${count} --local_ram_resources=HOST_RAM*0.7  --copt=-fPIC --host_copt=-fPIC"
    fi
  fi
  set -x
  [[ ${1} == "Test" ]] && CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --action_env APOLLO_CONF_PATH=${APOLLO_CONF_PATH}"
  bazel ${1,,} ${CMDLINE_OPTIONS} ${job_args} -- ${formatted_targets}
  set +x
}
