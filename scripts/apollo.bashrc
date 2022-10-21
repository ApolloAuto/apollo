#!/usr/bin/env bash

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

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
APOLLO_IN_DOCKER=false

# If inside docker container
if [ -f /.dockerenv ]; then
  APOLLO_IN_DOCKER=true
  APOLLO_ROOT_DIR="/apollo"
fi

export APOLLO_CONFIG_HOME="${APOLLO_CONFIG_HOME:=$HOME/.apollo}"

export APOLLO_ROOT_DIR="${APOLLO_ROOT_DIR}"
export APOLLO_IN_DOCKER="${APOLLO_IN_DOCKER}"
export APOLLO_CACHE_DIR="${APOLLO_ROOT_DIR}/.cache"
export APOLLO_SYSROOT_DIR="/opt/apollo/sysroot"

export TAB="    " # 4 spaces

export GPU_SETUP_COMPLETED

source ${APOLLO_ROOT_DIR}/scripts/common.bashrc

: ${VERBOSE:=yes}

BOLD='\033[1m'
RED='\033[0;31m'
BLUE='\033[1;34;48m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
  (echo >&2 -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
  (echo >&2 -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
  (echo >&2 -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

function print_delim() {
  echo "=============================================="
}

function get_now() {
  date +%s
}

function time_elapsed_s() {
  local start="${1:-$(get_now)}"
  local end="$(get_now)"
  echo "$end - $start" | bc -l
}

function success() {
  print_delim
  ok "$1"
  print_delim
}

function fail() {
  print_delim
  error "$1"
  print_delim
  exit 1
}

function determine_gpu_use_target() {
  local arch="$(uname -m)"
  local gpu_platform="UNKNOWN"
  local use_gpu=0
  local nv=0
  local amd=0
  local need_cuda=0
  local need_rocm=0
  if [[ "$USE_GPU_TARGET" != "0" ]]; then
      if [[ "${arch}" == "aarch64" ]]; then
        if lsmod | grep -q nvgpu; then
          if ldconfig -p | grep -q cudart; then
            use_gpu=1
          fi
        fi
      else ## x86_64 mode
        # Check the existence of nvidia-smi and rocm-smi
        if [[ ! -x "$(command -v nvidia-smi)" ]]; then
          nv=1
          info ${YELLOW}"No nvidia-smi found."${NO_COLOR}
        elif [[ -z "$(nvidia-smi)" ]]; then
          nv=2
          info ${YELLOW}"No NVIDIA GPU device found."${NO_COLOR}
        fi
        if [[ ! -x "$(command -v rocm-smi)" ]]; then
          amd=1
          info ${YELLOW}"No rocm-smi found."${NO_COLOR}
        elif [[ -z "$(rocm-smi)" ]]; then
          amd=2
          info ${YELLOW}"No AMD GPU device found."${NO_COLOR}
        fi
        if (( $nv == 0 )); then
          use_gpu=1
          need_cuda=1
          gpu_platform="NVIDIA"
          info ${GREEN}"NVIDIA GPU device found."${NO_COLOR}
        elif (( $amd == 0 )); then
          use_gpu=1
          need_rocm=1
          gpu_platform="AMD"
        else
          gpu_platform="UNKNOWN"
          warning ${YELLOW}"No any GPU device found."${NO_COLOR}
        fi
        if (( $amd == 0 )); then
          info ${GREEN}"AMD GPU device found."${NO_COLOR}
        fi
        if (( $nv == 0)) && (( $amd == 0 )); then
          info ${GREEN}"NVIDIA GPU device is chosen for the build."${NO_COLOR}
        fi
      fi
  fi
  export TF_NEED_CUDA="${need_cuda}"
  export TF_NEED_ROCM="${need_rocm}"
  export USE_GPU_TARGET="${use_gpu}"
  export GPU_PLATFORM="${gpu_platform}"
}

function file_ext() {
  local filename="$(basename $1)"
  local actual_ext="${filename##*.}"
  if [[ "${actual_ext}" == "${filename}" ]]; then
    actual_ext=""
  fi
  echo "${actual_ext}"
}

function c_family_ext() {
  local actual_ext
  actual_ext="$(file_ext $1)"
  for ext in "h" "hh" "hxx" "hpp" "cxx" "cc" "cpp" "cu"; do
    if [[ "${ext}" == "${actual_ext}" ]]; then
      return 0
    fi
  done
  return 1
}

function find_c_cpp_srcs() {
  find "$@" -type f -name "*.h" \
    -o -name "*.c" \
    -o -name "*.hpp" \
    -o -name "*.cpp" \
    -o -name "*.hh" \
    -o -name "*.cc" \
    -o -name "*.hxx" \
    -o -name "*.cxx" \
    -o -name "*.cu"
}

function proto_ext() {
  if [[ "$(file_ext $1)" == "proto" ]]; then
    return 0
  else
    return 1
  fi
}

function find_proto_srcs() {
  find "$@" -type f -name "*.proto"
}

function py_ext() {
  if [[ "$(file_ext $1)" == "py" ]]; then
    return 0
  else
    return 1
  fi
}

function find_py_srcs() {
  find "$@" -type f -name "*.py"
}

function bash_ext() {
  local actual_ext
  actual_ext="$(file_ext $1)"
  for ext in "sh" "bash" "bashrc"; do
    if [[ "${ext}" == "${actual_ext}" ]]; then
      return 0
    fi
  done
  return 1
}

function bazel_extended() {
  local actual_ext="$(file_ext $1)"
  if [[ -z "${actual_ext}" ]]; then
    if [[ "${arg}" == "BUILD" || "${arg}" == "WORKSPACE" ]]; then
      return 0
    else
      return 1
    fi
  else
    for ext in "BUILD" "bazel" "bzl"; do
      if [[ "${ext}" == "${actual_ext}" ]]; then
        return 0
      fi
    done
    return 1
  fi
}

function prettier_ext() {
  local actual_ext
  actual_ext="$(file_ext $1)"
  for ext in "md" "json" "yml"; do
    if [[ "${ext}" == "${actual_ext}" ]]; then
      return 0
    fi
  done
  return 1
}

function find_prettier_srcs() {
  find "$@" -type f -name "*.md" \
    -or -name "*.json" \
    -or -name "*.yml"
}

# Exits the script if the command fails.
function run() {
  if [ "${VERBOSE}" = yes ]; then
    echo "${@}"
    "${@}" || exit $?
  else
    local errfile="${APOLLO_ROOT_DIR}/.errors.log"
    echo "${@}" >"${errfile}"
    if ! "${@}" >>"${errfile}" 2>&1; then
      local exitcode=$?
      cat "${errfile}" 1>&2
      exit $exitcode
    fi
  fi
}

#commit_id=$(git log -1 --pretty=%H)
function git_sha1() {
  if [ -x "$(which git 2>/dev/null)" ] &&
    [ -d "${APOLLO_ROOT_DIR}/.git" ]; then
    git rev-parse --short HEAD 2>/dev/null || true
  fi
}

function git_date() {
  if [ -x "$(which git 2>/dev/null)" ] &&
    [ -d "${APOLLO_ROOT_DIR}/.git" ]; then
    git log -1 --pretty=%ai | cut -d " " -f 1 || true
  fi
}

function git_branch() {
  if [ -x "$(which git 2>/dev/null)" ] &&
    [ -d "${APOLLO_ROOT_DIR}/.git" ]; then
    git rev-parse --abbrev-ref HEAD
  else
    echo "@non-git"
  fi
}

function read_one_char_from_stdin() {
  local answer
  read -r -n1 answer
  # Bash 4.x+: ${answer,,} to lowercase, ${answer^^} to uppercase
  echo "${answer}" | tr '[:upper:]' '[:lower:]'
}

function optarg_check_for_opt() {
  local opt="$1"
  local optarg="$2"
  if [[ -z "${optarg}" || "${optarg}" =~ ^-.* ]]; then
      error "Missing parameter for ${opt}. Exiting..."
      exit 3
  fi
}

function setup_gpu_support() {
  if [ -z "${GPU_SETUP_COMPLETED}" ]; then
    GPU_SETUP_COMPLETED=1
    if [ -e /usr/local/cuda/ ]; then
      pathprepend /usr/local/cuda/bin
    fi
    determine_gpu_use_target
    local dev="cpu"
    if [ "${USE_GPU_TARGET}" -gt 0 ]; then
      dev="gpu"
    fi
    local torch_path="/usr/local/libtorch_${dev}/lib"
    if [ -d "${torch_path}" ]; then
      # Runtime default: for ./bazel-bin/xxx/yyy to work as expected
      pathprepend ${torch_path} LD_LIBRARY_PATH
    fi
  fi
}

if ${APOLLO_IN_DOCKER} ; then
  setup_gpu_support
fi
