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

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}
USE_GPU=-1
USE_OPT=-1

ENABLE_PROFILER=true

CMDLINE_OPTIONS=
SHORTHAND_TARGETS=
DISABLED_TARGETS=

APOLLO_BUILD=0

function _determine_perception_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} modules/perception"
  fi
}

function determine_disabled_targets() {
  if [ "$#" -eq 0 ]; then
    _determine_perception_disabled
    echo "${DISABLED_TARGETS}"
    return
  fi

  for component in $@; do
    case "${component}" in
      modules/perception*)
        _determine_perception_disabled
        ;;
    esac
  done

  echo "${DISABLED_TARGETS}"
}

# components="$(echo -e "${@// /\\n}" | sort -u)"
# if [ ${PIPESTATUS[0]} -ne 0 ]; then ... ; fi

function determine_build_targets() {
  local targets_all
  if [[ "$#" -eq 0 ]]; then
    targets_all="$(python3 ${TOP_DIR}/scripts/find_all_package.py)"
    echo "${targets_all}"
    return
  fi

  for component in $@; do
    local build_targets
    if [ "${component}" = "cyber" ]; then
      build_targets="cyber"
    elif [[ -d "${APOLLO_ROOT_DIR}/modules/${component}" ]]; then
      build_targets="modules/${component}"
    else
      error "Directory <APOLLO_ROOT_DIR>/modules/${component} not found. Exiting ..."
      exit 1
    fi
    if [ -z "${targets_all}" ]; then
      targets_all="${build_targets}"
    else
      targets_all="${targets_all} ${build_targets}"
    fi
  done
  echo "${targets_all}"
}

function _chk_n_set_gpu_arg() {
  local arg="$1"
  local use_gpu=-1
  if [ "${arg}" = "cpu" ]; then
    use_gpu=0
  elif [ "${arg}" = "gpu" ]; then
    use_gpu=1
  else
    # Do nothing
    return 0
  fi

  if [[ "${USE_GPU}" -lt 0 || "${USE_GPU}" = "${use_gpu}" ]]; then
    USE_GPU="${use_gpu}"
    return 0
  fi

  error "Mixed use of '--config=cpu' and '--config=gpu' may" \
    "lead to unexpected behavior. Exiting..."
  exit 1
}

function _chk_n_set_opt_arg() {
  local arg="$1"
  local use_opt=-1
  if [ "${arg}" = "dbg" ]; then
    use_opt=0
  elif [ "${arg}" = "opt" ]; then
    use_opt=1
  else
    return 0
  fi

  if [[ "${USE_OPT}" -lt 0 || "${USE_OPT}" = "${use_opt}" ]]; then
    USE_OPT="${use_opt}"
    return 0
  fi

  error "Mixed use of '--config=opt' and '--config=dbg' may" \
    "lead to unexpected behavior. Exiting..."
  exit 1 
}

function parse_cmdline_arguments() {
  # local known_options=""
  local remained_args=""

  for ((pos = 1; pos <= $#; pos++)); do #do echo "$#" "$i" "${!i}"; done
    local opt="${!pos}"
    local optarg

    case "${opt}" in
      --config=*)
        optarg="${opt#*=}"
        _chk_n_set_gpu_arg "${optarg}"
        _chk_n_set_opt_arg "${optarg}"
        ;;
      --config)
        ((++pos))
        optarg="${!pos}"
        _chk_n_set_gpu_arg "${optarg}"
        _chk_n_set_opt_arg "${optarg}"
        ;;
      *)
        remained_args="${remained_args} ${opt}"
        ;;
    esac
  done
  # Strip leading whitespaces
  remained_args="$(echo "${remained_args}" | sed -e 's/^[[:space:]]*//')"

  CMDLINE_OPTIONS="${known_options}"
  SHORTHAND_TARGETS="${remained_args}"
}

function determine_build_arguments() {
  if [ "${USE_GPU}" -lt 0 ]; then
    if [ "${USE_GPU_TARGET}" -eq 0 ]; then
      CMDLINE_OPTIONS="--cpu ${CMDLINE_OPTIONS}"
    else
      CMDLINE_OPTIONS="--gpu ${CMDLINE_OPTIONS}"
    fi
    # USE_GPU unset, defaults to USE_GPU_TARGET
    USE_GPU="${USE_GPU_TARGET}"
  elif [ "${USE_GPU}" -gt "${USE_GPU_TARGET}" ]; then
    warning "USE_GPU=${USE_GPU} without GPU can't compile. Exiting ..."
    exit 1
  else
    if [ "${USE_GPU}" -eq 1 ]; then
      CMDLINE_OPTIONS="--gpu ${CMDLINE_OPTIONS}" 
    else
      CMDLINE_OPTIONS="--cpu ${CMDLINE_OPTIONS}"
    fi
  fi

  if [ "${USE_GPU}" -eq 1 ]; then
    ok "Running GPU build on ${ARCH} platform."
  else
    ok "Running CPU build on ${ARCH} platform."
  fi

  if [ "${USE_OPT}" -lt 0 ]; then
    # unset, defaults to use opt to build
    CMDLINE_OPTIONS="--opt ${CMDLINE_OPTIONS}"
  else 
    if [ "${USE_OPT}" -eq 0 ]; then
      CMDLINE_OPTIONS="--dbg ${CMDLINE_OPTIONS}"
    else
      CMDLINE_OPTIONS="--opt ${CMDLINE_OPTIONS}"
    fi
  fi
}

function drop_targets() {
  local drop_result
  local drop_pattern=${1}
  local targets=${2}
  
  for i in $targets; do
    if [[ $i == $drop_pattern* ]]; then
      continue
    fi
    drop_result="${drop_result} ${i}"
  done

  echo ${drop_result}
  return
}

function run_bazel_build() {
  # Add profiler
  if $ENABLE_PROFILER; then
    # CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define ENABLE_PROFILER=${ENABLE_PROFILER}"
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} -a \'--define\' \'ENABLE_PROFILER=${ENABLE_PROFILER}\'"
  fi

  CMDLINE_OPTIONS="$(echo ${CMDLINE_OPTIONS} | xargs)"

  local build_targets
  [[ -z ${SHORTHAND_TARGETS} ]] && APOLLO_BUILD=1
  build_targets="$(determine_build_targets ${SHORTHAND_TARGETS})"

  local disabled_targets
  disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"
  disabled_targets="$(echo ${disabled_targets} | xargs)"

  for i in ${disabled_targets}; do
    build_targets=$(drop_targets "${i}" "${build_targets}")
  done

  # Note(storypku): Workaround for in case "/usr/bin/bazel: Argument list too long"
  # bazel build ${CMDLINE_OPTIONS} ${job_args} $(bazel query ${build_targets})
  # local formatted_targets="$(format_bazel_targets ${build_targets} ${disabled_targets})"

  info "Build Overview: "
  info "${TAB}USE_GPU: ${USE_GPU}  [ 0 for CPU, 1 for GPU ]"
  info "${TAB}buildtool Options: ${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
  if [ "${APOLLO_BUILD}" -eq 1 ]; then
    info "${TAB}Build Targets:     ${GREEN}Apollo${NO_COLOR}"
  else
    info "${TAB}Build Targets:     ${GREEN}${build_targets}${NO_COLOR}"  
  fi
  info "${TAB}Disabled:          ${YELLOW}${disabled_targets}${NO_COLOR}"

  local job_args="-j=$(nproc) -m=0.7"

  # keep the bazel-extend-tools used in core and park consistent
  if [[ ! -e /opt/apollo/neo/src/tools ]]; then
    sudo apt install apollo-neo-bazel-extend-tools
  fi
  ln -snf /apollo/tools /opt/apollo/neo/src/tools 
  rm -rf /opt/apollo/neo/packages/bazel-extend-tools/latest/src
  ln -snf /apollo/tools /opt/apollo/neo/packages/bazel-extend-tools/latest/src 

  buildtool build ${CMDLINE_OPTIONS} ${job_args} -p ${build_targets}

  [[ $? -ne 0 ]] && error "Build failed!" && exit -1
}

function main() {
  if ! "${APOLLO_IN_DOCKER}"; then
    error "The build operation must be run from within docker container"
    exit 1
  fi

  env_prepare
  parse_cmdline_arguments "$@"
  determine_build_arguments

  run_bazel_build

  if [ "${APOLLO_BUILD}" -eq 1 ]; then
    SHORTHAND_TARGETS="apollo"
  fi

  success "Done building ${SHORTHAND_TARGETS}. Enjoy!"
}

main "$@"
