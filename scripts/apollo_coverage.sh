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
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"

BAZEL_OUT="${TOP_DIR}/bazel-out" # $(bazel info output_path)
COVERAGE_HTML="${TOP_DIR}/.cache/coverage"
COVERAGE_DAT="${BAZEL_OUT}/_coverage/_coverage_report.dat"

# Note(storypku): branch coverage seems not work when running bazel coverage
# GENHTML_OPTIONS="--rc genhtml_branch_coverage=1 --highlight --legend"

ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}
USE_GPU=-1

use_cpu=-1
use_gpu=-1
use_nvidia=-1
use_amd=-1

CMDLINE_OPTIONS=
SHORTHAND_TARGETS=
DISABLED_TARGETS=

function _determine_drivers_disabled() {
  if ! ${USE_ESD_CAN}; then
    warning "ESD CAN library supplied by ESD Electronics doesn't exist."
    warning "If you need ESD CAN, please refer to:"
    warning "  third_party/can_card_library/esd_can/README.md"
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/drivers/canbus/can_client/esd/..."
  fi
}

function _determine_localization_disabled() {
  if [ "${ARCH}" != "x86_64" ]; then
    # Skip msf for non-x86_64 platforms
    DISABLED_TARGETS="${disabled} except //modules/localization/msf/..."
  fi
}

function _determine_perception_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    warning "Perception can't work without GPU, targets under 'modules/perception' skipped"
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/perception/..."
  fi
}

function _determine_planning_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/planning/learning_based/..."
  fi
}

function _determine_map_disabled() {
  if [ "${USE_GPU}" -eq 0 ]; then
    DISABLED_TARGETS="${DISABLED_TARGETS} except //modules/map/pnc_map:cuda_util_test"
  fi
}

function determine_disabled_targets() {
  if [[ "$#" -eq 0 ]]; then
    _determine_drivers_disabled
    _determine_localization_disabled
    _determine_perception_disabled
    _determine_planning_disabled
    _determine_map_disabled
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
      map*)
        _determine_map_disabled
        ;;
    esac
  done

  echo "${DISABLED_TARGETS}"
}

function determine_test_targets() {
  local targets_all
  if [ "$#" -eq 0 ]; then
    targets_all="//modules/... union //cyber/..."
    echo "${targets_all}"
    return
  fi

  for component in $@; do
    local test_targets
    if [ "${component}" = "cyber" ]; then
      test_targets="//cyber/..."
    elif [ -d "${APOLLO_ROOT_DIR}/modules/${component}" ]; then
      test_targets="//modules/${component}/..."
    else
      error "Directory <APOLLO_ROOT_DIR>/modules/${component} not found. Exiting ..."
      exit 1
    fi
    if [ -z "${targets_all}" ]; then
      targets_all="${test_targets}"
    else
      targets_all="${targets_all} union ${test_targets}"
    fi
  done
  echo "${targets_all}" | sed -e 's/^[[:space:]]*//'
}

function determine_cpu_or_gpu_coverage() {
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
    ok "Running ${GREEN}${GPU_PLATFORM} GPU${NO_COLOR} tests on ${GREEN}${ARCH}${NO_COLOR} platform."
  else
    ok "Running ${GREEN}CPU${NO_COLOR} tests on ${GREEN}${ARCH}${NO_COLOR} platform."
  fi
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

  if (( $use_cpu == 1)) && (( $use_gpu == 1 )); then
    error "${RED}Mixed use of '--config=cpu' and '--config=gpu' may" \
      "lead to unexpected behavior. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (( $use_cpu == 1)) && (( $use_nvidia == 1 )); then
    error "${RED}Mixed use of '--config=cpu' and '--config=nvidia' may" \
      "lead to unexpected behavior. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (( $use_cpu == 1)) && (( $use_amd == 1 )); then
    error "${RED}Mixed use of '--config=cpu' and '--config=amd' may" \
      "lead to unexpected behavior. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (( $use_nvidia == 1)) && (( $use_amd == 1 )); then
    error "${RED}Mixed use of '--config=amd' and '--config=nvidia':" \
      "please specify only one GPU target. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (( $use_nvidia == 1)) && (( $use_amd == -1)) && [ "$GPU_PLATFORM" == "AMD" ]; then
    error "${RED}Cross-compilation for NVIDIA GPU target is not supported on AMD GPU device':" \
      "please specify AMD or skip its specification to compile for AMD GPU target."\
      "To compile for NVIDIA GPU target NVIDIA GPU device should be installed. Exiting...${NO_COLOR}"
    exit 1
  fi
  if (( $use_amd == 1)) && (( $use_nvidia == -1)) && [ "$GPU_PLATFORM" == "NVIDIA" ]; then
    error "${RED}Cross-compilation for AMD GPU target is not supported on NVIDIA GPU device':" \
      "please specify NVIDIA or skip its specification to compile for NVIDIA GPU target."\
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
    if (( ${bazel} == 1 )); then
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
      -c)
        ((++pos))
        optarg="${!pos}"
        known_options="${known_options} ${opt} ${optarg}"
        ;;
      *)
        if (( ${bazel} == 0 )); then
          remained_args="${remained_args} ${opt}"
        elif (( ${bazel} == 2 )); then
          if (( ${known_bazel_opt} == 0 )); then
            known_options="${known_options} ${bazel_option}"
          fi
          bazel=0
        fi
        ;;
    esac
  done
  if (( ${bazel} == 1 )); then
    warning "Bazel option is not specified. Skipping..."
  fi
  # Strip leading whitespaces
  known_options="$(echo "${known_options}" | sed -e 's/^[[:space:]]*//')"
  remained_args="$(echo "${remained_args}" | sed -e 's/^[[:space:]]*//')"

  CMDLINE_OPTIONS="${known_options}"
  SHORTHAND_TARGETS="${remained_args}"
}

function format_bazel_targets() {
  local targets="$(echo $@ | xargs)"
  targets="${targets// union / }"   # replace all matches of "A union B" to "A B"
  targets="${targets// except / -}" # replaces all matches of "A except B" to "A-B"
  echo "${targets}"
}

function run_bazel_coverage() {
  if ${USE_ESD_CAN}; then
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define USE_ESD_CAN=${USE_ESD_CAN}"
  fi
  CMDLINE_OPTIONS="$(echo ${CMDLINE_OPTIONS} | xargs)"

  local test_targets
  test_targets="$(determine_test_targets ${SHORTHAND_TARGETS})"

  local disabled_targets
  disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"

  # Note(storypku): Workaround for "/usr/bin/bazel: Argument list too long"
  ## bazel coverage ${CMDLINE_OPTIONS} ${job_args} $(bazel query ${test_targets} ${disabled_targets})
  local formatted_targets="$(format_bazel_targets ${test_targets} ${disabled_targets})"

  info "Coverage Overview: "
  info "${TAB}USE_GPU: ${USE_GPU}  [ 0 for CPU, 1 for GPU ]"
  info "${TAB}Coverage Options: ${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
  info "${TAB}Coverage Targets: ${GREEN}${test_targets}${NO_COLOR}"
  info "${TAB}Disabled: ${YELLOW}${disabled_targets}${NO_COLOR}"

  local count="$(($(nproc) / 2))"
  local job_args="--jobs=${count} --local_ram_resources=HOST_RAM*0.7"
  bazel coverage ${CMDLINE_OPTIONS} ${job_args} -- ${formatted_targets}
}

function main() {
  if ! "${APOLLO_IN_DOCKER}"; then
    error "Coverage test must be run from within the docker container"
    exit 1
  fi

  parse_cmdline_arguments "$@"
  determine_cpu_or_gpu_coverage
  run_bazel_coverage $@
  genhtml "${COVERAGE_DAT}" --output-directory "${COVERAGE_HTML}"
  success "Done bazel coverage for ${SHORTHAND_TARGETS:-Apollo}. "
  info "Coverage report was generated under ${COVERAGE_HTML}"
}

main "$@"
