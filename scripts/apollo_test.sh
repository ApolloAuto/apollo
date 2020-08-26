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

ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

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
    warning "Perception module can not work without GPU, all targets skipped"
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

function _disabled_test_targets_all() {
  DISABLED_TARGETS=""
  _determine_drivers_disabled
  # TODO(all): arch exceptions should be done in BUILD file level.
  _determine_localization_disabled
  _determine_perception_disabled
  _determine_planning_disabled
  _determine_map_disabled

  echo "${DISABLED_TARGETS}"
  # TODO(all): exceptions for CPU mode: should be done in BUILD file level.
  # grep -v "cnn_segmentation_test\|yolo_camera_detector_test\|unity_recognize_test\|
  # perception_traffic_light_rectify_test\|cuda_util_test"`"
}

# bazel run //modules/planning/tools:inference_demo crash
function determine_disabled_targets() {
  if [[ "$#" -eq 0 ]]; then
    _disabled_test_targets_all
    return
  fi

  for component in $@; do
    case "${component}" in
      drivers)
        _determine_drivers_disabled
        ;;
      localization)
        _determine_localization_disabled
        ;;
      perception)
        _determine_perception_disabled
        ;;
      planning)
        _determine_planning_disabled
        ;;
      map)
        _determine_map_disabled
        ;;
    esac
  done

  echo "${DISABLED_TARGETS}"
}

function determine_test_targets() {
  local targets_all
  if [[ "$#" -eq 0 ]]; then
    targets_all="//modules/... union //cyber/..."
    echo "${targets_all}"
    return
  fi

  for component in $@; do
    local test_targets
    if [[ "${component}" == "cyber" ]]; then
      test_targets="//cyber/..."
    elif [[ -d "${APOLLO_ROOT_DIR}/modules/${component}" ]]; then
      test_targets="//modules/${component}/..."
    else
      error "Oops, no such component '${component}' under <APOLLO_ROOT_DIR>/modules/ . Exiting ..."
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

function _parse_cmdline_arguments() {
  local known_options=""
  local remained_args=""

  for ((pos = 1; pos <= $#; pos++)); do #do echo "$#" "$i" "${!i}"; done
    local opt="${!pos}"
    local optarg

    case "${opt}" in
      --config=*)
        optarg="${opt#*=}"
        known_options="${known_options} ${opt}"
        ;;
      --config)
        ((++pos))
        optarg="${!pos}"
        known_options="${known_options} ${opt} ${optarg}"
        ;;
      -c)
        ((++pos))
        optarg="${!pos}"
        known_options="${known_options} ${opt} ${optarg}"
        ;;
      *)
        remained_args="${remained_args} ${opt}"
        ;;
    esac
  done
  # Strip leading whitespaces
  known_options="$(echo "${known_options}" | sed -e 's/^[[:space:]]*//')"
  remained_args="$(echo "${remained_args}" | sed -e 's/^[[:space:]]*//')"

  CMDLINE_OPTIONS="${known_options}"
  SHORTHAND_TARGETS="${remained_args}"
}

function _run_bazel_test_impl() {
  local job_args="--jobs=$(nproc) --local_ram_resources=HOST_RAM*0.7"
  bazel test ${job_args} $@
}

function bazel_test() {
  if ! "${APOLLO_IN_DOCKER}"; then
    error "The build operation must be run from within docker container"
    # exit 1
  fi

  _parse_cmdline_arguments "$@"
  CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --define USE_ESD_CAN=${USE_ESD_CAN}"

  local test_targets
  test_targets="$(determine_test_targets ${SHORTHAND_TARGETS})"

  local disabled_targets
  disabled_targets="$(determine_disabled_targets ${SHORTHAND_TARGETS})"

  info "Test Overview: "
  info "${TAB}Test Options: ${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
  info "${TAB}Test Targets: ${GREEN}${test_targets}${NO_COLOR}"
  info "${TAB}Disabled:     ${YELLOW}${disabled_targets}${NO_COLOR}"

  _run_bazel_test_impl "${CMDLINE_OPTIONS}" "$(bazel query ${test_targets} ${disabled_targets})"
}

function main() {
  if [ "${USE_GPU}" -eq 1 ]; then
    info "Your GPU is enabled to run unit tests on ${ARCH} platform."
  else
    info "Running tests under CPU mode on ${ARCH} platform."
  fi
  bazel_test $@
  success "Done unit testing ${SHORTHAND_TARGETS}."
}

main "$@"
