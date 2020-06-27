#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

##========== Perception ================##
PERCEPTION_EXCEPTIONS="\
except //modules/perception/lidar/lib/detection/lidar_point_pillars:point_pillars_test \
"

##============= Localization ===================##
LOCALIZATION_EXCEPTIONS="\
except //modules/localization/ndt/ndt_locator:ndt_lidar_locator_test \
except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_test \
except //modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool_test \
except //modules/localization/msf:msf_localization_test \
except //modules/localization/msf/local_map/ndt_map:localization_msf_ndt_map_test \
except //modules/localization/msf/local_pyramid_map/ndt_map:localization_pyramid_map_ndt_map_test \
except //modules/localization/ndt:ndt_localization_pose_buffer_test \
except //modules/localization/ndt:ndt_localization_test \
except //modules/localization/ndt/ndt_locator:ndt_solver_test \
"

##============== Prediction ====================##
PREDICTION_EXCEPTIONS="\
except //modules/prediction/predictor/single_lane:single_lane_predictor_test \
except //modules/prediction/container/obstacles:obstacle_test \
except //modules/prediction/container/obstacles:obstacle_clusters_test \
except //modules/prediction/common:road_graph_test \
"

##====================== Planning ===============##
PLANNING_EXCEPTIONS="\
except //modules/planning/tasks/learning_model:learning_model_inference_task_test \
except //modules/planning/reference_line:qp_spline_reference_line_smoother_test   \
except //modules/planning/open_space/trajectory_smoother:dual_variable_warm_start_osqp_interface_test \
except //modules/planning/math/smoothing_spline:osqp_spline_2d_solver_test  \
except //modules/planning/math/smoothing_spline:osqp_spline_1d_solver_test  \
except //modules/planning/learning_based/model_inference:model_inference_test   \
except //modules/planning/integration_tests:sunnyvale_big_loop_test \
"

##======================= Failed Test Cases are Listed Above ================##
ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

CMDLINE_OPTIONS=
SHORTHAND_TARGETS=
DISABLED_TARGETS=

function _disabled_test_targets_all() {
    local disabled="${PERCEPTION_EXCEPTIONS}"
    disabled="${disabled} ${PREDICTION_EXCEPTIONS}"
    disabled="${disabled} ${LOCALIZATION_EXCEPTIONS}"
    disabled="${disabled} ${PLANNING_EXCEPTIONS}"

    if ! ${USE_ESD_CAN} ; then
        warning "ESD CAN library supplied by ESD Electronics doesn't exist."
        warning "If you need ESD CAN, please refer to:"
        warning "  third_party/can_card_library/esd_can/README.md"
        disabled="${disabled} except //modules/drivers/canbus/can_client/esd/..."
    fi
    # TODO(all): arch exceptions should be done in BUILD file level.
    if [[ "${ARCH}" != "x86_64" ]]; then
        disabled="${disabled} except //modules/localization/msf/..."
    fi
    echo "${disabled}"
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

    local disabled=
    for compo in $@ ; do
        if [[ "${compo}" == "drivers" ]]; then
            if ! ${USE_ESD_CAN} ; then
                warning "ESD CAN library supplied by ESD Electronics doesn't exist."
                warning "If you need ESD CAN, please refer to:"
                warning "  third_party/can_card_library/esd_can/README.md"
                disabled="${disabled} except //modules/drivers/canbus/can_client/esd/..."
            fi
        elif [[ "${compo}" == "localization" ]]; then
            if [[ "${ARCH}" != "x86_64" ]]; then
                disabled="${disabled} except //modules/localization/msf/..."
            fi
            disabled="${disabled} ${LOCALIZATION_EXCEPTIONS}"
        elif [[ "${compo}" == "prediction" ]]; then
            disabled="${disabled} ${PREDICTION_EXCEPTIONS}"
        elif [[ "${compo}" == "planning" ]]; then
            disabled="${disabled} ${PLANNING_EXCEPTIONS}"
        elif [[ "${compo}" == "perception" ]]; then
            disabled="${disabled} ${PERCEPTION_EXCEPTIONS}"
        fi
    done

    echo "${disabled}"
}

function determine_test_targets() {
    local targets_all
    if [[ "$#" -eq 0 ]]; then
        targets_all="//modules/... union //cyber/..."
        echo "${targets_all}"
        return
    fi

    for compo in $@ ; do
        local test_targets
        if [[ "${compo}" == "cyber" ]]; then
            test_targets="//cyber/..."
        elif [[ -d "${APOLLO_ROOT_DIR}/modules/${compo}" ]]; then
            test_targets="//modules/${compo}/..."
        else
            error "Oops, no such component '${compo}' under <APOLLO_ROOT_DIR>/modules/ . Exiting ..."
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

    for ((pos=1; pos <= $#; pos++)); do #do echo "$#" "$i" "${!i}"; done
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
    local job_args="--jobs=$(nproc)"
    bazel test --distdir="${APOLLO_CACHE_DIR}/distdir" "${job_args}" $@
}

function bazel_test() {
    if ! "${APOLLO_IN_DOCKER}" ; then
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

