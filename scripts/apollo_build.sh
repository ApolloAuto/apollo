#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}
: ${USE_GPU:=0}
: ${USE_ESD_CAN:=false}

COMMAND_LINE_OPTIONS=
BUILD_TARGETS=

function determine_disabled_targets() {
    local disabled=
    local compo="$1"
    if [[ -z "${compo}" || "${compo}" == "drivers" ]]; then
        if ! ${USE_ESD_CAN} ; then
            warning "ESD CAN library supplied by ESD Electronics doesn't exist."
            warning "If you need ESD CAN, please refer to:"
            warning "  third_party/can_card_library/esd_can/README.md"
            disabled="${disabled} except //modules/drivers/canbus/can_client/esd/..."
        fi
    elif [[ "${compo}" == "localization" && "${ARCH}" != "x86_64" ]]; then
        # Skip msf for non-x86_64 platforms
        disabled="${disabled} except //modules/localization/msf/..."
    fi

    echo "${disabled}"
    # DISABLED_CYBER_MODULES="except //cyber/record:record_file_integration_test"
}

# components="$(echo -e "${@// /\\n}" | sort -u)"
function determine_targets() {
    local targets=
    local compo="$1"
    if [[ -z "${compo}" || "${compo}" == "drivers" ]]; then
        local exceptions=
        if ! ${USE_ESD_CAN}; then
            exceptions="$(determine_disabled_targets ${compo})"
        fi
        if [ -z "${compo}" ]; then
            targets="//... ${exceptions}"
        else
            targets="//modules/drivers/... ${exceptions}"
        fi
    elif [[ "${compo}" == "cyber" ]]; then
        if [[ "${ARCH}" == "x86_64" ]]; then
            targets="//cyber/... union //modules/tools/visualizer/..."
        else
            targets="//cyber/..."
        fi
    elif [[ -d "${APOLLO_ROOT_DIR}/modules/${compo}" ]]; then
        targets="//modules/${compo}/..."
    else
        error "Oops, no such component under <APOLLO_ROOT_DIR>/modules/ . Exiting ..."
        exit 1
    fi
    echo "${targets}"
}

function parse_cmdline_options() {
    local build_mode="build"
    local compilation_mode="fastbuild"
    local args_to_pass_on=""

    while [ "$#" -gt 0 ]; do
        local option="$1"
        case "${option}" in
            --mode)
                build_mode="$(_check_build_mode $2)"; shift 2
                ;;
            -c|--compilation_mode)
                compilation_mode="$(_check_compilation_mode $2)"; shift 2
                ;;
            *)
                # Pass arguments we don't handle to bazel
                args_to_pass_on="${args_to_pass_on} ${option}"; shift
                ;;
        esac
    done

    local myopts=""
    if [ "${compilation_mode}" != "fastbuild" ]; then
        myopts="${myopts} -c ${compilation_mode}"
    fi
    # TODO(all):
    # Corner cases: build_gpu if env USE_GPU=0, build_cpu if env USE_GPU=1
    # And, interaction with apollo.bazelrc
    if [ "${build_mode}" == "build_cpu" ]; then
        myopts="${myopts} --cxxopt=-DCPU_ONLY"
    elif [ "${USE_GPU}" -eq 1 ]; then
        myopts="${myopts} --config=gpu"
    fi
    myopts="${myopts} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN}"

    local targets
    targets="$(determine_targets ${args_to_pass_on})"

    COMMAND_LINE_OPTIONS="${myopts}"
    BUILD_TARGETS="${targets}"

    info "Build Overview: "
    info "${TAB}Bazel Options: ${GREEN}${COMMAND_LINE_OPTIONS}${NO_COLOR}"
    info "${TAB}Build Targets: ${GREEN}${BUILD_TARGETS}${NO_COLOR}"
}

function _check_build_mode() {
    local supported_modes=" build build_cpu build_gpu "
    local mode="$1"

    if ! optarg_check_for_opt "--mode" "${mode}" ; then
        exit 1
    fi

    if [[ "${supported_modes}" != *" ${mode} "* ]]; then
        error "Unknown build mode: ${mode}. Supported values:"
        error " ${supported_modes}"
        exit 1
    fi

    echo "${mode}"
}

function _check_compilation_mode() {
    local supported_modes=" fastbuild dbg opt "
    local mode="$1"

    if ! optarg_check_for_opt "-c" "${mode}" ; then
        exit 1
    fi

    if [[ "${supported_modes}" != *" ${mode} "* ]]; then
        error "Unknown compilation mode: ${mode}. Supported values:"
        error " ${supported_modes}"
        exit 1
    fi
    echo "${mode}"
}

function bazel_build() {
    parse_cmdline_options "$@"

    if ! "${APOLLO_IN_DOCKER}" ; then
        error "The build operation must be run from within docker container"
        exit 1
    fi

    run bazel build --distdir="${APOLLO_CACHE_DIR}" "${COMMAND_LINE_OPTIONS}" "${BUILD_TARGETS}"
}

function main() {
    bazel_build "$@"
}

main "$@"
