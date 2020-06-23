#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

CMDLINE_OPTIONS=
SHORTHAND_TARGETS=
DISABLED_TARGETS=

function determine_disabled_test_targets() {
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
}

function determine_test_targets() {
    local targets_all
    if [[ "$#" -eq 0 ]]; then
        local exceptions=
        if ! ${USE_ESD_CAN}; then
            exceptions="$(determine_disabled_bazel_targets)"
        fi
        targets_all="//modules/... union //cyber/... ${exceptions}"
        echo "${targets_all}"
        return
    fi

    for compo in $@ ; do
        local build_targets
        if [[ "${compo}" == "drivers" ]]; then
            local exceptions=
            if ! ${USE_ESD_CAN}; then
                exceptions="$(determine_disabled_test_targets ${compo})"
            fi
            build_targets="//modules/drivers/... ${exceptions}"
        elif [[ "${compo}" == "cyber" ]]; then
            if [[ "${ARCH}" == "x86_64" ]]; then
                build_targets="//cyber/... union //modules/tools/visualizer/..."
            else
                build_targets="//cyber/..."
            fi
        elif [[ -d "${APOLLO_ROOT_DIR}/modules/${compo}" ]]; then
            build_targets="//modules/${compo}/..."
        else
            error "Oops, no such component '${compo}' under <APOLLO_ROOT_DIR>/modules/ . Exiting ..."
            exit 1
        fi
        if [ -z "${targets_all}" ]; then
            targets_all="${build_targets}"
        else
            targets_all="${targets_all} union ${build_targets}"
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
    CMDLINE_OPTIONS="${CMDLINE_OPTIONS} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN}"

    local test_targets
    test_targets="$(determine_test_targets ${SHORTHAND_TARGETS})"

    info "Test Overview: "
    info "${TAB}Bazel Options: ${GREEN}${CMDLINE_OPTIONS}${NO_COLOR}"
    info "${TAB}Test Targets: ${GREEN}${test_targets}${NO_COLOR}"
    exit 0

    _run_bazel_test_impl "${CMDLINE_OPTIONS}" "$(bazel query ${test_targets})"
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
