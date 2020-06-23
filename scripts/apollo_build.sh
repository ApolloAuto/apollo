#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"

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
            targets="//modules/... union //cyber/... ${exceptions}"
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

function _arg_parse() {
    local __retopts="$1"
    local __retargs="$2"
    shift 2

    local known_options=""
    local remained_args=""
    while [ "$#" -gt 0 ]; do
        local opt="$1"
        local optarg

        case "${opt}" in
            --config=*)
                optarg="${opt#*=}"
                known_options="${known_options} ${opt}"
                shift
                ;;
            --config)
                optarg="${2}"; shift 2
                # check here
                known_options="${known_options} ${opt} ${optarg}"
                ;;
            -c)
                optarg="$2"; shift 2
                # check here
                known_options="${known_options} ${opt} ${optarg}"
                ;;
            *)
                remained_args="${remained_args} ${opt}"
                shift
                ;;
        esac
    done
    eval ${__retopts}="'${known_options}'"
    eval ${__retargs}="'${remained_args}'"
}

function parse_cmdline_options() {
	local remained_args=""
	local known_options=""
	_arg_parse known_options remained_args

	# FIXME(all): Use "--define USE_ESD_CAN=${USE_ESD_CAN}" instead
    local myopts="${known_options} --cxxopt=-DUSE_ESD_CAN=${USE_ESD_CAN}"

    local targets
    targets="$(determine_targets ${remained_args})"

    COMMAND_LINE_OPTIONS="${myopts}"
    BUILD_TARGETS="${targets}"

    info "Build Overview: "
    info "${TAB}Bazel Options: ${GREEN}${COMMAND_LINE_OPTIONS}${NO_COLOR}"
    info "${TAB}Build Targets: ${GREEN}${BUILD_TARGETS}${NO_COLOR}"
}

function _run_bazel_build_impl() {
    local job_args="--jobs=$(nproc)"
    bazel build --distdir="${APOLLO_CACHE_DIR}/distdir" "${job_args}" $@
}

function bazel_build() {
    if ! "${APOLLO_IN_DOCKER}" ; then
        error "The build operation must be run from within docker container"
        # exit 1
    fi

    parse_cmdline_options "$@"
    _run_bazel_build_impl "${COMMAND_LINE_OPTIONS}" "$(bazel query ${BUILD_TARGETS})"
}

function main() {
    bazel_build $@
}

main "$@"
