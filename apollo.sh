#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"
SUPPORTED_ARCHS=" x86_64 aarch64 "
APOLLO_VERSION="@non-git"
APOLLO_ENV=""

USE_ESD_CAN=false
: ${STAGE:=dev}

function check_architecture_support() {
    if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then
        error "Unsupported CPU arch: ${ARCH}. Currently, Apollo only" \
              "supports running on the following CPU archs:"
        error "${TAB}${SUPPORTED_ARCHS}"
        exit 1
    fi
}

function check_platform_support() {
    local platform="$(uname -s)"
    if [ "$platform" != "Linux" ]; then
        error "Unsupported platform: ${platform}."
        error "${TAB}Apollo is expected to run on Linux systems (E.g., Debian/Ubuntu)."
        exit 1
    fi
}

function check_minimal_memory_requirement() {
    local minimal_mem_gb="2.0"
    local actual_mem_gb="$(free -m | awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}')"
    if (( $(echo "$actual_mem_gb < $minimal_mem_gb" | bc -l) )); then
        warning "System memory [${actual_mem_gb}G] is lower than the minimum required" \
                "[${minimal_mem_gb}G]. Apollo build could fail."
    fi
}

function determine_esdcan_use() {
    local esdcan_dir="${APOLLO_ROOT_DIR}/third_party/can_card_library/esd_can"
    local use_esd=false
    if [ -f "${esdcan_dir}/include/ntcan.h" ] && \
       [ -f "${esdcan_dir}/lib/libntcan.so.4" ]; then
        use_esd=true
    fi
    USE_ESD_CAN="${use_esd}"
}

function check_apollo_version() {
    local branch="$(git_branch)"
    if [ "${branch}" == "${APOLLO_VERSION}" ]; then
        return
    fi
    local sha1="$(git_sha1)"
    local stamp="$(git_date)"
    APOLLO_VERSION="${branch}-${stamp}-${sha1}"
}

function apollo_env_setup() {
    check_apollo_version

    check_architecture_support
    check_platform_support
    check_minimal_memory_requirement
    # determine_gpu_use # work done by scripts/apollo.bashrc
    determine_esdcan_use

    APOLLO_ENV="${APOLLO_ENV} STAGE=${STAGE}"
    APOLLO_ENV="${APOLLO_ENV} USE_ESD_CAN=${USE_ESD_CAN}"
    # Add more here ...

    info "Apollo Environment Settings:"
    info "${TAB}APOLLO_ROOT_DIR: ${APOLLO_ROOT_DIR}"
    info "${TAB}APOLLO_CACHE_DIR: ${APOLLO_CACHE_DIR}"
    info "${TAB}APOLLO_IN_DOCKER: ${APOLLO_IN_DOCKER}"
    info "${TAB}APOLLO_VERSION: ${APOLLO_VERSION}"
    info "${TAB}APOLLO_ENV: ${APOLLO_ENV} USE_GPU=${USE_GPU}"

    if [ ! -f "${APOLLO_ROOT_DIR}/.apollo.bazelrc" ]; then
        env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_config.sh" --noninteractive
    fi
}

#TODO(all): Update node modules
function build_dreamview_frontend() {
    pushd "${APOLLO_ROOT_DIR}/modules/dreamview/frontend" >/dev/null
        yarn build
    popd >/dev/null
}

function build_test_and_lint() {
    env ${APOLLO_ENV} bash "${build_sh}" --config=cpu
    env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test --config=cpu
    env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh" cpp
    success "Build and Test and Lint finished."
}

function _usage() {
  echo -e "\n${RED}Usage${NO_COLOR}:
  .${BOLD}/apollo.sh${NO_COLOR} [OPTION]"
  echo -e "\n${RED}Options${NO_COLOR}:
  ${BLUE}config [options]${NO_COLOR}: config bazel build environment either non-interactively (default) or interactively.
  ${BLUE}build [module]${NO_COLOR}: run build for cyber (<module> = cyber) or modules/<module>.  If <module> unspecified, build all.
  ${BLUE}build_dbg [module]${NO_COLOR}: run debug build (default).
  ${BLUE}build_opt [module]${NO_COLOR}: run optimized build.
  ${BLUE}build_cpu [module]${NO_COLOR}: build in CPU mode. Equivalent to 'bazel build --config=cpu'
  ${BLUE}build_gpu [module]${NO_COLOR}: run build in GPU mode. Equivalent to 'bazel build --config=gpu'
  ${BLUE}build_opt_gpu [module]${NO_COLOR}: optimized build in GPU mode. Equivalent to 'bazel build --config=opt --config=gpu'
  ${BLUE}test [module]${NO_COLOR}: run unit tests for cyber (<module> = cyber) or modules/<module>. If <module> unspecified, unit test all.
  ${BLUE}coverage [module]${NO_COLOR}: run bazel coverage test for cyber (<module> = cyber) or modules/<module>. If <module> unspecified, coverage test all.
  ${BLUE}lint${NO_COLOR}: run code style check
  ${BLUE}buildify${NO_COLOR}: run 'buildifier' to fix style of bazel files
  ${BLUE}check${NO_COLOR}: run build, test and lint on all modules. Make sure check pass before checking in new code
  ${BLUE}build_fe${NO_COLOR}: compile dreamland frontend javascript. Requires all the node_modules installed already
  ${BLUE}build_teleop${NO_COLOR}: run build with teleop enabled.
  ${BLUE}build_prof [module]${NO_COLOR}: build with perf profiling support. Not implemented yet.
  ${BLUE}doc${NO_COLOR}: generate doxygen document
  ${BLUE}clean${NO_COLOR}: cleanup, including 'bazel clean --async'.
  ${BLUE}usage${NO_COLOR}: show this message
  "
}

function main() {
    apollo_env_setup
    if [ "$#" -eq 0 ]; then
        exit 0
    fi
    local build_sh="${APOLLO_ROOT_DIR}/scripts/apollo_build.sh"
    local test_sh="${APOLLO_ROOT_DIR}/scripts/apollo_test.sh"
    local coverage_sh="${APOLLO_ROOT_DIR}/scripts/apollo_coverage.sh"
    local ci_sh="${APOLLO_ROOT_DIR}/scripts/apollo_ci.sh"
    local cmd="$1"; shift
    case "${cmd}" in
        config)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_config.sh" "$@"
            ;;
        build)
            env ${APOLLO_ENV} bash "${build_sh}" "$@"
            ;;
        build_opt)
            env ${APOLLO_ENV} bash "${build_sh}" --config=opt "$@"
            ;;
        build_dbg)
            env ${APOLLO_ENV} bash "${build_sh}" --config=dbg "$@"
            ;;
        build_cpu)
            env ${APOLLO_ENV} bash "${build_sh}" --config=cpu "$@"
            ;;
        build_gpu)
            env ${APOLLO_ENV} bash "${build_sh}" --config=gpu "$@"
            ;;
        build_opt_gpu)
            env ${APOLLO_ENV} bash "${build_sh}" --config=opt_gpu "$@"
            ;;
        build_prof)
            env ${APOLLO_ENV} bash "${build_sh}" --config=cpu_prof "$@"
            ;;
        build_teleop)
            env ${APOLLO_ENV} bash "${build_sh}" --config=teleop "$@"
            ;;
        build_fe)
            build_dreamview_frontend
            ;;
        test)
            env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test "$@"
            ;;
        coverage)
            env ${APOLLO_ENV} bash "${coverage_sh}" "$@"
            ;;
        cibuild)
            env ${APOLLO_ENV} bash "${ci_sh}" "build"
            ;;
        citest)
            env ${APOLLO_ENV} bash "${ci_sh}" "test"
            ;;
        check)
            build_test_and_lint
            ;;
        buildify)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_buildify.sh"
            ;;
        lint)
            # FIXME(all): apollo_lint.sh "$@" when bash/python scripts are ready.
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh" cpp
            ;;
        clean)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_clean.sh" "$@"
            ;;
        doc)
            env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_docs.sh" "$@"
            ;;
        usage)
            _usage
            ;;
        -h|--help)
            _usage
            ;;
        *)
            _usage
            ;;
    esac
}

main "$@"

