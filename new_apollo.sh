#! /usr/bin/env bash
set -e

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
APOLLO_IN_DOCKER=false
ARCH="$(uname -m)"
SUPPORTED_ARCHS=" x86_64 aarch64 "
APOLLO_VERSION="@non-git"

: ${USE_GPU:=0}
: ${STAGE:=dev}

function check_if_in_docker() {
    if [ -f /.dockerenv ]; then
        APOLLO_IN_DOCKER=true
        APOLLO_ROOT_DIR="/apollo"
    fi
}

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

function determine_gpu_use() {
    # If already inside container, use env USE_GPU directly
    if [[ "${APOLLO_IN_DOCKER}" == "true" ]] ; then
        return
    fi

local use_gpu=0
    # Check nvidia-driver and GPU device
    local nv_driver="nvidia-smi"
    if [ ! -x "$(command -v ${nv_driver} )" ]; then
        warning "No nvidia-driver found. CPU will be used."
    elif [ -z "$(eval ${nv_driver} )" ]; then
        warning "No GPU device found. CPU will be used."
    else
        use_gpu=1
    fi
    USE_GPU="${use_gpu}"
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
    check_if_in_docker
    APOLLO_CACHE_DIR="${APOLLO_CACHE_DIR:-${APOLLO_ROOT_DIR}/.cache}"
    source ${APOLLO_ROOT_DIR}/scripts/apollo.bashrc

    check_architecture_support
    check_platform_support
    check_minimal_memory_requirement
    determine_gpu_use
    check_apollo_version

    info "Apollo Environment Settings:"
    info "${TAB}APOLLO_ROOT_DIR: ${APOLLO_ROOT_DIR}"
    info "${TAB}APOLLO_CACHE_DIR: ${APOLLO_CACHE_DIR}"
    info "${TAB}APOLLO_IN_DOCKER: ${APOLLO_IN_DOCKER}"
    info "${TAB}APOLLO_VERSION: ${APOLLO_VERSION}"

    info "${TAB}USE_GPU=${USE_GPU}"
    info "${TAB}STAGE: ${STAGE}"
}

function run_cleanup() {
    if ! "${APOLLO_IN_DOCKER}" ; then
        error "The clean operation must be run from within docker container"
        exit 1
    fi
    bazel clean --async

    # Remove bazel cache in associated directories
    if [ -d /apollo-simulator ]; then
        pushd /apollo-simulator >/dev/null
            bazel clean --async
        popd >/dev/null
    fi
}

function _usage() {
    warning "Usage: Not implemented yet"
}


function main() {
    apollo_env_setup
    if [ "$#" -eq 0 ]; then
        exit 0
    fi
    local cmd="$1"; shift
    case "${cmd}" in
        doc)
            ${APOLLO_ROOT_DIR}/scripts/apollo_docs.sh "$@"
            ;;
        buildify)
            ${APOLLO_ROOT_DIR}/scripts/apollo_buildify.sh "$@"
            ;;
        clean)
            run_cleanup
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

