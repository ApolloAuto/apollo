#! /usr/bin/env bash

APOLLO_REPO="apolloauto/apollo"

SUPPORTED_ARCHS=(
    x86_64
    aarch64
)

HOST_ARCH="$(uname -m)"
TARGET_ARCH=

SUPPORTED_STAGES=(
    base
    cyber
    dev
)

SUPPORTED_DIST=(
    stable
    testing
)

DOCKERFILE=
TARGET_STAGE=
IMAGE_IN=
IMAGE_OUT=

GEOLOC="us"
MODE="download"
DIST="stable"
BUILD_CLEAN=0
DRY_RUN=0

CUDA_LITE=""
CUDNN_VERSION=""
TENSORRT_VERSION=""
UBUNTU_LTS="18.04"

TIMESTAMP="$(date +%Y%m%d_%H%M)"

TAB="    "

function check_experimental_docker() {
    local jq_cmd="$(command -v jq)"
    if [ -z "${jq_cmd}" ]; then
        echo "Oops, command 'jq' not found."
        echo "For Ubuntu, you can install it via:"
        echo "  sudo apt-get -y update && sudo apt-get -y install jq"
        exit 1
    fi
    local daemon_cfg="/etc/docker/daemon.json"
    local enabled="$(jq '.experimental' ${daemon_cfg} )"
    if [ "${enabled}" != "true" ]; then
        echo "Experimental features should be enabled to run Apollo docker build."
        echo "Please perform the following two steps to have it enabled:"
        echo "  1) Add '\"experimental\": true' to '${daemon_cfg}'"
        echo "     The simplest ${daemon_cfg} looks like:"
        echo "       {"
        echo "           \"experimental\": true "
        echo "       }"
        echo "  2) Restart docker daemon. E.g., 'sudo systemctl restart docker'"
        exit 1
    fi
}

function cpu_arch_support_check() {
    local arch="$1"
    for entry in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${entry}" == "${arch}" ]]; then
            return
        fi
    done
    echo "Unsupported CPU architecture: ${arch}. Exiting..."
    exit 1
}

function build_stage_check() {
    local stage="$1"
    for entry in "${SUPPORTED_STAGES[@]}"; do
        if [[ "${entry}" == "${stage}" ]]; then
            return
        fi
    done
    echo "Unsupported build stage: ${stage}. Exiting..."
    exit 1
}

function determine_target_arch_and_stage() {
    local dockerfile="$(basename "$1")"
    IFS='.' read -ra __arr <<< "${dockerfile}"
    if [[ ${#__arr[@]} -ne 3 ]]; then
        echo "Expected dockerfile with name [prefix_]<target>.<arch>.dockerfile"
        echo "Got ${dockerfile}. Exiting..."
        exit 1
    fi
    IFS=' '

    local arch="${__arr[1]}"
    cpu_arch_support_check "${arch}"
    TARGET_ARCH="${arch}"
    if [[ "${TARGET_ARCH}" != "${HOST_ARCH}" ]]; then
        echo "[WARNING] HOST_ARCH(${HOST_ARCH}) != TARGET_ARCH(${TARGET_ARCH})"
        echo "[WARNING] Make sure you have executed the following command:"
        echo "[WARNING] ${TAB}docker run --rm --privileged multiarch/qemu-user-static --reset -p yes"
    fi

    local stage="${__arr[0]##*_}"
    build_stage_check "${stage}"
    TARGET_STAGE="${stage}"
}

function determine_images_for_x86_64() {
    local stage="$1" # Build stage, base/cyber/dev
    local dist="$2"  # stable or testing

    if [[ "${dist}" == "stable" ]]; then
        CUDA_LITE=10.2
        CUDNN_VERSION="7.6.5.32"
        TENSORRT_VERSION="7.0.0"
    else
        CUDA_LITE=11.1
        CUDNN_VERSION="8.0.4.30"
        TENSORRT_VERSION="7.2.1"
    fi
    local cudnn_ver="${CUDNN_VERSION%%.*}"
    local trt_ver="${TENSORRT_VERSION%%.*}"

    local base_image="${APOLLO_REPO}:cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}-x86_64"
    if [[ "${stage}" == "base" ]]; then
        IMAGE_IN="nvidia/cuda:${CUDA_LITE}-devel-ubuntu${UBUNTU_LTS}"
        IMAGE_OUT="${base_image}"
    elif [[ "${stage}" == "cyber" ]]; then
        IMAGE_IN="${base_image}"
        if [[ "${dist}" == "stable" ]]; then
            IMAGE_OUT="${APOLLO_REPO}:cyber-x86_64-${UBUNTU_LTS}-${TIMESTAMP}"
        else
            IMAGE_OUT="${APOLLO_REPO}:cyber-x86_64-${UBUNTU_LTS}-testing-${TIMESTAMP}"
        fi
    elif [[ "${stage}" == "dev" ]]; then
        if [[ "${dist}" == "stable" ]]; then
            IMAGE_IN="${APOLLO_REPO}:cyber-x86_64-${UBUNTU_LTS}-20201210_0223"
            IMAGE_OUT="${APOLLO_REPO}:dev-x86_64-${UBUNTU_LTS}-${TIMESTAMP}"
        else
            IMAGE_IN="${APOLLO_REPO}:cyber-x86_64-${UBUNTU_LTS}-testing-20201208_2246"
            IMAGE_OUT="${APOLLO_REPO}:dev-x86_64-${UBUNTU_LTS}-testing-${TIMESTAMP}"
        fi
    else
        echo "Unknown build stage: ${stage}. Exiting..."
        exit 1
    fi
}

function determine_images_for_aarch64() {
    local stage="$1"
    CUDA_LITE="10.2"
    CUDNN_VERSION="8.0.0.180"
    TENSORRT_VERSION="7.1.3"
    local cudnn_ver="${CUDNN_VERSION%%.*}"
    local trt_ver="${TENSORRT_VERSION%%.*}"

    local BASE_FMT="l4t-cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}"
    local CYBER_FMT="cyber-aarch64-${UBUNTU_LTS}"

    if [[ "${stage}" == "base" ]]; then
        # Ref: https://developer.nvidia.com/embedded/linux-tegra
        IMAGE_IN="nvcr.io/nvidia/l4t-base:r32.4.4"
        IMAGE_OUT="${APOLLO_REPO}:${BASE_FMT}-${TIMESTAMP}"
    elif [[ "${stage}" == "cyber" ]]; then
        IMAGE_IN="${APOLLO_REPO}:${BASE_FMT}-20201217_0752"
        IMAGE_OUT="${APOLLO_REPO}:${CYBER_FMT}-${TIMESTAMP}"
    elif [[ "${stage}" == "dev" ]]; then
        IMAGE_IN="${APOLLO_REPO}:${CYBER_FMT}-20201217_1302"
        IMAGE_OUT="${APOLLO_REPO}:dev-aarch64-${UBUNTU_LTS}-${TIMESTAMP}"
    else
        echo "Unknown build stage: ${stage}. Exiting..."
        exit 1
    fi
}

function print_usage() {
    local prog="$(basename $0)"
    echo "Usage:"
    echo "${TAB}${prog} -f <Dockerfile> [Options]"
    echo "Available options:"
    echo "${TAB}-c,--clean  Use \"--no-cache=true\" for docker build"
    echo "${TAB}-m,--mode   \"build\" for build everything from source if possible, \"download\" for using prebuilt ones"
    echo "${TAB}-g,--geo    Enable geo-specific mirrors to speed up build. Currently \"cn\" and \"us\" are supported."
    echo "${TAB}-d,--dist   Whether to build stable(\"stable\") or experimental(\"testing\") Docker images"
    echo "${TAB}--dry       Dry run (for testing purpose)"
    echo "${TAB}-h,--help   Show this message and exit"
    echo "E.g.,"
    echo "${TAB}${prog} -f cyber.x86_64.dockerfile -m build -g cn"
    echo "${TAB}${prog} -f dev.aarch64.dockerfile -m download -b testing"
}

function check_opt_arg() {
    local opt="$1"
    local arg="$2"
    if [[ -z "${arg}" || "${arg}" =~ ^-.* ]]; then
        echo "Argument missing for option ${opt}. Exiting..."
        exit 1
    fi
}

function parse_arguments() {
    if [[ $# -eq 0 ]] || [[ "$1" == "--help" ]]; then
        print_usage
        exit 0
    fi
    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case $opt in
            -f|--dockerfile)
                check_opt_arg "${opt}" "$1"
                DOCKERFILE="$1"
                shift
                ;;
            -m|--mode)
                check_opt_arg "${opt}" "$1"
                MODE="$1"
                shift
                ;;
            -g|--geo)
                check_opt_arg "${opt}" "$1"
                GEOLOC="$1"
                shift
                ;;
            -c|--clean)
                BUILD_CLEAN=1
                ;;
            -d|--dist)
                check_opt_arg "${opt}" "$1"
                DIST="$1"
                shift
                ;;
            --dry)
                DRY_RUN=1
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            *)
                echo "Unknown option: ${opt}"
                print_usage
                exit 1
                ;;
        esac
    done
}

function check_arguments() {
    if [[ "${MODE}" == "build" ]]; then
        echo "Build all packages from source if possible"
    elif [[ "${MODE}" != "download" ]]; then
        echo "Unknown build mode: ${MODE}, defaults to [download]"
        # Use prebuilt packages for dependencies if possible
        MODE="download"
    fi

    if [[ "${GEOLOC}" == "cn" ]]; then
        echo "Docker image built for CN users"
    elif [[ "${GEOLOC}" != "us" ]]; then
        echo "Unknown geo: ${GEOLOC}, defaults to [us]"
        GEOLOC="us"
    fi

    if [[ "${DIST}" == "testing" ]]; then
        echo "Build experimental Docker images."
    elif [[ "${DIST}" != "stable" ]]; then
        echo "Build stable Docker images"
        DIST="stable"
    fi

    if [[ -z "${DOCKERFILE}" ]]; then
        echo "Dockfile not specified, Exiting..."
        exit 1
    fi
    determine_target_arch_and_stage "${DOCKERFILE}"
}

function docker_build_preview() {
    echo "=====.=====.===== Docker Build Preview for ${TARGET_STAGE} =====.=====.====="
    echo "|  Generated image: ${IMAGE_OUT}"
    echo "|  FROM image: ${IMAGE_IN}"
    echo "|  Dockerfile: ${DOCKERFILE}"
    echo "|  TARGET_ARCH=${TARGET_ARCH}, HOST_ARCH=${HOST_ARCH}"
    echo "|  INSTALL_MODE=${MODE}, GEOLOC=${GEOLOC}, APOLLO_DIST=${DIST}"
    echo "=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.====="
}

function docker_build_run() {
    local extra_args="--squash"
    if [[ "${BUILD_CLEAN}" -gt 0 ]]; then
        extra_args="${extra_args} --no-cache=true"
    fi
    local context="$(dirname "${BASH_SOURCE[0]}")"

    set -x
    docker build ${extra_args} -t "${IMAGE_OUT}" \
        --build-arg INSTALL_MODE="${MODE}" \
        --build-arg GEOLOC="${GEOLOC}"     \
        --build-arg APOLLO_DIST="${DIST}"  \
        --build-arg BASE_IMAGE="${IMAGE_IN}" \
        --build-arg CUDA_LITE="${CUDA_LITE}" \
        --build-arg CUDNN_VERSION="${CUDNN_VERSION}" \
        --build-arg TENSORRT_VERSION="${TENSORRT_VERSION}" \
        --build-arg CLEAN_DEPS="yes" \
        -f "${DOCKERFILE}" \
        "${context}"
    set +x
}

function main() {
    parse_arguments "$@"
    check_arguments
    check_experimental_docker
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        determine_images_for_x86_64 "${TARGET_STAGE}" "${DIST}"
    elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
        determine_images_for_aarch64 "${TARGET_STAGE}" "${DIST}"
    fi
    docker_build_preview
    if [[ "${DRY_RUN}" -eq 0 ]]; then
        docker_build_run
    fi
}

main "$@"

