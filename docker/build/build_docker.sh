#! /usr/bin/env bash
TAB="    " # 4 Spaces

APOLLO_REPO="apolloauto/apollo"
UBUNTU_LTS="18.04"

SUPPORTED_ARCHS=(
    x86_64
    aarch64
)

SUPPORTED_GPUS=(
    amd
    nvidia
)

SUPPORTED_STAGES=(
    base
    cyber
    dev
    runtime
)

SUPPORTED_DIST=(
    stable
    testing
)

HOST_ARCH="$(uname -m)"
INSTALL_MODE="download"
TARGET_GEOLOC="us"

TARGET_DIST="stable"
TARGET_ARCH=
TARGET_GPU=
TARGET_STAGE=

DOCKERFILE=
PREV_IMAGE_TIMESTAMP=

USE_CACHE=1
DRY_RUN_ONLY=0

function check_experimental_docker() {
    local daemon_cfg="/etc/docker/daemon.json"
    local enabled="$(docker version -f '{{.Server.Experimental}}')"
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

function gpu_support_check() {
    local gpu="$1"
    for entry in "${SUPPORTED_GPUS[@]}"; do
        if [[ "${entry}" == "${gpu}" ]]; then
            return
        fi
    done
    echo "Unsupported GPU: ${gpu}. Exiting..."
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
    if [[ ${#__arr[@]} -ne 4 ]]; then
        echo "Expected dockerfile with name [prefix_]<target>.<arch>.<gpu>.dockerfile"
        echo "Got ${dockerfile}. Exiting..."
        exit 1
    fi
    IFS=' '

    local gpu="${__arr[2]}"
    gpu_support_check "${gpu}"
    TARGET_GPU="${gpu}"

    local arch="${__arr[1]}"
    cpu_arch_support_check "${arch}"
    TARGET_ARCH="${arch}"
    if [[ "${TARGET_ARCH}" != "${HOST_ARCH}" ]]; then
        echo "[WARNING] HOST_ARCH(${HOST_ARCH}) != TARGET_ARCH(${TARGET_ARCH}) detected."
        echo "[WARNING] Make sure you have executed the following command:"
        echo "[WARNING] ${TAB}docker run --rm --privileged multiarch/qemu-user-static --reset -p yes"
    fi

    local stage="${__arr[0]##*_}"
    build_stage_check "${stage}"
    TARGET_STAGE="${stage}"
}

CUDA_LITE=
CUDNN_VERSION=
TENSORRT_VERSION=
function determine_cuda_versions() {
    local arch="$1"
    local dist="$2"
    if [[ "${arch}" == "x86_64" ]]; then
        if [[ "${dist}" == "stable" ]]; then
            CUDA_LITE=11.1
            CUDNN_VERSION="8.0.4.30"
            TENSORRT_VERSION="7.2.1"
        else # testing
            CUDA_LITE=11.1
            CUDNN_VERSION="8.0.4.30"
            TENSORRT_VERSION="7.2.1"
        fi
    else # aarch64
        CUDA_LITE="10.2"
        CUDNN_VERSION="8.0.0.180"
        TENSORRT_VERSION="7.1.3"
    fi
}

function determine_prev_image_timestamp() {
    if [[ ! -z "${PREV_IMAGE_TIMESTAMP}" ]]; then
        return
    fi

    local arch="$1" # x86_64 / aarch64
    local stage="$2" # base/cyber/dev/runtime
    local dist="$3" # stable/testing

    local result=
    if [[ "${arch}" == "x86_64" ]]; then
        if [[ "${stage}" == "dev" ]]; then
            if [[ "${dist}" == "stable" ]]; then
                result="20210315_1535"
            else
                result="20210108_1510"
            fi
        elif [[ "${stage}" == "runtime" ]]; then
            if [[ "${dist}" == "stable" ]]; then
                result="20220803_1505"
            fi
        fi
    else # aarch64
        if [[ "${stage}" == "cyber" ]]; then
            if [[ "${dist}" == "stable" ]]; then
                result="20201217_0752"
            fi
        elif [[ "${stage}" == "dev" ]]; then
            if [[ "${dist}" == "stable" ]]; then
                result="20201217_1302"
            fi
        fi
    fi
    PREV_IMAGE_TIMESTAMP="${result}"
}

IMAGE_IN=
IMAGE_OUT=
DEV_IMAGE_IN=

function determine_images_in_out_x86_64() {
    local stage="$1" # Build stage, base/cyber/dev
    local dist="$2"  # stable or testing
    local timestamp="$3" # Timestamp

    local cudnn_ver="${CUDNN_VERSION%%.*}"
    local trt_ver="${TENSORRT_VERSION%%.*}"
    local arch
    if [[ "${TARGET_GPU}" == "amd" ]]; then
        arch="x86_64-amd"
    elif [[ "${TARGET_GPU}" == "nvidia" ]]; then
        arch="x86_64-nvidia"
    fi

    local base_nvidia_image="${APOLLO_REPO}:cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}-${arch}"
    local base_amd_image="rocm/dev-ubuntu-18.04:5.1.1-complete"
    if [[ "${stage}" == "base" ]]; then
        if [[ "${TARGET_GPU}" == "nvidia" ]]; then
            IMAGE_IN="nvidia/cuda:${CUDA_LITE}-devel-ubuntu${UBUNTU_LTS}"
            IMAGE_OUT="${base_nvidia_image}"
        else
            echo "The base Docker for AMD GPU is not ready. Exiting..."
            exit 1
        fi
    elif [[ "${stage}" == "cyber" ]]; then
        if [[ "${TARGET_GPU}" == "nvidia" ]]; then
            IMAGE_IN="${base_nvidia_image}"
        elif [[ "${TARGET_GPU}" == "amd" ]]; then
            IMAGE_IN="${base_amd_image}"
        fi

        if [[ "${dist}" == "stable" ]]; then
            IMAGE_OUT="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
        else
            IMAGE_OUT="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-testing-${timestamp}"
        fi
    elif [[ "${stage}" == "dev" ]]; then
        if [[ "${dist}" == "stable" ]]; then
            IMAGE_IN="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${PREV_IMAGE_TIMESTAMP}"
            IMAGE_OUT="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
        else
            IMAGE_IN="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-testing-${PREV_IMAGE_TIMESTAMP}"
            IMAGE_OUT="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-testing-${timestamp}"
        fi
    elif [[ "${stage}" == "runtime" ]]; then
        if [[ "${dist}" == "stable" ]]; then
            if [[ "${TARGET_GPU}" == "amd" ]]; then
                echo "AMD is not supported for runtime Docker. Exiting..."
                exit 1
            fi
            IMAGE_IN="nvidia/cuda:${CUDA_LITE}-runtime-ubuntu${UBUNTU_LTS}"
            DEV_IMAGE_IN="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${PREV_IMAGE_TIMESTAMP}"
            IMAGE_OUT="${APOLLO_REPO}:runtime-${arch}-${UBUNTU_LTS}-${timestamp}"
        else
            echo "Runtime Docker for Apollo Testing not ready. Exiting..."
            exit 1
        fi
    else
        echo "Unknown build stage: ${stage}. Exiting..."
        exit 1
    fi
}

function determine_images_in_out_aarch64() {
    local stage="$1"
    local timestamp="$2"

    local cudnn_ver="${CUDNN_VERSION%%.*}"
    local trt_ver="${TENSORRT_VERSION%%.*}"

    local BASE_FMT="l4t-cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}"
    local CYBER_FMT="cyber-aarch64-nvidia-${UBUNTU_LTS}"

    if [[ "${TARGET_GPU}" == "amd" ]]; then
        echo "ARM64 architecture is not supported for AMD GPU target. Exiting..."
        exit 1
    fi

    if [[ "${stage}" == "base" ]]; then
        # Ref: https://developer.nvidia.com/embedded/linux-tegra
        IMAGE_IN="nvcr.io/nvidia/l4t-base:r32.4.4"
        IMAGE_OUT="${APOLLO_REPO}:${BASE_FMT}-${timestamp}"
    elif [[ "${stage}" == "cyber" ]]; then
        IMAGE_IN="${APOLLO_REPO}:${BASE_FMT}-${PREV_IMAGE_TIMESTAMP}"
        IMAGE_OUT="${APOLLO_REPO}:${CYBER_FMT}-${timestamp}"
    elif [[ "${stage}" == "dev" ]]; then
        IMAGE_IN="${APOLLO_REPO}:${CYBER_FMT}-${PREV_IMAGE_TIMESTAMP}"
        IMAGE_OUT="${APOLLO_REPO}:dev-aarch64-nvidia-${UBUNTU_LTS}-${timestamp}"
    elif [[ "${stage}" == "runtime" ]]; then
        echo "Runtime Docker for AArch64 not ready yet. Exiting..."
        exit 1
    else
        echo "Unknown build stage: ${stage}. Exiting..."
        exit 1
    fi
}

function determine_images_in_out() {
    local arch="$1"
    local stage="$2"
    local dist="$3"
    local timestamp="$(date +%Y%m%d_%H%M)"

    determine_cuda_versions "${arch}" "${dist}"
    determine_prev_image_timestamp "${arch}" "${stage}" "${dist}"

    if [[ "${arch}" == "x86_64" ]]; then
        determine_images_in_out_x86_64 "${stage}" "${dist}" "${timestamp}"
    else
        # Only stable images for Aarch64
        determine_images_in_out_aarch64 "${stage}" "${timestamp}"
    fi
}

function print_usage() {
    local prog="$(basename $0)"
    echo "Usage:"
    echo "${TAB}${prog} -f <Dockerfile> [Options]"
    echo "Available options:"
    echo "${TAB}-c,--clean      Use \"--no-cache=true\" for docker build"
    echo "${TAB}-m,--mode       \"build\" for build everything from source if possible, \"download\" for using prebuilt ones"
    echo "${TAB}-g,--geo        Enable geo-specific mirrors to speed up build. Currently \"cn\" and \"us\" are supported."
    echo "${TAB}-d,--dist       Whether to build stable(\"stable\") or experimental(\"testing\") Docker images"
    echo "${TAB}-t,--timestamp  Specify image timestamp for previous stage to build image upon. Format: yyyymmdd_hhmm (e.g 20210205_1520)"
    echo "${TAB}--dry           Dry run (for testing purpose)"
    echo "${TAB}-h,--help       Show this message and exit"
    echo "E.g.,"
    echo "${TAB}${prog} -f cyber.x86_64.dockerfile -m build -g cn"
    echo "${TAB}${prog} -f dev.aarch64.dockerfile -m download -d testing"
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
                INSTALL_MODE="$1"
                shift
                ;;
            -g|--geo)
                check_opt_arg "${opt}" "$1"
                TARGET_GEOLOC="$1"
                shift
                ;;
            -c|--clean)
                USE_CACHE=0
                ;;
            -d|--dist)
                check_opt_arg "${opt}" "$1"
                TARGET_DIST="$1"
                shift
                ;;
            -t|--timestamp)
                check_opt_arg "${opt}" "$1"
                PREV_IMAGE_TIMESTAMP="$1"
                shift
                ;;
            --dry)
                DRY_RUN_ONLY=1
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
    if [[ "${INSTALL_MODE}" == "download" ]]; then
        echo "Use prebuilt packages for dependencies if possible"
    elif [[ "${INSTALL_MODE}" == "build" ]]; then
        echo "Build all packages from source"
    else
        echo "Unknown INSTALL_MODE: ${INSTALL_MODE}."
        exit 1
    fi

    if [[ "${TARGET_GEOLOC}" == "cn" ]]; then
        echo "Docker image built w/ CN based mirrors."
    elif [[ "${TARGET_GEOLOC}" != "us" ]]; then
        echo "Unknown GEOLOC: ${TARGET_GEOLOC}, defaults to 'us'"
        TARGET_GEOLOC="us"
    fi

    if [[ "${TARGET_DIST}" == "stable" ]]; then
        echo "Stable Docker image will be built."
    elif [[ "${TARGET_DIST}" == "testing" ]]; then
        echo "Testing (experimental) Docker image will be built."
    else
        echo "Unknown APOLLO_DIST: ${TARGET_DIST}. Exit..."
        exit 1
    fi

    if [[ -z "${DOCKERFILE}" ]]; then
        echo "Dockfile not specified. Exiting..."
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
    echo "|  INSTALL_MODE=${INSTALL_MODE}, GEOLOC=${TARGET_GEOLOC}, APOLLO_DIST=${TARGET_DIST}"
    echo "=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.=====.====="
}

function docker_build_run() {
    local extra_args="--squash"
    if [[ "${USE_CACHE}" -eq 0 ]]; then
        extra_args="${extra_args} --no-cache=true"
    fi
    local context="$(dirname "${BASH_SOURCE[0]}")"

    local build_args="--build-arg BASE_IMAGE=${IMAGE_IN}"

    if [[ "${TARGET_STAGE}" == "base" ]]; then
        build_args="${build_args} --build-arg CUDA_LITE=${CUDA_LITE}"
        build_args="${build_args} --build-arg CUDNN_VERSION=${CUDNN_VERSION}"
        build_args="${build_args} --build-arg TENSORRT_VERSION=${TENSORRT_VERSION}"
    elif [[ "${TARGET_STAGE}" == "cyber" || "${TARGET_STAGE}" == "dev" ]]; then
        build_args="${build_args} --build-arg APOLLO_DIST=${TARGET_DIST}"
        build_args="${build_args} --build-arg GEOLOC=${TARGET_GEOLOC}"
        build_args="${build_args} --build-arg CLEAN_DEPS=yes"
        build_args="${build_args} --build-arg INSTALL_MODE=${INSTALL_MODE}"
    elif [[ "${TARGET_STAGE}" == "runtime" ]]; then
        build_args="${build_args} --build-arg GEOLOC=${TARGET_GEOLOC}"
        build_args="${build_args} --build-arg CUDA_LITE=${CUDA_LITE}"
        build_args="${build_args} --build-arg CUDNN_VERSION=${CUDNN_VERSION}"
        build_args="${build_args} --build-arg TENSORRT_VERSION=${TENSORRT_VERSION}"
        build_args="${build_args} --build-arg DEV_IMAGE_IN=${DEV_IMAGE_IN}"
    else
        echo "Unknown build stage: ${TARGET_STAGE}. Exiting..."
        exit 1
    fi

    set -x
    docker build --network=host ${extra_args} -t "${IMAGE_OUT}" \
            ${build_args} \
            -f "${DOCKERFILE}" \
            "${context}"
    set +x
}

function main() {
    parse_arguments "$@"
    check_arguments

    check_experimental_docker
    determine_images_in_out "${TARGET_ARCH}" "${TARGET_STAGE}" "${TARGET_DIST}"
    docker_build_preview

    if [[ "${DRY_RUN_ONLY}" -gt 0 ]]; then
        return
    fi

    docker_build_run
}

main "$@"
