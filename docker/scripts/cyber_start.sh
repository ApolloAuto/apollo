#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "${APOLLO_ROOT_DIR}/scripts/apollo.bashrc"

# CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

VERSION_X86_64="cyber-x86_64-18.04-20201210_0223"
TESTING_VERSION_X86_64="cyber-x86_64-18.04-testing-20201208_2246"

#L4T
VERSION_AARCH64="cyber-aarch64-18.04-20201217_1302"

CYBER_CONTAINER="apollo_cyber_${USER}"
CYBER_INSIDE="in-cyber-docker"

DOCKER_REPO="apolloauto/apollo"
DOCKER_RUN_CMD="docker run"
DOCKER_PULL_CMD="docker pull"
SHM_SIZE="2G"

SUPPORTED_ARCHS=(x86_64 aarch64)
HOST_ARCH="$(uname -m)"
HOST_OS="$(uname -s)"
TARGET_ARCH=""

USE_GPU_HOST=0
USE_LOCAL_IMAGE=0
CUSTOM_DIST=
CUSTOM_VERSION=
GEOLOC=
GEO_REGISTRY=

# Check whether user has agreed license agreement
function check_agreement() {
    local agreement_record="${HOME}/.apollo_agreement.txt"
    if [[ -e "${agreement_record}" ]]; then
        return 0
    fi
    local agreement_file
    agreement_file="$APOLLO_ROOT_DIR/scripts/AGREEMENT.txt"
    if [[ ! -f "${agreement_file}" ]]; then
        error "AGREEMENT ${agreement_file} does not exist."
        exit 1
    fi

    cat "${agreement_file}"
    local tip="Type 'y' or 'Y' to agree to the license agreement above, \
or type any other key to exit:"
    echo -n "${tip}"
    read -r -n 1 user_agreed
    echo
    if [[ "${user_agreed}" == "y" || "${user_agreed}" == "Y" ]]; then
        cp -f "${agreement_file}" "${agreement_record}"
        echo
        echo "${tip}" >>"${agreement_record}"
        echo "${user_agreed}" >>"${agreement_record}"
    else
        exit 1
    fi
}

function check_host_environment() {
    echo "Done checking host environment."
}

function _optarg_check_for_opt() {
    local opt="$1"
    local optarg="$2"

    if [[ -z "${optarg}" || "${optarg}" =~ ^-.* ]]; then
        error "Missing parameter for ${opt}. Exiting..."
        exit 2
    fi
}

function _target_arch_check() {
    local arch="$1"
    for k in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${k}" == "${arch}" ]]; then
            return
        fi
    done
    error "Unsupported target architecture: ${arch}."
    exit 1
}

function show_usage() {
    cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -g <us|cn>             Pull docker image from mirror registry based on geolocation.
    -h, --help             Display this help and exit.
    -t, --tag <TAG>        Specify docker image with tag to start
    -d, --dist             Specify Apollo distribution(stable/testing)
    -l, --local            Use local docker image.
    -m <arch>              Specify docker image for a different CPU arch.
    --shm-size <bytes>     Size of /dev/shm . Passed directly to "docker run"
    stop [-f|--force]      Stop all running Apollo containers. Use "-f" to force removal.
EOF
}

function stop_all_apollo_containers_for_user() {
    local force="$1"
    local running_containers
    running_containers="$(docker ps -a --format '{{.Names}}')"
    for container in ${running_containers[*]}; do
        if [[ "${container}" =~ apollo_.*_${USER} ]]; then
            #printf %-*s 70 "Now stop container: ${container} ..."
            #printf "\033[32m[DONE]\033[0m\n"
            #printf "\033[31m[FAILED]\033[0m\n"
            info "Now stop container ${container} ..."
            if docker stop "${container}" >/dev/null; then
                if [[ "${force}" == "-f" || "${force}" == "--force" ]]; then
                    docker rm -f "${container}" >/dev/null
                fi
                info "Done."
            else
                warning "Failed."
            fi
        fi
    done
    if [[ "${force}" == "-f" || "${force}" == "--force" ]]; then
        info "OK. Done stop and removal"
    else
        info "OK. Done stop."
    fi
}

function parse_arguments() {
    local use_local_image=0
    local custom_version=""
    local custom_dist=""
    local target_arch=""
    local shm_size=""
    local geo=""

    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case "${opt}" in
            -g | --geo)
                geo="$1"
                shift
                _optarg_check_for_opt "${opt}" "${geo}"
                ;;
            -t | --tag)
                if [[ ! -z "${custom_version}" ]]; then
                    warning "Multiple option ${opt} specified, only the last one will take effect."
                fi
                custom_version="$1"
                shift
                _optarg_check_for_opt "${opt}" "${custom_version}"
                ;;
            -d | --dist)
                custom_dist="$1"
                shift
                _optarg_check_for_opt "${opt}" "${custom_dist}"
                ;;
            -h | --help)
                show_usage
                exit 1
                ;;
            -l | --local)
                use_local_image=1
                ;;
            -m)
                target_arch="$1"
                shift
                _optarg_check_for_opt "${opt}" "${target_arch}"
                _target_arch_check "${target_arch}"
                ;;
            --shm-size)
                shm_size="$1"
                shift
                _optarg_check_for_opt "${opt}" "${shm_size}"
                ;;
            stop)
                local force="$1"
                shift
                info "Now, stop all apollo containers created by ${USER} ..."
                stop_all_apollo_containers_for_user "${force}"
                exit 0
                ;;
            *)
                warning "Unknown option: $1"
                exit 2
                ;;
        esac
    done # End while loop

    [[ ! -z "${geo}" ]] && GEOLOC="${geo}"
    USE_LOCAL_IMAGE="${use_local_image}"
    [[ -n "${target_arch}" ]] && TARGET_ARCH="${target_arch}"
    [[ -n "${custom_version}" ]] && CUSTOM_VERSION="${custom_version}"
    [[ -n "${custom_dist}" ]] && CUSTOM_DIST="${custom_dist}"
    [[ -n "${shm_size}" ]] && SHM_SIZE="${shm_size}"
}

# if [ ! -e /apollo ]; then
#    sudo ln -sf "${APOLLO_ROOT_DIR}" /apollo
# fi
# if [ -e /proc/sys/kernel ]; then
#    echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern > /dev/null
# fi
# <cyber|dev>-<arch>-<ubuntu-release>-<timestamp>
function guess_arch_from_tag() {
    local tag="$1"
    local __result="$2"

    local arch
    IFS='-' read -ra __arr <<<"${tag}"
    IFS=' ' # restore
    if [[ ${#__arr[@]} -lt 3 ]]; then
        warning "Unexpected image: ${tag}"
        arch=""
    else
        arch="${__arr[1]}"
    fi
    eval "${__result}='${arch}'"
}

function determine_target_version_and_arch() {
    local version="$1"
    # If no custom version specified
    if [[ -z "${version}" ]]; then
        # if target arch not set, assume it is equal to host arch.
        if [[ -z "${TARGET_ARCH}" ]]; then
            TARGET_ARCH="${HOST_ARCH}"
        fi
        _target_arch_check "${TARGET_ARCH}"
        if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
            if [[ "${CUSTOM_DIST}" == "testing" ]]; then
                version="${TESTING_VERSION_X86_64}"
            else
                version="${VERSION_X86_64}"
            fi
        elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
            version="${VERSION_AARCH64}"
        else
            error "CAN'T REACH HERE"
            exit 1
        fi
    else # CUSTOM_VERSION specified
        local supposed_arch
        guess_arch_from_tag "${version}" supposed_arch
        if [[ -z "${supposed_arch}" ]]; then
            error "Can't guess target arch from image tag: ${version}"
            error "  Expected format <target>-<arch>-<ubuntu_release>-<timestamp>"
            exit 1
        fi
        if [[ -z "${TARGET_ARCH}" ]]; then
            _target_arch_check "${supposed_arch}"
            TARGET_ARCH="${supposed_arch}"
        elif [[ "${TARGET_ARCH}" != "${supposed_arch}" ]]; then
            error "Target arch (${TARGET_ARCH}) doesn't match supposed arch" \
                "(${supposed_arch}) from ${version}"
            exit 1
        fi
    fi
    CUSTOM_VERSION="${version}"
}

function geo_specific_config() {
    local geo="$1"
    if [[ -z "${geo}" ]]; then
        return
    fi
    info "Setup geolocation specific configurations for ${geo}"
    if [[ "${geo}" == "cn" ]]; then
        GEO_REGISTRY="registry.baidubce.com"
    fi
}

function determine_gpu_use_aarch64() {
    local use_gpu=0
    if lsmod | grep -q nvgpu; then
        use_gpu=1
    fi
    if [[ "${use_gpu}" -eq 1 ]]; then
        local docker_version
        docker_version="$(docker version --format '{{.Server.Version}}')"
        if dpkg --compare-versions "${docker_version}" "ge" "19.03"; then
            DOCKER_RUN_CMD="docker run --gpus all"
        else
            warning "You must upgrade to Docker-CE 19.03+ to access GPU from container!"
            use_gpu=0
        fi
    fi
    USE_GPU_HOST="${use_gpu}"
}

function determine_gpu_use_amd64() {
    # Check nvidia-driver and GPU device
    local nv_driver="nvidia-smi"
    if [ ! -x "$(command -v ${nv_driver})" ]; then
        warning "No nvidia-driver found. CPU will be used"
    elif [ -z "$(eval ${nv_driver})" ]; then
        warning "No GPU device found. CPU will be used."
    else
        USE_GPU_HOST=1
    fi

    # Try to use GPU inside container
    local nv_docker_doc="https://github.com/NVIDIA/nvidia-docker/blob/master/README.md"
    if [[ "${USE_GPU_HOST}" -eq 1 ]]; then
        if [[ -x "$(which nvidia-container-toolkit)" ]]; then
            local docker_version
            docker_version="$(docker version --format '{{.Server.Version}}')"
            if dpkg --compare-versions "${docker_version}" "ge" "19.03"; then
                DOCKER_RUN_CMD="docker run --gpus all"
            else
                warning "You must upgrade to docker-ce 19.03+ to access GPU from container!"
                USE_GPU_HOST=0
            fi
        elif [[ -x "$(which nvidia-docker)" ]]; then
            DOCKER_RUN_CMD="nvidia-docker run"
        else
            USE_GPU_HOST=0
            warning "Cannot access GPU from within container. Please install latest Docker" \
                "and NVIDIA Container Toolkit as described by: "
            warning "  ${nv_docker_doc}"
        fi
    fi
}

function determine_gpu_use_host() {
    if [[ "${HOST_ARCH}" == "x86_64" ]]; then
        determine_gpu_use_amd64
    else
        determine_gpu_use_aarch64
    fi

}
function setup_devices_and_mount_volumes() {
    local __retval="$1"

    if [[ "${HOST_ARCH}" == "${TARGET_ARCH}" ]]; then
        source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh" CYBER_ONLY
        setup_device
    fi

    local volumes
    volumes="-v ${APOLLO_ROOT_DIR}:/apollo"

    if [[ "${HOST_OS}" != "Linux" ]]; then
        warning "Running Cyber container on ${HOST_OS} is experimental!"
    else
        local os_release="$(lsb_release -rs)"
        case "${os_release}" in
            16.04)
                # Mount host devices into container (/dev)
                warning "[Deprecated] Support for Ubuntu 16.04 will be removed" \
                    "in the near future. Please upgrade to ubuntu 18.04+."
                if [[ "${HOST_ARCH}" == "${TARGET_ARCH}" ]]; then
                    volumes="${volumes} -v /dev:/dev"
                fi
                ;;
            18.04 | 20.04 | *)
                if [[ "${HOST_ARCH}" == "${TARGET_ARCH}" ]]; then
                    volumes="${volumes} -v /dev:/dev"
                fi
                ;;
        esac
    fi

    volumes="${volumes} -v /etc/localtime:/etc/localtime:ro"
    # volumes="${volumes} -v /usr/src:/usr/src"
    if [[ "${kernel}" == "Linux" ]]; then
        if [[ "${HOST_ARCH}" == "${TARGET_ARCH}" ]]; then
            volumes="${volumes} -v /dev/null:/dev/raw1394 \
                            -v /media:/media \
                            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                            -v /lib/modules:/lib/modules \
                    "
        fi
    fi
    volumes="$(tr -s " " <<<"${volumes}")"
    eval "${__retval}='${volumes}'"
}

function remove_existing_cyber_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "${CYBER_CONTAINER}"; then
        info "Removing existing cyber container ${CYBER_CONTAINER}"
        docker stop "${CYBER_CONTAINER}" >/dev/null
        docker rm -v -f "${CYBER_CONTAINER}" >/dev/null
    fi
}

function docker_pull_if_needed() {
    local image="$1"
    local use_local_image="$2"
    if [[ ${use_local_image} -eq 1 ]]; then
        info "Start cyber container based on local image: ${image}"
    elif docker images -a --format '{{.Repository}}:{{.Tag}}' |
        grep -q "${image}"; then
        info "Image ${image} found locally, will be used."
        use_local_image=1
    fi
    if [[ ${use_local_image} -eq 1 ]]; then
        return
    fi

    image="${DOCKER_REPO}:${image##*:}"
    echo "Start pulling docker image: ${image}"
    if [[ -n "${GEO_REGISTRY}" ]]; then
        image="${GEO_REGISTRY}/${image}"
    fi
    if ! ${DOCKER_PULL_CMD} "${image}"; then
        error "Failed to pull docker image: ${image}"
        exit 1
    fi
}

function check_multi_arch_support() {
    if [[ "${TARGET_ARCH}" == "${HOST_ARCH}" ]]; then
        return
    fi
    info "Cyber ${TARGET_ARCH} container running on ${HOST_ARCH} host."

    # Note(storypku):
    # ubuntu 18.04: apt-get -y install qemu-user-static
    # with sudo-no-suid problem

    local qemu="multiarch/qemu-user-static"
    local refer="https://github.com/multiarch/qemu-user-static"
    local qemu_cmd="docker run --rm --privileged ${qemu} --reset -p yes"
    if docker images --format "{{.Repository}}" | grep -q "${qemu}"; then
        info "Run: ${qemu_cmd}"
        ## docker run --rm --privileged "${qemu}" --reset -p yes >/dev/null
        eval "${qemu_cmd}" >/dev/null
        info "Multiarch support has been enabled. Ref: ${refer}"
    else
        warning "Multiarch support hasn't been enabled. Please make sure the"
        warning "following command has been executed before continuing:"
        warning "  ${qemu_cmd}"
        warning "Refer to ${refer} for more."
        exit 1
    fi
}

function start_cyber_container() {
    local image="$1"
    # if [ ! -d "${CACHE_ROOT_DIR}" ]; then
    #    mkdir "${CACHE_ROOT_DIR}"
    # fi
    info "Starting docker container \"${CYBER_CONTAINER}\" ..."

    local user="${USER}"
    local uid="$(id -u)"

    local group=$(id -g -n)
    local gid=$(id -g)
    local local_host="$(hostname)"

    local local_volumes
    setup_devices_and_mount_volumes local_volumes

    local display="${DISPLAY:-:0}"

    set -x
    ${DOCKER_RUN_CMD} -it \
        -d \
        --privileged \
        --name "${CYBER_CONTAINER}" \
        -e DISPLAY="${display}" \
        -e DOCKER_USER="${user}" \
        -e USER="${user}" \
        -e DOCKER_USER_ID="${uid}" \
        -e DOCKER_GRP="${group}" \
        -e DOCKER_GRP_ID="${gid}" \
        -e DOCKER_IMG="${image}" \
        -e HOST_OS="${HOST_OS}" \
        -e USE_GPU_HOST="${USE_GPU_HOST}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        -e OMP_NUM_THREADS=1 \
        ${local_volumes} \
        --net host \
        -w /apollo \
        --add-host "${CYBER_INSIDE}:172.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${CYBER_INSIDE}" \
        --shm-size "${SHM_SIZE}" \
        --pid=host \
        "${image}" \
        /bin/bash

    if [ $? -ne 0 ]; then
        error "Failed to start docker container \"${CYBER_CONTAINER}\" based on image: ${image}"
        exit 1
    fi
    set +x
}

function after_run_setup() {
    if [[ "${USER}" != "root" ]]; then
        docker exec -u root "${CYBER_CONTAINER}" \
            bash -c '/apollo/scripts/docker_start_user.sh'
    fi
    # if [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    #    warning "!!! Due to problem with 'docker exec' on Drive PX2 platform," \
    #            "please run '/apollo/scripts/docker_start_user.sh' the first" \
    #            "time you login into cyber docker !!!"
    # fi
}

function main() {
    check_agreement
    check_host_environment

    parse_arguments "$@"
    determine_target_version_and_arch "${CUSTOM_VERSION}"

    check_multi_arch_support

    local image="${DOCKER_REPO}:${CUSTOM_VERSION}"

    geo_specific_config "${GEOLOC}"
    docker_pull_if_needed "${image}" "${USE_LOCAL_IMAGE}"

    remove_existing_cyber_container

    determine_gpu_use_host
    info "DOCKER_RUN_CMD evaluated to: ${DOCKER_RUN_CMD}"

    start_cyber_container "${image}"

    after_run_setup

    ok "Congrats, you have successfully finished setting up Apollo cyber docker environment." \
        "To login into cyber container, please run the following command:"
    ok "  bash docker/scripts/cyber_into.sh"
    ok "Enjoy!"
}

main "$@"
