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
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CURR_DIR}/docker_base.sh"

VERSION_X86_64="cyber-x86_64-18.04-20210315_1535"
TESTING_VERSION_X86_64="cyber-x86_64-18.04-testing-20210108_1510"

#L4T
VERSION_AARCH64="cyber-aarch64-18.04-20201217_1302"

CYBER_CONTAINER="apollo_cyber_${USER}"
CYBER_INSIDE="in-cyber-docker"

DOCKER_REPO="apolloauto/apollo"
DOCKER_PULL_CMD="docker pull"
SHM_SIZE="2G"

SUPPORTED_ARCHS=(x86_64 aarch64)
TARGET_ARCH=""

USE_LOCAL_IMAGE=0
CUSTOM_DIST=
CUSTOM_VERSION=
GEOLOC=

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
                optarg_check_for_opt "${opt}" "${geo}"
                ;;
            -t | --tag)
                if [[ ! -z "${custom_version}" ]]; then
                    warning "Multiple option ${opt} specified, only the last one will take effect."
                fi
                custom_version="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_version}"
                ;;
            -d | --dist)
                custom_dist="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_dist}"
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
                optarg_check_for_opt "${opt}" "${target_arch}"
                _target_arch_check "${target_arch}"
                ;;
            --shm-size)
                shm_size="$1"
                shift
                optarg_check_for_opt "${opt}" "${shm_size}"
                ;;
            stop)
                local force="$1"
                info "Now, stop all Apollo containers created by ${USER} ..."
                stop_all_apollo_containers "${force}"
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


function setup_devices_and_mount_volumes() {
    local __retval="$1"

    if [[ "${HOST_ARCH}" == "${TARGET_ARCH}" ]]; then
        source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"
        setup_device
    fi

    local volumes
    volumes="-v ${APOLLO_ROOT_DIR}:/apollo"

    [ -d "${APOLLO_CONFIG_HOME}" ] || mkdir -p "${APOLLO_CONFIG_HOME}"
    volumes="-v ${APOLLO_CONFIG_HOME}:${APOLLO_CONFIG_HOME} ${volumes}"


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

function main() {
    check_agreement

    parse_arguments "$@"
    determine_target_version_and_arch "${CUSTOM_VERSION}"

    check_multi_arch_support

    local image="${DOCKER_REPO}:${CUSTOM_VERSION}"

    geo_specific_config "${GEOLOC}"
    info "GEO_REGISTRY evaluated to: ${GEO_REGISTRY}"

    docker_pull_if_needed "${image}" "${USE_LOCAL_IMAGE}"

    remove_container_if_exists "${CYBER_CONTAINER}"

    determine_gpu_use_host
    info "DOCKER_RUN_CMD evaluated to: ${DOCKER_RUN_CMD}"

    start_cyber_container "${image}"

    postrun_start_user "${CYBER_CONTAINER}"

    ok "Congratulations! You have successfully finished setting up CyberRT Docker environment."
    ok "To log into the newly created CyberRT container, please run the following command:"
    ok "  bash docker/scripts/cyber_into.sh"
    ok "Enjoy!"
}

main "$@"
