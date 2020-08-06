#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

DOCKER_REPO="apolloauto/apollo"
DEV_INSIDE="in-dev-docker"

SUPPORTED_ARCHS=" x86_64 aarch64 "
HOST_ARCH="$(uname -m)"
TARGET_ARCH="$(uname -m)"
USER_VERSION_OPT=

DOCKER_RUN="docker run"
FAST_MODE="no"
USE_LOCAL_IMAGE="no"
USE_GPU_HOST=0

VOLUME_VERSION="latest"
USER_SPECIFIED_MAP=
MAP_VOLUME_CONF=
OTHER_VOLUME_CONF=

DEFAULT_MAPS=(
  sunnyvale_big_loop
  sunnyvale_loop
  sunnyvale_with_two_offices
  san_mateo
)

DEFAULT_TEST_MAPS=(
  sunnyvale_big_loop
  sunnyvale_loop
)

eval $(grep ^VERSION_X86_64= ${APOLLO_ROOT_DIR}/docker/scripts/dev_start.sh)
eval $(grep ^VERSION_AARCH64= ${APOLLO_ROOT_DIR}/docker/scripts/dev_start.sh)

function _optarg_check_for_opt() {
    local opt="$1"
    local optarg="$2"

    if [[ -z "${optarg}" || "${optarg}" =~ ^-.* ]]; then
        error "Missing argument for ${opt}. Exiting..."
        exit 2
    fi
}

function parse_arguments() {
    local custom_version=""

    while [ $# -gt 0 ] ; do
        local opt="$1"; shift
        case "${opt}" in
        -t|--tag)
            if [ -n "${custom_version}" ]; then
                warning "Multiple option ${opt} specified, only the last one will take effect."
            fi
            custom_version="$1"; shift
            _optarg_check_for_opt "${opt}" "${custom_version}"
            ;;

        -f|--fast)
            FAST_MODE="yes"
            ;;

        -l|--local)
            USE_LOCAL_IMAGE="yes"
            ;;

        --map)
            map_name="$1"; shift
            USER_SPECIFIED_MAP="${USER_SPECIFIED_MAP} ${map_name}"
            ;;
        *)
            warning "Unknown option: ${opt}"
            exit 2
            ;;
        esac
    done # End while loop

    [ -n "${custom_version}" ] && USER_VERSION_OPT="${custom_version}"
}

function determine_dev_image() {
    local version="$1"
    # If no custom version specified
    if [ -z "${version}" ]; then
        if [ "${USE_LOCAL_IMAGE}" = "yes" ]; then
            version="local_dev"
        elif [ "${TARGET_ARCH}" = "x86_64" ]; then
            version="${VERSION_X86_64}"
        elif [ "${TARGET_ARCH}" = "aarch64" ]; then
            version="${VERSION_AARCH64}"
        else
            error "Logic can't reach here! Please file an issue to Apollo GitHub."
            exit 3
        fi
    fi
    APOLLO_DEV_IMAGE="${DOCKER_REPO}:${version}"
}

function check_host_environment() {
    local kernel="$(uname -s)"
    if [ "${kernel}" != "Linux" ]; then
        warning "Running Apollo dev container on ${kernel} is UNTESTED, exiting..."
        exit 1
    fi
}

function check_target_arch() {
    local arch="${TARGET_ARCH}"
    if [[ "${SUPPORTED_ARCHS}" != *" ${arch} "* ]]; then
        error "Unsupported target architecture: ${arch}. Allowed values:${SUPPORTED_ARCHS}"
        exit 1
    fi
}

function setup_devices_and_mount_local_volumes() {
    local __retval="$1"

    [ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

    source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh" CYBER_ONLY
    setup_device

    local volumes="-v $APOLLO_ROOT_DIR:/apollo"
    local teleop="${APOLLO_ROOT_DIR}/../apollo-teleop"
    if [ -d "${teleop}" ]; then
        volumes="-v ${teleop}:/apollo/modules/teleop ${volumes}"
    fi

    local os_release="$(lsb_release -rs)"
    case "${os_release}" in
        14.04)
            warning "[Deprecated] Support for Ubuntu 14.04 will be removed" \
                    "in the near future. Please upgrade to ubuntu 18.04+."
            ;;
        16.04|18.04|20.04|*)
            volumes="${volumes} -v /dev:/dev"
            ;;
    esac
    volumes="${volumes} -v /media:/media \
                        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                        -v /etc/localtime:/etc/localtime:ro \
                        -v /usr/src:/usr/src \
                        -v /lib/modules:/lib/modules"
    volumes="$(tr -s " " <<< "${volumes}")"
    eval "${__retval}='${volumes}'"
}

function determine_gpu_use_host() {
    if [ "${HOST_ARCH}" = "aarch64" ]; then
        if lsmod | grep -q "^nvgpu"; then
            USE_GPU_HOST=1
        fi
    else
        # Check nvidia-driver and GPU device
        local nv_driver="nvidia-smi"
        if [ ! -x "$(command -v ${nv_driver} )" ]; then
            warning "No nvidia-driver found. CPU will be used"
        elif [ -z "$(eval ${nv_driver} )" ]; then
            warning "No GPU device found. CPU will be used."
        else
            USE_GPU_HOST=1
        fi
    fi

    # Try to use GPU inside container
    local nv_docker_doc="https://github.com/NVIDIA/nvidia-docker/blob/master/README.md"
    if [ ${USE_GPU_HOST} -eq 1 ]; then
        DOCKER_VERSION=$(docker version --format '{{.Server.Version}}')
        if [ ! -z "$(which nvidia-docker)" ]; then
            DOCKER_RUN="nvidia-docker run"
            warning "nvidia-docker is deprecated. Please install latest docker " \
                    "and nvidia-container-toolkit as described by:"
            warning "  ${nv_docker_doc}"
        elif [ ! -z "$(which nvidia-container-toolkit)" ]; then
            if dpkg --compare-versions "${DOCKER_VERSION}" "ge" "19.03"; then
                DOCKER_RUN="docker run --gpus all"
            else
                warning "You must upgrade to docker-ce 19.03+ to access GPU from container!"
                USE_GPU_HOST=0
            fi
        else
            USE_GPU_HOST=0
            warning "Cannot access GPU from within container. Please install " \
                    "latest docker and nvidia-container-toolkit as described by: "
            warning "  ${nv_docker_doc}"
        fi
    fi
}


function docker_pull() {
    local img="$1"
    if [ "${USE_LOCAL_IMAGE}" = "yes" ];then
        if docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${img}" ; then
            info "Local image ${img} found and will be used."
            return
        fi
        warning "Image ${img} not found locally although local mode enabled. Trying to pull from remote registry."
    fi

    info "Start pulling docker image ${img} ..."
    if ! docker pull "${img}" ; then
        error "Failed to pull docker image : ${img}"
        exit 1
    fi
}

function restart_map_volume_if_needed() {
    local map_name="$1"
    local map_version="$2"
    local map_volume="apollo_map_volume-${map_name}_${USER}"
    if [[ ${MAP_VOLUME_CONF} == *"${map_volume}"* ]]; then
        info "Map ${map_name} has already been included."
    else
        local map_image=
        if [ "${TARGET_ARCH}" = "aarch64" ]; then
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${TARGET_ARCH}-${map_version}"
        else
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${map_version}"
        fi
        info "Load map ${map_name} from image: ${map_image}"

        MAP_VOLUME_CONF="${MAP_VOLUME_CONF} --volumes-from ${map_volume}"
    fi
}

function mount_map_volumes() {
    info "Starting mounting map volumes ..."
    if [ -n "${USER_SPECIFIED_MAP}" ]; then
        for map_name in ${USER_SPECIFIED_MAP}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi

    if [ "$FAST_MODE" = "no" ]; then
        for map_name in ${DEFAULT_MAPS[@]}; do
            restart_map_volume_if_needed ${map_name} "${VOLUME_VERSION}"
        done
    else
        for map_name in ${DEFAULT_TEST_MAPS[@]}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi
}

function mount_other_volumes() {
    info "Mount other volumes ..."
    local volume_conf=
    if [ "${FAST_MODE}" = "no" ]; then
        # YOLO3D
        local yolo3d_volume="apollo_yolo3d_volume_${USER}"
        local yolo3d_image="${DOCKER_REPO}:yolo3d_volume-${TARGET_ARCH}-latest"
        volume_conf="${volume_conf} --volumes-from ${yolo3d_volume}"
    fi

    # LOCALIZATION
    local localization_volume="apollo_localization_volume_${USER}"
    local localization_image="${DOCKER_REPO}:localization_volume-${TARGET_ARCH}-latest"
    volume_conf="${volume_conf} --volumes-from ${localization_volume}"

    if [ "${TARGET_ARCH}" = "x86_64" ]; then
        local local_3rdparty_volume="apollo_local_third_party_volume_${USER}"
        local local_3rdparty_image="${DOCKER_REPO}:local_third_party_volume-${TARGET_ARCH}-latest"
        volume_conf="${volume_conf} --volumes-from ${local_3rdparty_volume}"
    fi

    OTHER_VOLUME_CONF="${volume_conf}"
}

function main() {
    check_host_environment
    check_target_arch

    parse_arguments "$@"

    determine_dev_image "${USER_VERSION_OPT}"

    if [ "${USE_LOCAL_IMAGE}" = "yes" ];then
        info "Start docker container based on local image : ${APOLLO_DEV_IMAGE}"
    fi

    if ! docker_pull "${APOLLO_DEV_IMAGE}" ; then
        error "Failed to pull docker image."
        exit 1
    fi

    info "Determine whether host GPU is available ..."
    determine_gpu_use_host
    info "USE_GPU_HOST: ${USE_GPU_HOST}"
    local local_host="$(hostname)"
    local local_volumes=
    setup_devices_and_mount_local_volumes local_volumes

    mount_map_volumes
    mount_other_volumes
    set -x

    ${DOCKER_RUN} -it --rm   \
        --privileged    \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        ${MAP_VOLUME_CONF}      \
        ${OTHER_VOLUME_CONF}    \
        ${local_volumes}        \
        --net host \
        -w /apollo \
        --add-host "${DEV_INSIDE}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${DEV_INSIDE}" \
        --shm-size 2G   \
        --pid=host      \
        -v /dev/null:/dev/raw1394 \
        "${APOLLO_DEV_IMAGE}" \
        /bin/bash /apollo/scripts/apollo_ci.sh

    if [ $? -ne 0 ];then
        error "Failed to start docker container  based on image: ${APOLLO_DEV_IMAGE}"
        exit 1
    fi
    set +x
    ok "Enjoy!"
}

main "$@"
