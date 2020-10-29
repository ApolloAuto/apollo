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

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${APOLLO_ROOT_DIR}/scripts/apollo.bashrc"

CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"
DOCKER_REPO="apolloauto/apollo"
DEV_INSIDE="in-dev-docker"
HOST_ARCH="$(uname -m)"
TARGET_ARCH="$(uname -m)"
DOCKER_RUN="docker run"
FAST_MODE="no"
USE_GPU_HOST=0
VOLUME_VERSION="latest"
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

function parse_arguments() {
    while [ $# -gt 0 ]; do
        local opt="$1"
        shift
        case "${opt}" in
            -f | --fast)
                FAST_MODE="yes"
                ;;

            *)
                warning "Unknown option: ${opt}"
                exit 2
                ;;
        esac
    done # End while loop

}

function determine_dev_image() {
    local version=""
    if [ "${TARGET_ARCH}" = "x86_64" ]; then
        version="${VERSION_X86_64}"
    elif [ "${TARGET_ARCH}" = "aarch64" ]; then
        version="${VERSION_AARCH64}"
    else
        error "Logic can't reach here! Please file an issue to Apollo GitHub."
        exit 3
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

function setup_devices_and_mount_local_volumes() {
    local __retval="$1"

    [ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

    source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"
    setup_device

    local volumes="-v $APOLLO_ROOT_DIR:/apollo"

    local os_release="$(lsb_release -rs)"
    case "${os_release}" in
        16.04)
            warning "[Deprecated] Support for Ubuntu 16.04 will be removed" \
                "in the near future. Please upgrade to ubuntu 18.04+."
            volumes="${volumes} -v /dev:/dev"
            ;;
        18.04 | 20.04 | *)
            volumes="${volumes} -v /dev:/dev"
            ;;
    esac
    volumes="${volumes} -v /media:/media \
                        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                        -v /etc/localtime:/etc/localtime:ro \
                        -v /usr/src:/usr/src \
                        -v /lib/modules:/lib/modules"
    volumes="$(tr -s " " <<<"${volumes}")"
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
        if [ ! -x "$(command -v ${nv_driver})" ]; then
            warning "No nvidia-driver found. CPU will be used"
        elif [ -z "$(eval ${nv_driver})" ]; then
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
    info "Start pulling docker image ${img} ..."
    if ! docker pull "${img}"; then
        error "Failed to pull docker image : ${img}"
        exit 1
    fi
}

# Note(storypku): Reuse existing docker volumes for CI
function reuse_or_start_volume() {
    local container="$1"
    if docker ps --format "{{.Names}}" | grep -q "${container}"; then
        info "Found existing volume \"${container}\", will be reused."
        return
    fi
    local image="$2"
    docker_pull "${image}"
    docker run -id --rm --name "${container}" "${image}"
}

function start_map_volume() {
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
        reuse_or_start_volume "${map_volume}" "${map_image}"
        MAP_VOLUME_CONF="${MAP_VOLUME_CONF} --volumes-from ${map_volume}"
    fi
}

function mount_map_volumes() {
    info "Starting mounting map volumes ..."
    if [ "$FAST_MODE" = "no" ]; then
        for map_name in ${DEFAULT_MAPS[@]}; do
            start_map_volume "${map_name}" "${VOLUME_VERSION}"
        done
    else
        for map_name in ${DEFAULT_TEST_MAPS[@]}; do
            start_map_volume "${map_name}" "${VOLUME_VERSION}"
        done
    fi
}

function mount_other_volumes() {
    info "Mount other volumes ..."
    local volume_conf=

    # AUDIO
    local audio_volume="apollo_audio_volume_${USER}"
    local audio_image="${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest"
    reuse_or_start_volume "${audio_volume}" "${audio_image}"
    volume_conf="${volume_conf} --volumes-from ${audio_volume}"

    # YOLOV4
    local yolov4_volume="apollo_yolov4_volume_${USER}"
    local yolov4_image="${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest"
    reuse_or_start_volume "${yolov4_volume}" "${yolov4_image}"
    volume_conf="${volume_conf} --volumes-from ${yolov4_volume}"

    # FASTER_RCNN
    local faster_rcnn_volume="apollo_faster_rcnn_volume_${USER}"
    local faster_rcnn_image="${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest"
    reuse_or_start_volume "${faster_rcnn_volume}" "${faster_rcnn_image}"
    volume_conf="${volume_conf} --volumes-from ${faster_rcnn_volume}"

    if [ "${TARGET_ARCH}" = "x86_64" ]; then
        local local_3rdparty_volume="apollo_local_third_party_volume_${USER}"
        local local_3rdparty_image="${DOCKER_REPO}:local_third_party_volume-${TARGET_ARCH}-latest"
        reuse_or_start_volume "${local_3rdparty_volume}" "${local_3rdparty_image}"
        volume_conf="${volume_conf} --volumes-from ${local_3rdparty_volume}"
    fi

    OTHER_VOLUME_CONF="${volume_conf}"
}

function main() {
    check_host_environment

    parse_arguments "$@"
    determine_dev_image

    if ! docker_pull "${APOLLO_DEV_IMAGE}"; then
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

    ${DOCKER_RUN} -i --rm \
        --privileged \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        -e DOCKER_IMG="${APOLLO_DEV_IMAGE}" \
        -e USE_GPU_HOST="${USE_GPU_HOST}" \
        ${MAP_VOLUME_CONF} \
        ${OTHER_VOLUME_CONF} \
        ${local_volumes} \
        --net host \
        -w /apollo \
        --add-host "${DEV_INSIDE}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${DEV_INSIDE}" \
        --shm-size 2G \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        "${APOLLO_DEV_IMAGE}" \
        /bin/bash /apollo/scripts/apollo_ci.sh

    if [ $? -ne 0 ]; then
        error "CI failed based on image: ${APOLLO_DEV_IMAGE}"
        exit 1
    fi
    set +x
    ok "CI success based on image: ${APOLLO_DEV_IMAGE}"
}

main "$@"
