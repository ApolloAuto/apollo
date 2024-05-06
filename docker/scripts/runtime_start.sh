#!/usr/bin/env bash

###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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

DOCKER_REPO="apolloauto/apollo"
RUNTIME_CONTAINER="apollo_runtime_${USER}"
RUNTIME_INSIDE="in-runtime-docker"
RUNTIME_STANDALONE="false"

TARGET_ARCH="$(uname -m)"

VERSION_X86_64="runtime-x86_64-18.04-20220803_1505"
USER_VERSION_OPT=

FAST_MODE="y"

GEOLOC=

USE_LOCAL_IMAGE=0

VOLUME_VERSION="latest"
SHM_SIZE="2G"
USER_SPECIFIED_MAPS=
MAP_VOLUMES_CONF=
OTHER_VOLUMES_CONF=

DEFAULT_MAPS=(
    sunnyvale_big_loop
    sunnyvale_loop
    sunnyvale_with_two_offices
    san_mateo
)

DEFAULT_TEST_MAPS=(
    sunnyvale_loop
)

function show_usage() {
    cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help             Display this help and exit.
    -f, --fast             Fast mode without pulling all map volumes.
    -g, --geo <us|cn|none> Pull docker image from geolocation specific registry mirror.
    -l, --local            Use local docker image.
    -s, --standalone       Run standalone container with all volumes and apollo itself included.
    -t, --tag <TAG>        Specify docker image with tag <TAG> to start.
    --shm-size <bytes>     Size of /dev/shm . Passed directly to "docker run"
EOF
}

function parse_arguments() {
    local custom_version=""
    local shm_size=""
    local geo=""
    local fast_mode=""

    while [ $# -gt 0 ]; do
        local opt="$1"
        shift
        case "${opt}" in
            -t | --tag)
                if [ -n "${custom_version}" ]; then
                    warning "Multiple option ${opt} specified, only the last one will take effect."
                fi
                custom_version="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_version}"
                ;;

            -h | --help)
                show_usage
                exit 1
                ;;

            -f | --fast)
                fast_mode="$1"
                shift
                optarg_check_for_opt "${opt}" "${fast_mode}"
                ;;

            -g | --geo)
                geo="$1"
                shift
                optarg_check_for_opt "${opt}" "${geo}"
                ;;

            -l | --local)
                USE_LOCAL_IMAGE=1
                ;;

            -s | --standalone)
                RUNTIME_STANDALONE="true"
                ;;

            --shm-size)
                shm_size="$1"
                shift
                optarg_check_for_opt "${opt}" "${shm_size}"
                ;;
            --map)
                map_name="$1"
                shift
                USER_SPECIFIED_MAPS="${USER_SPECIFIED_MAPS} ${map_name}"
                ;;
            *)
                warning "Unknown option: ${opt}"
                exit 2
                ;;
        esac
    done # End while loop

    [[ -n "${fast_mode}" ]] && FAST_MODE="${fast_mode}"
    [[ -n "${geo}" ]] && GEOLOC="${geo}"
    [[ -n "${custom_version}" ]] && USER_VERSION_OPT="${custom_version}"
    [[ -n "${shm_size}" ]] && SHM_SIZE="${shm_size}"
}

function determine_runtime_image() {
    local version="$1"
    if [[ -n "${version}" ]]; then
        RUNTIME_IMAGE="${DOCKER_REPO}:${version}"
        return
    fi

    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        version="${VERSION_X86_64}"
        RUNTIME_IMAGE="${DOCKER_REPO}:${version}"
    else
        error "Runtime Docker for ${TARGET_ARCH} Not Ready. Exiting..."
        exit 3
    fi

}

function check_host_environment() {
    if [[ "${HOST_OS}" != "Linux" ]]; then
        warning "Linux ONLY support for Apollo Runtime Docker!"
        exit 1
    fi
    if [[ "${HOST_ARCH}" != "x86_64" ]]; then
        warning "Apollo Runtime Docker supports x86_64 ONLY!"
        exit 2
    fi
}

function setup_devices_and_mount_local_volumes() {
    local __retval="$1"

    source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"
    setup_device

    local volumes=""
    if $RUNTIME_STANDALONE; then
        volumes="-v ${APOLLO_ROOT_DIR}/data:/apollo/data \
                 -v ${APOLLO_ROOT_DIR}/modules/calibration/data:/apollo/modules/calibration/data \
                 -v ${APOLLO_ROOT_DIR}/modules/map/data:/apollo/modules/map/data \
                 -v ${APOLLO_ROOT_DIR}/output:/apollo/output"
    else
        volumes="-v ${APOLLO_ROOT_DIR}:/apollo"
    fi

    [ -d "${APOLLO_CONFIG_HOME}" ] || mkdir -p "${APOLLO_CONFIG_HOME}"
    volumes="-v ${APOLLO_CONFIG_HOME}:${APOLLO_CONFIG_HOME} ${volumes}"


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
                        -v /lib/modules:/lib/modules"
    volumes="$(tr -s " " <<<"${volumes}")"
    eval "${__retval}='${volumes}'"
}

function docker_pull() {
    local img="$1"
    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        if docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "${img}"; then
            info "Local image ${img} found and will be used."
            return
        fi
        warning "Image ${img} not found locally although local mode enabled. Trying to pull from remote registry."
    fi
    if [[ -n "${GEO_REGISTRY}" ]]; then
        img="${GEO_REGISTRY}/${img}"
    fi

    info "Start pulling docker image ${img} ..."
    if ! docker pull "${img}"; then
        error "Failed to pull docker image : ${img}"
        exit 1
    fi
}

function docker_restart_volume() {
    local volume="$1"
    local image="$2"
    local path="$3"
    info "Create volume ${volume} from image: ${image}"
    docker_pull "${image}"
    docker volume rm "${volume}" >/dev/null 2>&1
    docker run -v "${volume}":"${path}" --rm "${image}" true
}

function restart_map_volume_if_needed() {
    local map_name="$1"
    local map_version="$2"
    local map_volume="apollo_map_volume-${map_name}_${USER}"
    local map_path="/apollo/modules/map/data/${map_name}"

    if [[ ${MAP_VOLUMES_CONF} == *"${map_volume}"* ]]; then
        info "Map ${map_name} has already been included."
    else
        local map_image=
        if [ "${TARGET_ARCH}" = "aarch64" ]; then
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${TARGET_ARCH}-${map_version}"
        else
            map_image="${DOCKER_REPO}:map_volume-${map_name}-${map_version}"
        fi
        info "Load map ${map_name} from image: ${map_image}"

        docker_restart_volume "${map_volume}" "${map_image}" "${map_path}"
        MAP_VOLUMES_CONF="${MAP_VOLUMES_CONF} --volume ${map_volume}:${map_path}"
    fi
}

function mount_map_volumes() {
    info "Starting mounting map volumes ..."
    if [ -n "${USER_SPECIFIED_MAPS}" ]; then
        for map_name in ${USER_SPECIFIED_MAPS}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi

    if [ "$FAST_MODE" == "n" ] || [ "$FAST_MODE" == "no" ]; then
        for map_name in ${DEFAULT_MAPS[@]}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
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

    # AUDIO
    local audio_volume="apollo_audio_volume_${USER}"
    local audio_image="${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest"
    local audio_path="/apollo/modules/audio/data/"
    docker_restart_volume "${audio_volume}" "${audio_image}" "${audio_path}"
    volume_conf="${volume_conf} --volume ${audio_volume}:${audio_path}"

    # YOLOV4
    local yolov4_volume="apollo_yolov4_volume_${USER}"
    local yolov4_image="${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest"
    local yolov4_path="/apollo/modules/perception/camera/lib/obstacle/detector/yolov4/model/"
    docker_restart_volume "${yolov4_volume}" "${yolov4_image}" "${yolov4_path}"
    volume_conf="${volume_conf} --volume ${yolov4_volume}:${yolov4_path}"

    # FASTER_RCNN
    local faster_rcnn_volume="apollo_faster_rcnn_volume_${USER}"
    local faster_rcnn_image="${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest"
    local faster_rcnn_path="/apollo/modules/perception/production/data/perception/camera/models/traffic_light_detection/faster_rcnn_model"
    docker_restart_volume "${faster_rcnn_volume}" "${faster_rcnn_image}" "${faster_rcnn_path}"
    volume_conf="${volume_conf} --volume ${faster_rcnn_volume}:${faster_rcnn_path}"

    # SMOKE
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        local smoke_volume="apollo_smoke_volume_${USER}"
        local smoke_image="${DOCKER_REPO}:smoke_volume-yolo_obstacle_detection_model-${TARGET_ARCH}-latest"
        local smoke_path="/apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector/smoke_libtorch_model"
        docker_restart_volume "${smoke_volume}" "${smoke_image}" "${smoke_path}"
        volume_conf="${volume_conf} --volume ${smoke_volume}:${smoke_path}"
    fi

    OTHER_VOLUMES_CONF="${volume_conf}"
}

function main() {
    check_host_environment

    parse_arguments "$@"

    determine_runtime_image "${USER_VERSION_OPT}"
    geo_specific_config "${GEOLOC}"

    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        info "Start Runtime container based on local image : ${RUNTIME_IMAGE}"
    fi

    if ! docker_pull "${RUNTIME_IMAGE}"; then
        error "Failed to pull docker image ${RUNTIME_IMAGE}"
        exit 1
    fi

    $RUNTIME_STANDALONE && RUNTIME_CONTAINER="apollo_runtime_standalone_$USER"

    info "Check and remove existing Apollo Runtime container ..."
    remove_container_if_exists "${RUNTIME_CONTAINER}"

    info "Determine whether host GPU is available ..."
    determine_gpu_use_host
    info "USE_GPU_HOST: ${USE_GPU_HOST}"

    local local_volumes=
    setup_devices_and_mount_local_volumes local_volumes

    mount_map_volumes
    $RUNTIME_STANDALONE || mount_other_volumes

    info "Starting docker container \"${RUNTIME_CONTAINER}\" ..."

    local local_host="$(hostname)"
    local display="${DISPLAY:-:0}"
    local docker_user="${USER}"
    local docker_uid="$(id -u)"
    local docker_group="$(id -g -n)"
    local docker_gid="$(id -g)"

    set -x
    ${DOCKER_RUN_CMD} -itd \
        --privileged \
        --name "${RUNTIME_CONTAINER}" \
        -e DISPLAY="${display}" \
        -e USER="${USER}" \
        -e DOCKER_USER="${docker_user}" \
        -e DOCKER_USER_ID="${docker_uid}" \
        -e DOCKER_GRP="${docker_group}" \
        -e DOCKER_GRP_ID="${docker_gid}" \
        -e DOCKER_IMG="${RUNTIME_IMAGE}" \
        -e USE_GPU_HOST="${USE_GPU_HOST}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        ${MAP_VOLUMES_CONF} \
        ${OTHER_VOLUMES_CONF} \
        ${local_volumes} \
        --net host \
        -w /apollo \
        --add-host "${RUNTIME_INSIDE}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${RUNTIME_INSIDE}" \
        --shm-size "${SHM_SIZE}" \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        "${RUNTIME_IMAGE}" \
        /bin/bash

    if [ $? -ne 0 ]; then
        error "Failed to start docker container \"${RUNTIME_CONTAINER}\" based on image: ${RUNTIME_IMAGE}"
        exit 1
    fi
    set +x

    postrun_start_user ${RUNTIME_CONTAINER}

    $RUNTIME_STANDALONE && docker exec -u root "${RUNTIME_CONTAINER}" bash -c "chown -R ${docker_uid}:${docker_gid} /apollo"

    ok "Congratulations! You have successfully finished setting up Apollo Runtime Environment."
    ok "To login into the newly created ${RUNTIME_CONTAINER} container, please run the following command:"
    $RUNTIME_STANDALONE || ok "  bash docker/scripts/runtime_into.sh"
    $RUNTIME_STANDALONE && ok "  bash docker/scripts/runtime_into_standalone.sh"
    ok "Enjoy!"
}

main "$@"
