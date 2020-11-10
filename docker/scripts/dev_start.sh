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

AGREEMENT_FILE="${APOLLO_ROOT_DIR}/scripts/AGREEMENT.txt"
CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

DOCKER_REPO="apolloauto/apollo"
APOLLO_DEV="apollo_dev_${USER}"
DEV_INSIDE="in-dev-docker"

SUPPORTED_ARCHS=(x86_64 aarch64)
HOST_ARCH="$(uname -m)"
TARGET_ARCH="$(uname -m)"

VERSION_X86_64="dev-x86_64-18.04-20201110_0617"
TESTING_VERSION_X86_64="dev-x86_64-18.04-testing-20201110_0646"

VERSION_AARCH64="dev-aarch64-18.04-20201006_0154"
USER_VERSION_OPT=

DOCKER_RUN="docker run"
FAST_MODE="no"

GEOLOC=
GEO_REGISTRY=

USE_LOCAL_IMAGE=0
CUSTOM_DIST=
USE_GPU_HOST=0
USER_AGREED="no"

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
  sunnyvale_big_loop
  sunnyvale_loop
)

# Check whether user has agreed license agreement
function check_agreement() {
    local agreement_record="${HOME}/.apollo_agreement.txt"
    if [ -e "$agreement_record" ]; then
        return
    fi

    if [ ! -e "$AGREEMENT_FILE" ]; then
        error "AGREEMENT $AGREEMENT_FILE does not exist."
        exit 1
    fi

    if [ "$USER_AGREED" = "yes" ]; then
        local progname="$(basename $0)"
        cat "${AGREEMENT_FILE}" > "${agreement_record}"
        local msgtext="By specifying '-y' option when running ${progname}, \
you have agreed to the license agreement above."
        echo "${msgtext}" | tee -a "${agreement_record}"
    else
        cat "${AGREEMENT_FILE}"
        local tip="Type 'y' or 'Y' to agree to the license agreement above, or type any other key to exit"

        echo -n "${tip}"
        read -r -n 1 user_agreed
        echo

        if [[ "${user_agreed}" = "y" || "${user_agreed}" == "Y" ]]; then
            cat "${AGREEMENT_FILE}" > "${agreement_record}"
            echo "${tip}" >> "${agreement_record}"
            echo "${user_agreed}" >> $agreement_record
        else
            exit 1
        fi
    fi
}

function _optarg_check_for_opt() {
    local opt="$1"
    local optarg="$2"

    if [[ -z "${optarg}" || "${optarg}" =~ ^-.* ]]; then
        error "Missing argument for ${opt}. Exiting..."
        exit 2
    fi
}

function show_usage() {
cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help             Display this help and exit.
    -f, --fast             Fast mode without pulling all map volumes.
    -g, --geo <us|cn|none> Pull docker image from geolocation specific registry mirror.
    -l, --local            Use local docker image.
    -t, --tag <TAG>        Specify docker image with tag <TAG> to start.
    -d, --dist             Specify Apollo distribution(stable/testing)
    --shm-size <bytes>     Size of /dev/shm . Passed directly to "docker run"
    -y                     Agree to Apollo License Agreement non-interactively.
    stop                   Stop all running Apollo containers.
EOF
}

function stop_all_apollo_containers_for_user() {
    local force="$1"
    local running_containers
    running_containers="$(docker ps -a --format '{{.Names}}')"
    for container in ${running_containers[*]} ; do
        if [[ "${container}" =~ apollo_.*_${USER} ]] ; then
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

function geo_specific_config() {
    local geo="$1"
    if [[ -z "${geo}" ]]; then
        info "Use default GeoLocation settings"
    elif [[ "${geo}" == "cn" ]]; then
        info "GeoLocation settings for Mainland China"
        GEO_REGISTRY="registry.baidubce.com"
    else
        info "GeoLocation settings for ${geo} is not ready, fallback to default"
    fi
}

function parse_arguments() {
    local custom_version=""
    local custom_dist=""
    local shm_size=""
    local geo=""

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

        -d|--dist)
            custom_dist="$1"; shift
            _optarg_check_for_opt "${opt}" "${custom_dist}"
            ;;

        -h|--help)
            show_usage
            exit 1
            ;;

        -f|--fast)
            FAST_MODE="yes"
            ;;

        -g|--geo)
            geo="$1"; shift
            _optarg_check_for_opt "${opt}" "${geo}"
            ;;

        -l|--local)
            USE_LOCAL_IMAGE=1
            ;;

        --shm-size)
            shm_size="$1"; shift
            _optarg_check_for_opt "${opt}" "${shm_size}"
            ;;

        --map)
            map_name="$1"; shift
            USER_SPECIFIED_MAPS="${USER_SPECIFIED_MAPS} ${map_name}"
            ;;
        -y)
            USER_AGREED="yes"
            ;;
        stop)
            stop_all_apollo_containers_for_user "-f"
            exit 0
            ;;
        *)
            warning "Unknown option: ${opt}"
            exit 2
            ;;
        esac
    done # End while loop

    [[ -n "${geo}" ]] && GEOLOC="${geo}"
    [[ -n "${custom_version}" ]] && USER_VERSION_OPT="${custom_version}"
    [[ -n "${custom_dist}" ]] && CUSTOM_DIST="${custom_dist}"
    [[ -n "${shm_size}" ]] && SHM_SIZE="${shm_size}"
}

function determine_dev_image() {
    local version="$1"
    # If no custom version specified
    if [[ -z "${version}" ]]; then
        if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
            if [[ "${CUSTOM_DIST}" == "testing" ]]; then
                version="${TESTING_VERSION_X86_64}"
            else
                version="${VERSION_X86_64}"
            fi
        elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
            version="${VERSION_AARCH64}"
        else
            error "Logic can't reach here! Please report this issue to Apollo@GitHub."
            exit 3
        fi
    fi
    APOLLO_DEV_IMAGE="${DOCKER_REPO}:${version}"
}

function check_host_environment() {
    local kernel="$(uname -s)"
    if [[ "${kernel}" != "Linux" ]]; then
        warning "Running Apollo dev container on ${kernel} is UNTESTED, exiting..."
        exit 1
    fi
}

function check_target_arch() {
    local arch="${TARGET_ARCH}"
    for ent in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${ent}" == "${TARGET_ARCH}" ]]; then
            return 0
        fi
    done
    error "Unsupported target architecture: ${TARGET_ARCH}."
    exit 1
}

function setup_devices_and_mount_local_volumes() {
    local __retval="$1"

    [ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

    source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"
    setup_device

    local volumes="-v $APOLLO_ROOT_DIR:/apollo"
    local teleop="${APOLLO_ROOT_DIR}/../apollo-teleop"
    if [ -d "${teleop}" ]; then
        volumes="-v ${teleop}:/apollo/modules/teleop ${volumes}"
    fi

    local os_release="$(lsb_release -rs)"
    case "${os_release}" in
        16.04)
            warning "[Deprecated] Support for Ubuntu 16.04 will be removed" \
                    "in the near future. Please upgrade to ubuntu 18.04+."
            volumes="${volumes} -v /dev:/dev"
            ;;
        18.04|20.04|*)
            volumes="${volumes} -v /dev:/dev"
            ;;
    esac
    # local tegra_dir="/usr/lib/aarch64-linux-gnu/tegra"
    # if [[ "${TARGET_ARCH}" == "aarch64" && -d "${tegra_dir}" ]]; then
    #    volumes="${volumes} -v ${tegra_dir}:${tegra_dir}:ro"
    # fi
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
        if [[ -x "$(which nvidia-container-toolkit)" ]]; then
            if dpkg --compare-versions "${DOCKER_VERSION}" "ge" "19.03"; then
                DOCKER_RUN="docker run --gpus all"
            else
                warning "You must upgrade to Docker-CE 19.03+ to access GPU from container!"
                USE_GPU_HOST=0
            fi
        elif [[ -x "$(which nvidia-docker)" ]]; then
            DOCKER_RUN="nvidia-docker run"
        else
            USE_GPU_HOST=0
            warning "Cannot access GPU from within container. Please install " \
                    "latest Docker and NVIDIA Container Toolkit as described by: "
            warning "  ${nv_docker_doc}"
        fi
    fi
}

function remove_existing_dev_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "${APOLLO_DEV}"; then
        docker stop "${APOLLO_DEV}" >/dev/null
        docker rm -v -f "${APOLLO_DEV}" >/dev/null
    fi
}

function docker_pull() {
    local img="$1"
    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        if docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "${img}" ; then
            info "Local image ${img} found and will be used."
            return
        fi
        warning "Image ${img} not found locally although local mode enabled. Trying to pull from remote registry."
    fi
    if [[ -n "${GEO_REGISTRY}" ]]; then
        img="${GEO_REGISTRY}/${img}"
    fi

    info "Start pulling docker image ${img} ..."
    if ! docker pull "${img}" ; then
        error "Failed to pull docker image : ${img}"
        exit 1
    fi
}

function docker_restart_volume() {
    local container="$1"
    local image="$2"
    info "Restart volume ${container} from image: ${image}"
    docker stop "${container}" &>/dev/null
    docker_pull "${image}"
    docker run -itd --rm --name "${container}" "${image}"
}

function restart_map_volume_if_needed() {
    local map_name="$1"
    local map_version="$2"
    local map_volume="apollo_map_volume-${map_name}_${USER}"
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

        docker_restart_volume "${map_volume}" "${map_image}"
        MAP_VOLUMES_CONF="${MAP_VOLUMES_CONF} --volumes-from ${map_volume}"
    fi
}

function mount_map_volumes() {
    info "Starting mounting map volumes ..."
    if [ -n "${USER_SPECIFIED_MAPS}" ]; then
        for map_name in ${USER_SPECIFIED_MAPS}; do
            restart_map_volume_if_needed "${map_name}" "${VOLUME_VERSION}"
        done
    fi

    if [[ "$FAST_MODE" == "no" ]]; then
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
    docker_restart_volume "${audio_volume}" "${audio_image}"
    volume_conf="${volume_conf} --volumes-from ${audio_volume}"

    # YOLOV4
    local yolov4_volume="apollo_yolov4_volume_${USER}"
    local yolov4_image="${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest"
    docker_restart_volume "${yolov4_volume}" "${yolov4_image}"
    volume_conf="${volume_conf} --volumes-from ${yolov4_volume}"

    # FASTER_RCNN
    local faster_rcnn_volume="apollo_faster_rcnn_volume_${USER}"
    local faster_rcnn_image="${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest"
    docker_restart_volume "${faster_rcnn_volume}" "${faster_rcnn_image}"
    volume_conf="${volume_conf} --volumes-from ${faster_rcnn_volume}"

    if [ "${TARGET_ARCH}" = "x86_64" ]; then
        local local_3rdparty_volume="apollo_local_third_party_volume_${USER}"
        local local_3rdparty_image="${DOCKER_REPO}:local_third_party_volume-${TARGET_ARCH}-latest"
        docker_restart_volume "${local_3rdparty_volume}" "${local_3rdparty_image}"
        volume_conf="${volume_conf} --volumes-from ${local_3rdparty_volume}"
    fi

    OTHER_VOLUMES_CONF="${volume_conf}"
}

function post_run_setup() {
    if [ "${USER}" != "root" ]; then
        docker exec -u root "${APOLLO_DEV}" bash -c '/apollo/scripts/docker_start_user.sh'
    fi
}

function main() {
    check_host_environment
    check_target_arch

    parse_arguments "$@"
    check_agreement

    determine_dev_image "${USER_VERSION_OPT}"
    geo_specific_config "${GEOLOC}"

    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        info "Start docker container based on local image : ${APOLLO_DEV_IMAGE}"
    fi

    if ! docker_pull "${APOLLO_DEV_IMAGE}" ; then
        error "Failed to pull docker image ${APOLLO_DEV_IMAGE}"
        exit 1
    fi

    info "Check and remove existing Apollo dev container ..."
    remove_existing_dev_container

    info "Determine whether host GPU is available ..."
    determine_gpu_use_host
    info "USE_GPU_HOST: ${USE_GPU_HOST}"

    local local_volumes=
    setup_devices_and_mount_local_volumes local_volumes

    mount_map_volumes
    mount_other_volumes

    info "Starting docker container \"${APOLLO_DEV}\" ..."

    local local_host="$(hostname)"
    local display="${DISPLAY:-:0}"
    local user="${USER}"
    local uid="$(id -u)"
    local group="$(id -g -n)"
    local gid="$(id -g)"

    set -x

    ${DOCKER_RUN} -itd  \
        --privileged    \
        --name "${APOLLO_DEV}"      \
        -e DISPLAY="${display}"     \
        -e DOCKER_USER="${user}"    \
        -e USER="${user}"           \
        -e DOCKER_USER_ID="${uid}"  \
        -e DOCKER_GRP="${group}"    \
        -e DOCKER_GRP_ID="${gid}"   \
        -e DOCKER_IMG="${APOLLO_DEV_IMAGE}" \
        -e USE_GPU_HOST="${USE_GPU_HOST}"   \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        ${MAP_VOLUMES_CONF}      \
        ${OTHER_VOLUMES_CONF}    \
        ${local_volumes}        \
        --net host \
        -w /apollo \
        --add-host "${DEV_INSIDE}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${DEV_INSIDE}" \
        --shm-size "${SHM_SIZE}"   \
        --pid=host      \
        -v /dev/null:/dev/raw1394 \
        "${APOLLO_DEV_IMAGE}" \
        /bin/bash

    if [ $? -ne 0 ];then
        error "Failed to start docker container \"${APOLLO_DEV}\" based on image: ${APOLLO_DEV_IMAGE}"
        exit 1
    fi
    set +x

    post_run_setup

    ok "Congratulations! You have successfully finished setting up Apollo Dev Environment."
    ok "To login into the newly created ${APOLLO_DEV} container, please run the following command:"
    ok "  bash docker/scripts/dev_into.sh"
    ok "Enjoy!"
}

main "$@"
