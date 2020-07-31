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

VERSION_X86_64="dev-x86_64-18.04-20200729_1839"
VERSION_AARCH64="dev-aarch64-20170927_1111"

CUSTOM_VERSION=
VOLUME_VERSION="latest"
GEOLOC="none"

TARGET_ARCH="$(uname -m)"

DOCKER_RUN="docker run"
USE_GPU_HOST=0

USER_AGREED="no"
USE_LOCAL_IMAGE="no"
FAST_MODE="no"

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
    -t, --tag <version>    Specify which version of a docker image to pull.
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

function _geo_specific_config_for_cn() {
    sed -i '$i  ,"registry-mirrors": [ "http://hub-mirror.c.163.com","https://reg-mirror.qiniu.com","https://dockerhub.azk8s.cn"]' /etc/docker/daemon.json
    service docker restart
}

function geo_specific_config() {
    local geo="$1"
    if [ -z "${geo}" ] || [ "${geo}" = "none" ]; then
        info "GeoLocation based settings: use default."
    elif [ "${geo}" = "cn" ]; then
        info "GeoLocation based settings: from within China"
        _geo_specific_config_for_cn
    else
        info "GeoLocation based settings for ${geo}: not ready, fallback to default"
    fi
}

function parse_arguments() {
    local custom_version=""
    local geo="none"

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
            USE_LOCAL_IMAGE="yes"
            ;;

        --map)
            map_name="$1"; shift
            source ${APOLLO_ROOT_DIR}/docker/scripts/restart_map_volume.sh \
                "${map_name}" "${VOLUME_VERSION}"
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

    [ -n "${geo}" ] && GEOLOC="${geo}"
    [ -n "${custom_version}" ] && CUSTOM_VERSION="${custom_version}"
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

MAP_VOLUME_CONF=""
OTHER_VOLUME_CONF=""

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

function determine_gpu_use_host() {
    # Check nvidia-driver and GPU device
    local nv_driver="nvidia-smi"
    if [ ! -x "$(command -v ${nv_driver} )" ]; then
        warning "No nvidia-driver found. CPU will be used"
    elif [ -z "$(eval ${nv_driver} )" ]; then
        warning "No GPU device found. CPU will be used."
    else
        USE_GPU_HOST=1
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

function remove_existing_dev_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "${APOLLO_DEV}"; then
        info "Removing existing apollo dev container: ${APOLLO_DEV}"
        docker stop "${APOLLO_DEV}" >/dev/null
        docker rm -v -f "${APOLLO_DEV}" >/dev/null
    fi
}

function post_run_setup() {
    if [ "${USER}" != "root" ]; then
        docker exec -u root "${APOLLO_DEV}" bash -c '/apollo/scripts/docker_start_user.sh'
    fi
}

function main() {
    check_host_environment
    parse_arguments "$@"
    check_agreement

    determine_dev_image "${CUSTOM_VERSION}"
    geo_specific_config "${GEOLOC}"

    if [ "${USE_LOCAL_IMAGE}" = "yes" ];then
        info "Start docker container based on local image : ${APOLLO_DEV_IMAGE}"
    fi

    if ! docker_pull "${APOLLO_DEV_IMAGE}" ; then
        error "Failed to pull docker image."
        exit 1
    fi

    info "Check and remove existing ${APOLLO_DEV}..."
    remove_existing_dev_container

    local local_volumes=
    setup_devices_and_mount_local_volumes local_volumes

    if [ "$FAST_MODE" == "no" ]; then
        # Included default maps.
        for map_name in ${DEFAULT_MAPS[@]}; do
            source ${APOLLO_ROOT_DIR}/docker/scripts/restart_map_volume.sh ${map_name} "${VOLUME_VERSION}"
        done
        YOLO3D_VOLUME=apollo_yolo3d_volume_$USER
        docker stop ${YOLO3D_VOLUME} > /dev/null 2>&1

        YOLO3D_VOLUME_IMAGE=${DOCKER_REPO}:yolo3d_volume-${TARGET_ARCH}-latest
        docker_pull ${YOLO3D_VOLUME_IMAGE}
        docker run -it -d --rm --name ${YOLO3D_VOLUME} ${YOLO3D_VOLUME_IMAGE}

        OTHER_VOLUME_CONF="${OTHER_VOLUME_CONF} --volumes-from ${YOLO3D_VOLUME}"
    else
        for map_name in ${DEFAULT_TEST_MAPS[@]}; do
            source ${APOLLO_ROOT_DIR}/docker/scripts/restart_map_volume.sh ${map_name} "${VOLUME_VERSION}"
        done
    fi

    LOCALIZATION_VOLUME=apollo_localization_volume_$USER
    docker stop ${LOCALIZATION_VOLUME} > /dev/null 2>&1

    LOCALIZATION_VOLUME_IMAGE=${DOCKER_REPO}:localization_volume-${TARGET_ARCH}-latest
    docker_pull ${LOCALIZATION_VOLUME_IMAGE}
    docker run -it -d --rm --name ${LOCALIZATION_VOLUME} ${LOCALIZATION_VOLUME_IMAGE}

    LOCAL_THIRD_PARTY_VOLUME=apollo_local_third_party_volume_$USER
    docker stop ${LOCAL_THIRD_PARTY_VOLUME} > /dev/null 2>&1

    LOCAL_THIRD_PARTY_VOLUME_IMAGE=${DOCKER_REPO}:local_third_party_volume-${TARGET_ARCH}-latest
    docker_pull ${LOCAL_THIRD_PARTY_VOLUME_IMAGE}
    docker run -it -d --rm --name ${LOCAL_THIRD_PARTY_VOLUME} ${LOCAL_THIRD_PARTY_VOLUME_IMAGE}

    OTHER_VOLUME_CONF="${OTHER_VOLUME_CONF} --volumes-from ${LOCALIZATION_VOLUME} "
    OTHER_VOLUME_CONF="${OTHER_VOLUME_CONF} --volumes-from ${LOCAL_THIRD_PARTY_VOLUME}"

    determine_gpu_use_host
    info "Starting docker container \"${APOLLO_DEV}\" ..."

    local local_host="$(hostname)"
    local display="${DISPLAY:-:0}"
    local user="${USER}"
    local uid="$(id -u)"
    local group=$(id -g -n)
    local gid=$(id -g)

    set -x

    ${DOCKER_RUN} -itd  \
        --privileged    \
        --name "${APOLLO_DEV}"  \
        ${MAP_VOLUME_CONF}      \
        ${OTHER_VOLUME_CONF}    \
        -e DISPLAY="${display}"     \
        -e DOCKER_USER="${user}"    \
        -e USER="${user}"           \
        -e DOCKER_USER_ID="${uid}"  \
        -e DOCKER_GRP="${group}"    \
        -e DOCKER_GRP_ID="${gid}"   \
        -e DOCKER_IMG="${APOLLO_DEV_IMAGE}" \
        -e USE_GPU="${USE_GPU_HOST}"         \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
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
        /bin/bash

    if [ $? -ne 0 ];then
        error "Failed to start docker container \"${APOLLO_DEV}\" based on image: ${APOLLO_DEV_IMAGE}"
        exit 1
    fi
    set +x

    post_run_setup

    ok "Congratulations! You have successfully finished setting up Apollo dev docker environment." \
       "To login into the newly created ${APOLLO_DEV} container, please run the following command:"
    ok "  bash docker/scripts/dev_into.sh"
    ok "Enjoy!"
}

main "$@"
