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

CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

DOCKER_REPO="apolloauto/apollo"
DEV_CONTAINER_PREFIX='apollo_dev_'
DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}"
DEV_INSIDE="in-dev-docker"

CO_DEV_PATH=

SUPPORTED_ARCHS=(x86_64 aarch64)
TARGET_ARCH="$(uname -m)"

VERSION_X86_64="dev-x86_64-18.04-20240620_1444"
TESTING_VERSION_X86_64="dev-x86_64-18.04-testing-20210112_0008"

VERSION_AARCH64="dev-aarch64-20.04-20240626_1642"
USER_VERSION_OPT=

ROCM_DOCKER_REPO="rocmapollo/apollo"
VERSION_ROCM_X86_64="dev-x86_64-rocm-18.04-20221027_0916"
TESTING_VERSION_ROCM_X86_64="dev-x86_64-rocm-18.04-testing"

FAST_MODE="no"

GEOLOC=
TIMEZONE_CN=(
  "Time zone: Asia/Shanghai (CST, +0800)"
  "+0800"
)

USE_LOCAL_IMAGE=0
CUSTOM_DIST=
USER_AGREED="no"

VOLUME_VERSION="latest"
SHM_SIZE="2G"
USER_SPECIFIED_MAPS=
MAP_VOLUMES_CONF=
OTHER_VOLUMES_CONF=

CROSS_PLATFORM_FLAG=0

# Map
DEFAULT_MAPS=(
  sunnyvale
  sunnyvale_big_loop
  sunnyvale_loop
  sunnyvale_with_two_offices
  san_mateo
  apollo_virutal_map
)

DEFAULT_TEST_MAPS=(
  sunnyvale_big_loop
  sunnyvale_loop
)

function show_usage() {
  cat << EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help                    Display this help and exit
    -f, --fast                    Fast mode without pulling all map volumes
    -g, --geo <us|cn|none>        Pull docker image from geolocation specific registry mirror
    -l, --local                   Use local docker image
    -t, --tag <TAG>               Specify docker image with tag <TAG> to start
    -d, --dist                    Specify Apollo distribution(stable/testing)
    -c, --cross-platform <arch>   Run a cross-platform image
    --co-dev <path>               Run collaborative env between source image and package manager image
    --shm-size <bytes>            Size of /dev/shm, passed directly to "docker run"
    -y                            Agree to Apollo License Agreement non-interactively
    stop                          Stop all running Apollo containers
EOF
}

function cross_platform_setup() {
  info "Setup qemu user static..."
  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes > /dev/null
  if [[ ! $? -eq 0 ]]; then
    error "Qemu setup failed! Please report this issue to Apollo team."
    exit -1
  fi
}

function parse_arguments() {
  local custom_version=""
  local custom_dist=""
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

      -c | --cross-platform)
        custom_arch="$1"
        shift
        if [[ "$TARGET_ARCH" == "aarch64" && "$custom_arch" == "x86_64" ]]; then
          error "Run x86_64 image on aarch64 currently is not supported!"
          exit -1
        fi
        if [[ ! "$TARGET_ARCH" == "$custom_arch" ]]; then
          CROSS_PLATFORM_FLAG=1
        fi
        TARGET_ARCH="$custom_arch"
        check_target_arch
        cross_platform_setup
        ;;

      --co-dev)
        mount_workspace_path="$1"
        shift
        CO_DEV_PATH="$mount_workspace_path"
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

      --user)
        export CUSTOM_USER="$1"
        shift
        ;;

      --uid)
        export CUSTOM_UID="$1"
        shift
        ;;

      --group)
        export CUSTOM_GROUP="$1"
        shift
        ;;
      --gid)
        export CUSTOM_GID="$1"
        shift
        ;;

      -n | --name)
        DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${1}"
        shift
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
      -y)
        USER_AGREED="yes"
        ;;
      stop)
        info "Now, stop all Apollo containers created by ${USER} ..."
        stop_all_apollo_containers "-f"
        exit 0
        ;;
      *)
        warning "Unknown option: ${opt}"
        exit 2
        ;;
    esac
  done # End while loop

  [[ -n "${geo}" ]] && GEOLOC="${geo}"
  [[ -n "${fast_mode}" ]] && FAST_MODE="${fast_mode}"
  [[ -n "${custom_version}" ]] && USER_VERSION_OPT="${custom_version}"
  [[ -n "${custom_dist}" ]] && CUSTOM_DIST="${custom_dist}"
  [[ -n "${shm_size}" ]] && SHM_SIZE="${shm_size}"
}

function determine_dev_image() {
  local docker_repo="${DOCKER_REPO}"
  local version="$1"
  # If no custom version specified
  if [[ -z "${version}" ]]; then
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
      if [[ ${USE_AMD_GPU} == 1 ]]; then
        docker_repo="${ROCM_DOCKER_REPO}"
        version="${VERSION_ROCM_X86_64}"
      elif (($USE_NVIDIA_GPU == 1)) || (($USE_GPU_HOST == 0)); then
        if [[ "${CUSTOM_DIST}" == "testing" ]]; then
          version="${TESTING_VERSION_X86_64}"
        else
          version="${VERSION_X86_64}"
        fi
      fi
    elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
      version="${VERSION_AARCH64}"
    else
      error "Logic can't reach here! Please report this issue to Apollo@GitHub."
      exit 3
    fi
  fi
  DEV_IMAGE="${docker_repo}:${version}"
}

function check_host_environment() {
  if [[ "${HOST_OS}" != "Linux" ]]; then
    warning "Running Apollo dev container on ${HOST_OS} is UNTESTED, exiting..."
    exit 1
  fi
}

function check_target_arch() {
  local arch="${TARGET_ARCH}"
  local support_arch=""
  for ent in "${SUPPORTED_ARCHS[@]}"; do
    support_arch="$support_arch $ent"
    if [[ "${ent}" == "${TARGET_ARCH}" ]]; then
      return 0
    fi
  done
  error "Unsupported target architecture: ${TARGET_ARCH}."
  error "Current Apollo support architecture:$support_arch."
  exit 1
}

function check_timezone_cn() {
  # https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
  # time_zone=$(timedatectl | grep "Time zone" | xargs)
  # time_zone=$(date +"%z")

  # for tz in "${TIMEZONE_CN[@]}"; do
  #   if [[ "${time_zone}" == "${tz}" ]]; then
  #     GEOLOC="cn"
  #     return 0
  #   fi
  # done
  # disable docker.io image source
  GEOLOC="cn" 
}

function setup_devices_and_mount_local_volumes() {
  local __retval="$1"

  [ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

  source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"
  setup_device

  local volumes="-v $APOLLO_ROOT_DIR:/apollo"

  [ ! -z "${CO_DEV_PATH}" ] && volumes="-v ${PWD}/${CO_DEV_PATH}:/apollo_workspace ${volumes}"
  [ ! -z "${CO_DEV_PATH}" ] && volumes="-e CO_DEV=1 ${volumes}"

  [ -d "${APOLLO_CONFIG_HOME}" ] || mkdir -p "${APOLLO_CONFIG_HOME}"
  volumes="-v ${APOLLO_CONFIG_HOME}:${APOLLO_CONFIG_HOME} ${volumes}"

  local teleop="${APOLLO_ROOT_DIR}/../apollo-teleop"
  if [ -d "${teleop}" ]; then
    volumes="${volumes} -v ${teleop}:/apollo/modules/teleop ${volumes}"
  fi
  local apollo_tools="${APOLLO_ROOT_DIR}/../apollo-tools"
  if [ -d "${apollo_tools}" ]; then
    volumes="${volumes} -v ${apollo_tools}:/tools"
  fi

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
  docker volume rm "${volume}" > /dev/null 2>&1
  docker run -v "${volume}":"${path}" --rm "${image}" true > /dev/null
}

function restart_map_volume_if_needed() {
  local map_name="$1"
  local map_version="$2"
  local map_volume="apollo_map_volume-${map_name}_${USER}"
  local map_path="/apollo/modules/map/data/${map_name}"
  # some map image does not support aarch64, force to use image of x86_64
  local TARGET_ARCH="x86_64"

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
  local TARGET_ARCH="x86_64"

  # AUDIO
  local audio_volume="apollo_audio_volume_${USER}"
  local audio_image="${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest"
  local audio_path="/apollo/modules/audio/data/"
  docker_restart_volume "${audio_volume}" "${audio_image}" "${audio_path}"
  volume_conf="${volume_conf} --volume ${audio_volume}:${audio_path}"
  OTHER_VOLUMES_CONF="${volume_conf}"
}

function main() {
  check_host_environment
  check_target_arch

  parse_arguments "$@"

  if [[ "${USER_AGREED}" != "yes" ]]; then
    check_agreement
  fi

  info "Determine whether host GPU is available ..."
  determine_gpu_use_host
  info "USE_GPU_HOST: ${USE_GPU_HOST}"
  info "USE_AMD_GPU: ${USE_AMD_GPU}"
  info "USE_NVIDIA_GPU: ${USE_NVIDIA_GPU}"

  determine_dev_image "${USER_VERSION_OPT}"

  [[ -z "${GEOLOC}" ]] && check_timezone_cn
  geo_specific_config "${GEOLOC}"

  if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
    info "Start docker container based on local image : ${DEV_IMAGE}"
  fi

  if ! docker_pull "${DEV_IMAGE}"; then
    error "Failed to pull docker image ${DEV_IMAGE}"
    exit 1
  fi

  info "Remove existing Apollo Development container ..."
  remove_container_if_exists ${DEV_CONTAINER}

  local local_volumes=
  setup_devices_and_mount_local_volumes local_volumes

  mount_map_volumes
  mount_other_volumes

  warning "Starting in 9.0 we do not automatically download models!" \
    "If you want to run the perception module, download the model manually" \
    "reference https://github.com/ApolloAuto/apollo/discussions/15212"

  info "Starting Docker container \"${DEV_CONTAINER}\" ..."

  local local_host="$(hostname)"
  local display="${DISPLAY:-:0}"
  local user="${CUSTOM_USER-$USER}"
  local uid="${CUSTOM_UID-$(id -u)}"
  local group="${CUSTOM_GROUP-$(id -g -n)}"
  local gid="${CUSTOM_GID-$(id -g)}"

  local start_img="${DEV_IMAGE}"
  if [[ -n "${GEO_REGISTRY}" ]]; then
    start_img="${GEO_REGISTRY}/${DEV_IMAGE}"
  fi

  set -x

  ${DOCKER_RUN_CMD} -itd \
    --privileged \
    --name "${DEV_CONTAINER}" \
    --label "owner=${USER}" \
    -e CROSS_PLATFORM="${CROSS_PLATFORM_FLAG}" \
    -e DISPLAY="${display}" \
    -e DOCKER_USER="${user}" \
    -e USER="${user}" \
    -e DOCKER_USER_ID="${uid}" \
    -e DOCKER_GRP="${group}" \
    -e DOCKER_GRP_ID="${gid}" \
    -e DOCKER_IMG="${DEV_IMAGE}" \
    -e USE_GPU_HOST="${USE_GPU_HOST}" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
    ${MAP_VOLUMES_CONF} \
    ${OTHER_VOLUMES_CONF} \
    ${local_volumes} \
    --net host \
    -w /apollo \
    --add-host "${DEV_INSIDE}:127.0.0.1" \
    --add-host "${local_host}:127.0.0.1" \
    --hostname "${DEV_INSIDE}" \
    --shm-size "${SHM_SIZE}" \
    --pid=host \
    -v /dev/null:/dev/raw1394 \
    "${start_img}" \
    /bin/bash

  if [ $? -ne 0 ]; then
    error "Failed to start docker container \"${DEV_CONTAINER}\" based on image: ${start_img}"
    exit 1
  fi
  set +x

  postrun_start_user "${DEV_CONTAINER}"
  postrun_cross_platfrom_download "${DEV_CONTAINER}" "${CROSS_PLATFORM_FLAG}"

  ok "Congratulations! You have successfully finished setting up Apollo Dev Environment."
  ok "To login into the newly created ${DEV_CONTAINER} container, please run the following command:"
  ok "  bash docker/scripts/dev_into.sh"
  ok "Enjoy!"
}

main "$@"
