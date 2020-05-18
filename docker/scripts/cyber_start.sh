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
APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

INCHINA="no"
LOCAL_IMAGE="no"
VERSION=""
ARCH=$(uname -m)
VERSION_X86_64="cyber-x86_64-18.04-20200518_0934"
VERSION_AARCH64="cyber-aarch64-18.04-20190621_1606"
VERSION_OPT=""

# Check whether user has agreed license agreement
function check_agreement() {
  local agreement_record="${HOME}/.apollo_agreement.txt"
  if [ -e "$agreement_record" ]; then
    return
  fi

  AGREEMENT_FILE="$APOLLO_ROOT_DIR/scripts/AGREEMENT.txt"
  if [ ! -e "$AGREEMENT_FILE" ]; then
    error "AGREEMENT $AGREEMENT_FILE does not exist."
    exit 1
  fi

  cat "${AGREEMENT_FILE}"
  local tip="Type 'y' or 'Y' to agree to the license agreement above, or type any other key to exit"
  echo "${tip}"
  read -n 1 user_agreed
  if [ "$user_agreed" == "y" ] || [ "$user_agreed" == "Y" ]; then
    cp $AGREEMENT_FILE $agreement_record
    echo "$tip" >> $agreement_record
    echo "$user_agreed" >> $agreement_record
  else
    exit 1
  fi
}

function check_host_environment() {
  echo 'Host environment checking done.'
}

function show_usage()
{
cat <<EOF
Usage: $(basename $0) [options] ...
OPTIONS:
    -C                     Pull docker image from China mirror.
    -h, --help             Display this help and exit.
    -t, --tag <version>    Specify which version of a docker image to pull.
    -l, --local            Use local docker image.
    stop                   Stop all running Apollo containers.
EOF
exit 0
}

function stop_containers()
{
running_containers=$(docker ps --format "{{.Names}}")

for i in ${running_containers[*]}
do
  if [[ "$i" =~ apollo_* ]];then
    printf %-*s 70 "stopping container: $i ..."
    if docker stop "$i" >/dev/null ; then
      printf "\033[32m[DONE]\033[0m\n"
    else
      printf "\033[31m[FAILED]\033[0m\n"
    fi
  fi
done
}


if [ ! -e /apollo ]; then
    sudo ln -sf "${APOLLO_ROOT_DIR}" /apollo
fi

if [ -e /proc/sys/kernel ]; then
    echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern > /dev/null
fi

source ${APOLLO_ROOT_DIR}/scripts/apollo_base.sh CYBER_ONLY
check_agreement
check_host_environment

while [ $# -gt 0 ]
do
    case "$1" in
    -C|--docker-cn-mirror)
        INCHINA="yes"
        ;;
    -t|--tag)
        VAR=$1
        [ -z $VERSION_OPT ] || echo -e "\033[093mWarning\033[0m: mixed option $VAR with $VERSION_OPT, only the last one will take effect.\n"
        shift
        VERSION_OPT=$1
        [ -z "${VERSION_OPT// /}" ] && echo -e "Missing parameter for $VAR" && exit 2
        [[ $VERSION_OPT =~ ^-.* ]] && echo -e "Missing parameter for $VAR" && exit 2
        ;;
    -h|--help)
        show_usage
        ;;
    -l|--local)
        LOCAL_IMAGE="yes"
        ;;
    stop)
	stop_containers
	exit 0
	;;
    *)
        echo -e "\033[93mWarning\033[0m: Unknown option: $1"
        exit 2
        ;;
    esac
    shift
done

if [ ! -z "$VERSION_OPT" ]; then
    VERSION=$VERSION_OPT
elif [ "${ARCH}" == "x86_64" ]; then
    VERSION=${VERSION_X86_64}
elif [ "${ARCH}" == "aarch64" ]; then
    VERSION=${VERSION_AARCH64}
else
    echo "Unknown architecture: ${ARCH}"
    exit 0
fi

if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=apolloauto/apollo
fi

if [ "$INCHINA" == "yes" ]; then
    DOCKER_REPO=registry.docker-cn.com/apolloauto/apollo
fi

if [ "$LOCAL_IMAGE" == "yes" ] && [ -z "$VERSION_OPT" ]; then
    VERSION="local_cyber_dev"
fi


IMG=${DOCKER_REPO}:$VERSION

function local_volumes() {
    # Apollo root and bazel cache dirs are required.
    volumes="-v $APOLLO_ROOT_DIR:/apollo"
    case "$(uname -s)" in
        Linux)
            case "$(lsb_release -r | cut -f2)" in
                14.04)
                    volumes="${volumes} "
                    ;;
                *)
                    volumes="${volumes} -v /dev:/dev "
                    ;;
            esac
            volumes="${volumes} -v /media:/media \
                                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                                -v /etc/localtime:/etc/localtime:ro \
                                -v /usr/src:/usr/src \
                                -v /lib/modules:/lib/modules"
            ;;
        Darwin)
            ;;
    esac
    echo "${volumes}"
}


DOCKER_RUN="docker run"
USE_GPU=0

function determine_gpu_use() {
  # Check nvidia-driver and GPU device
  local nv_driver="nvidia-smi"
  if [ ! -x "$(command -v ${nv_driver} )" ]; then
    warning "No nvidia-driver found. CPU will be used"
  elif [ -z "$(eval ${nv_driver} )" ]; then
    warning "No GPU device found. CPU will be used."
  else
    USE_GPU=1
  fi

  # Try to use GPU inside container
  local nv_docker_doc="https://github.com/NVIDIA/nvidia-docker/blob/master/README.md"
  if [ ${USE_GPU} -eq 1 ]; then
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
        USE_GPU=0
      fi
    else
      USE_GPU=0
      warning "Cannot access GPU from within container. Please install latest docker " \
              "and nvidia-container-toolkit as described by: "
      warning "  ${nv_docker_doc}"
    fi
  fi
}

function main(){

    if [ "$LOCAL_IMAGE" = "yes" ];then
        info "Start docker container based on local image : $IMG"
    else
        info "Start pulling docker image $IMG ..."
        docker pull "${IMG}"
        if [ $? -ne 0 ];then
            error "Failed to pull docker image."
            exit 1
        fi
    fi

    APOLLO_CYBER="apollo_cyber_${USER}"
    docker ps -a --format "{{.Names}}" | grep "$APOLLO_CYBER" 1>/dev/null
    if [ $? == 0 ]; then
        docker stop ${APOLLO_CYBER} 1>/dev/null
        docker rm -v -f ${APOLLO_CYBER} 1>/dev/null
    fi

    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi

    setup_device

    USER_ID=$(id -u)
    GRP=$(id -g -n)
    GRP_ID=$(id -g)
    LOCAL_HOST=$(hostname)
    if [ ! -d "${CACHE_ROOT_DIR}" ]; then
        mkdir "${CACHE_ROOT_DIR}"
    fi

    info "Starting docker container \"${APOLLO_CYBER}\" ..."

    determine_gpu_use

    info "DockerRun: ${DOCKER_RUN}"

    ${DOCKER_RUN} -it \
        -d \
        --privileged \
        --name $APOLLO_CYBER \
        -e DISPLAY=$display \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP="$GRP" \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -e USE_GPU=$USE_GPU \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        -e OMP_NUM_THREADS=1 \
        $(local_volumes) \
        --net host \
        -w /apollo \
        --add-host in_cyber_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_cyber_docker \
        --shm-size 2G \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        $IMG \
        /bin/bash

    if [ $? -ne 0 ];then
        error "Failed to start docker container \"${APOLLO_CYBER}\" based on image: $IMG"
        exit 1
    fi


    if [ "${ARCH}" == "x86_64" ]; then
        # User with uid=1000 or username=apollo excluded
        if [[ "${USER}" != "root" ]] && [[ "${USER}" != "apollo" ]] \
            && [[ $USER_ID -ne 1000 ]]; then
            docker exec -u root "${APOLLO_CYBER}" bash -c '/apollo/scripts/docker_adduser.sh'
        fi
    else
        warning "!!! Due to the problem with 'docker exec' on Drive PX platform, please run '/apollo/scripts/docker_adduser.sh' for the first time when you get into the docker !!!"
    fi

    ok "Finished setting up Apollo docker environment. Now you can enter with: \nbash docker/scripts/cyber_into.sh"
    ok "Enjoy!"
}

main
