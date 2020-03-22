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


APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}")/../.." && pwd )"
# the machine type, currently support x86_64, aarch64
MACHINE_ARCH=$(uname -m)

source $APOLLO_ROOT_DIR/scripts/apollo_base.sh

echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern

VERSION="release-${MACHINE_ARCH}-v2.0.1"
if [[ $# == 1 ]];then
    VERSION=$1
fi
if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=apolloauto/apollo
fi
IMG=${DOCKER_REPO}:$VERSION

DATA_DIR="${HOME}/data"
if [ ! -e "${DATA_DIR}/log" ]; then
  mkdir -p "${DATA_DIR}/log"
fi

if [ ! -e "${DATA_DIR}/bag" ]; then
  mkdir -p "${DATA_DIR}/bag"
fi

if [ ! -e "${DATA_DIR}/core" ]; then
  mkdir -p "${DATA_DIR}/core"
fi

function main() {
    echo "Type 'y' or 'Y' to pull docker image from China mirror or any other key from US mirror."
    read -t 10 -n 1 INCHINA
    if [ "$INCHINA" == "y" ] || [ "$INCHINA" == "Y" ]; then
        docker pull "registry.docker-cn.com/${IMG}"
    else
        docker pull $IMG
    fi

    docker ps -a --format "{{.Names}}" | grep 'apollo_release' 1>/dev/null
    if [ $? == 0 ]; then
        docker stop apollo_release 1>/dev/null
        docker rm -f apollo_release 1>/dev/null
    fi

    setup_device

    local devices=" -v /dev:/dev"

    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi
    USER_ID=$(id -u)
    GRP=$(id -g -n)
    GRP_ID=$(id -g)
    LOCAL_HOST=`hostname`
    DOCKER_HOME="/home/$USER"
    if [ "$USER" == "root" ];then
        DOCKER_HOME="/root"
    fi
    if [ ! -d "$HOME/.cache" ];then
        mkdir "$HOME/.cache"
    fi

    DOCKER_CMD="nvidia-docker"
    if ! [ -x "$(command -v ${DOCKER_CMD})" ]; then
        DOCKER_CMD="docker"
    fi
    ${DOCKER_CMD} run -it \
        -d --privileged \
        --name apollo_release \
        --net host \
        -v /media:/media \
        -v ${HOME}/data:/apollo/data \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /etc/localtime:/etc/localtime:ro \
        -v $HOME/.cache:${DOCKER_HOME}/.cache \
        -w /apollo \
        -e DISPLAY=${display} \
        -e RELEASE_DOCKER=1 \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP=$GRP \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -e PYTHONPATH=/apollo/lib \
        ${devices} \
        --add-host in_release_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_release_docker \
        --shm-size 2G \
        $IMG
    if [ "${USER}" != "root" ]; then
      docker exec apollo_release bash -c "/apollo/scripts/docker_adduser.sh"
      docker exec apollo_release bash -c "chown -R ${USER}:${GRP} /apollo/data"
      docker exec apollo_release bash -c "chmod a+w /apollo"

      DATA_DIRS=("/apollo/modules/common/data"
                 "/apollo/modules/control/conf"
                 "/apollo/modules/localization/msf/params/gnss_params"
                 "/apollo/modules/localization/msf/params/velodyne_params"
                 "/apollo/modules/perception/data/params"
                 "/apollo/modules/tools/ota"
                 "/apollo/modules/drivers/gnss/conf")
      for DATA_DIR in "${DATA_DIRS[@]}"; do
        docker exec apollo_release bash -c \
            "mkdir -p '${DATA_DIR}'; chmod a+rw -R '${DATA_DIR}'"
      done
    fi
    docker exec apollo_release bash -c '/apollo/docker/scripts/container_setup.sh'
    docker exec -u ${USER} -it apollo_release "/apollo/scripts/bootstrap.sh"
}

main
