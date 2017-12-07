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

VERSION=""
ARCH=$(uname -m)
VERSION_X86_64="dev-x86_64-20171206_1627"
VERSION_AARCH64="dev-aarch64-20170927_1111"
if [[ $# == 1 ]];then
    VERSION=$1
elif [ ${ARCH} == "x86_64" ]; then
    VERSION=${VERSION_X86_64}
elif [ ${ARCH} == "aarch64" ]; then
    VERSION=${VERSION_AARCH64}
else
    echo "Unknown architecture: ${ARCH}"
    exit 0
fi

if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=apolloauto/apollo
fi

IMG=${DOCKER_REPO}:$VERSION
APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

if [ ! -e /apollo ]; then
    sudo ln -sf ${APOLLO_ROOT_DIR} /apollo
fi

echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern

source ${APOLLO_ROOT_DIR}/scripts/apollo_base.sh

function main(){
    echo "Type 'y' or 'Y' to pull docker image from China mirror or any other key from US mirror."
    read -t 10 -n 1 INCHINA
    if [ "$INCHINA" == "y" ] || [ "$INCHINA" == "Y" ]; then
        docker pull "registry.docker-cn.com/${IMG}"
    else
        docker pull $IMG
    fi

    docker ps -a --format "{{.Names}}" | grep 'apollo_dev' 1>/dev/null
    if [ $? == 0 ]; then
        docker stop apollo_dev 1>/dev/null
        docker rm -f apollo_dev 1>/dev/null
    fi
    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi

    setup_device

    local devices=""
    devices="${devices} $(find_device ttyUSB*)"
    devices="${devices} $(find_device ttyS*)"
    devices="${devices} $(find_device can*)"
    devices="${devices} $(find_device ram*)"
    devices="${devices} $(find_device loop*)"
    devices="${devices} $(find_device nvidia*)"
    devices="${devices} -v /dev/camera/obstacle:/dev/camera/obstacle "
    devices="${devices} -v /dev/camera/trafficlights:/dev/camera/trafficlights "
    devices="${devices} -v /dev/novatel0:/dev/novatel0"
    devices="${devices} -v /dev/novatel1:/dev/novatel1"
    devices="${devices} -v /dev/novatel2:/dev/novatel2"

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
    set -x
    docker run -it \
        -d \
        --privileged \
        --name apollo_dev \
        -e DISPLAY=$display \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP=$GRP \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $APOLLO_ROOT_DIR:/apollo \
        -v /media:/media \
        -v $HOME/.cache:${DOCKER_HOME}/.cache \
        -v /etc/localtime:/etc/localtime:ro \
-v /usr/src:/usr/src \
-v /lib/modules:/lib/modules \
        --net host \
        -w /apollo \
        ${devices} \
        --add-host in_dev_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_dev_docker \
        --shm-size 512M \
        $IMG \
        /bin/bash
    set +x
    if [ "${USER}" != "root" ]; then
        docker exec apollo_dev bash -c '/apollo/scripts/docker_adduser.sh'
    fi
}

main
