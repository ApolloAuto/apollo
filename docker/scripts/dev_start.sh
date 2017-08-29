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
VERSION_X86_64="dev-x86_64-20170829_1405"
VERSION_AARCH64="dev-aarch64-20170712_1533"
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
    DOCKER_REPO=apolloauto/internal
fi

IMG=${DOCKER_REPO}:$VERSION
LOCAL_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

if [ ! -e "${LOCAL_DIR}/data/log" ]; then
    mkdir -p "${LOCAL_DIR}/data/log"
fi
if [ ! -e "${LOCAL_DIR}/data/bag" ]; then
    mkdir -p "${LOCAL_DIR}/data/bag"
fi
if [ ! -e "${LOCAL_DIR}/data/core" ]; then
    mkdir -p "${LOCAL_DIR}/data/core"
fi

source ${LOCAL_DIR}/scripts/apollo_base.sh

function find_device() {
    # ${1} = device pattern
    local device_list=$(find /dev -name "${1}")
    if [ -z "${device_list}" ]; then
        warning "Failed to find device with pattern \"${1}\" ..."
    else
        local devices=""
        for device in $(find /dev -name "${1}"); do
            ok "Found device: ${device}."
            devices="${devices} --device ${device}:${device}"
        done
        echo "${devices}"
    fi
}

function main(){
    #FIX ME: remove login when open source.
    docker login -u autoapollo -p baidu123
    docker pull $IMG
    
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

    # setup CAN device
    if [ ! -e /dev/can0 ]; then
        sudo mknod --mode=a+rw /dev/can0 c 52 0
    fi

    local devices=""
    devices="${devices} $(find_device ttyUSB*)"
    devices="${devices} $(find_device ttyS*)"
    devices="${devices} $(find_device can*)"
    devices="${devices} $(find_device ram*)"
    devices="${devices} $(find_device loop*)"
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
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $LOCAL_DIR:/apollo \
        -v /media:/media \
        -v $HOME/.cache:${DOCKER_HOME}/.cache \
        -v /etc/localtime:/etc/localtime:ro \
        --net host \
        -w /apollo \
        ${devices} \
        --add-host in_dev_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_dev_docker \
        --shm-size 512M \
        $IMG
    if [ "${USER}" != "root" ]; then
        docker exec apollo_dev bash -c '/apollo/scripts/docker_adduser.sh'
    fi
}

main
