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

# Pre-check.
DOCKER_MACHINE=apollo
if [ ! $(which docker-machine) ] || [ ! $(which docker) ] ||
        [ ! $(which virtualbox) ]; then
    echo "Make sure you have installed docker-machine, docker and virtualbox:"
    echo "    $ brew install docker docker-machine"
    echo "    $ brew cask install virtualbox"
    exit 1
elif [ ! "$(docker-machine ls --filter "name=apollo" | grep -v "^NAME")" ]; then
    echo "Make sure you have started a docker machine:"
    echo "    $ docker-machine create --driver virtualbox \\"
    echo "        --virtualbox-memory 4096 \\"
    echo "        --virtualbox-cpu-count 2 \\"
    echo "        ${DOCKER_MACHINE}"
    echo "Your should allocate at least 4GB memory!"
    exit 1
elif [ ! "$(docker-machine ls --filter "name=apollo" | grep Running)" ]; then
    echo "The docker machine is down. Restart..."
    docker-machine restart ${DOCKER_MACHINE}
fi

VERSION=""
ARCH=$(uname -m)
VERSION_X86_64="dev-x86_64-20171103_2113"
VERSION_AARCH64="dev-aarch64-20170927_1111"
if [[ $# == 1 ]]; then
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
    # Get docker environment variables.
    eval $(docker-machine env ${DOCKER_MACHINE})

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

    USER=apollo
    USER_ID=1000
    GRP=apollo
    GRP_ID=1000
    LOCAL_HOST=`hostname`
    DOCKER_HOME="/home/$USER"
    docker run -it \
        -d \
        --privileged \
        --name apollo_dev \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP=$GRP \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -v $APOLLO_ROOT_DIR:/apollo \
        -v /media:/media \
        -v /etc/localtime:/etc/localtime:ro \
        --net host \
        -w /apollo \
        --add-host in_dev_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_dev_docker \
        --shm-size 512M \
        $IMG \
        /bin/bash

    docker exec apollo_dev bash -c '/apollo/scripts/docker_adduser.sh'

    echo "########## Attention ##########"
    echo "Your container is running at $(docker-machine ip), while not"
    echo "localhost or 127.0.0.1. Use the IP to access tools such as Dreamview."
    echo "###############################"
}

main
