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

INCHINA="no"
LOCAL_IMAGE="no"
VERSION=""
ARCH=$(uname -m)
VERSION_X86_64="dev-x86_64-20180419_1510"
VERSION_AARCH64="dev-aarch64-20170927_1111"
VERSION_OPT=""

function show_usage()
{
cat <<EOF
Usage: $(basename $0) [options] ...
OPTIONS:
    -C                     Pull docker image from China mirror.
    -h, --help             Display this help and exit.
    -t, --tag <version>    Specify which version of a docker image to pull.
    -l, --local            Use local docker image.
EOF
exit 0
}

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

if [ ! -e /apollo ]; then
    sudo ln -sf ${APOLLO_ROOT_DIR} /apollo
fi

if [ -e /proc/sys/kernel ]; then
    echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern > /dev/null
fi

source ${APOLLO_ROOT_DIR}/scripts/apollo_base.sh

VOLUME_VERSION="latest"
DEFAULT_MAPS=(
  sunnyvale_big_loop
  sunnyvale_loop
)
MAP_VOLUME_CONF=""

while [ $# -gt 0 ]
do
    case "$1" in
    -C|--docker-cn-mirror)
        INCHINA="yes"
        ;;
    -image)
        echo -e "\033[093mWarning\033[0m: This option has been replaced by \"-t\" and \"--tag\", please use the new one.\n"
        show_usage
        ;;
    -t|--tag)
        VAR=$1
        [ -z $VERSION_OPT ] || echo -e "\033[093mWarning\033[0m: mixed option $VAR with $VERSION_OPT, only the last one will take effect.\n"
        shift
        VERSION_OPT=$1
        [ -z ${VERSION_OPT// /} ] && echo -e "Missing parameter for $VAR" && exit 2
        [[ $VERSION_OPT =~ ^-.* ]] && echo -e "Missing parameter for $VAR" && exit 2
        ;;
    dev-*) # keep backward compatibility, should be removed from further version.
        [ -z $VERSION_OPT ] || echo -e "\033[093mWarning\033[0m: mixed option $1 with -t/--tag, only the last one will take effect.\n"
        VERSION_OPT=$1
        echo -e "\033[93mWarning\033[0m: You are using an old style command line option which may be removed from"
        echo -e "further versoin, please use -t <version> instead.\n"
        ;;
    -h|--help)
        show_usage
        ;;
    -l|--local)
        LOCAL_IMAGE="yes"
        ;;
    --map)
        map_name=$2
        shift
        source ${APOLLO_ROOT_DIR}/docker/scripts/restart_map_volume.sh \
            "${map_name}" "${VOLUME_VERSION}"
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

if [ "$INCHINA" == "yes" ]; then
    DOCKER_REPO=registry.docker-cn.com/apolloauto/apollo
fi

if [ "$LOCAL_IMAGE" == "yes" ] && [ -z "$VERSION_OPT" ]; then
    VERSION="local_dev"
fi

# Included default maps.
for map_name in ${DEFAULT_MAPS[@]}; do
    source ${APOLLO_ROOT_DIR}/docker/scripts/restart_map_volume.sh ${map_name} "${VOLUME_VERSION}"
done

IMG=${DOCKER_REPO}:$VERSION

function local_volumes() {
    # Apollo root and bazel cache dirs are required.
    volumes="-v $APOLLO_ROOT_DIR:/apollo \
             -v $HOME/.cache:${DOCKER_HOME}/.cache"
    case "$(uname -s)" in
        Linux)
            volumes="${volumes} -v /dev:/dev \
                                -v /media:/media \
                                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                                -v /etc/localtime:/etc/localtime:ro \
                                -v /usr/src:/usr/src \
                                -v /lib/modules:/lib/modules"
            ;;
        Darwin)
            # MacOS has strict limitations on mapping volumes.
            chmod -R a+wr ~/.cache/bazel
            ;;
    esac
    echo "${volumes}"
}

function main(){

    if [ "$LOCAL_IMAGE" = "yes" ];then
        info "Start docker container based on local image : $IMG"
    else
        info "Start pulling docker image $IMG ..."
        docker pull $IMG
        if [ $? -ne 0 ];then
            error "Failed to pull docker image."
            exit 1
        fi
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

    LOCALIZATION_VOLUME=apollo_localization_volume
    docker stop ${LOCALIZATION_VOLUME} > /dev/null 2>&1

    LOCALIZATION_VOLUME_IMAGE=${DOCKER_REPO}:localization_volume-${ARCH}-latest
    docker pull ${LOCALIZATION_VOLUME_IMAGE}
    docker run -it -d --rm --name ${LOCALIZATION_VOLUME} ${LOCALIZATION_VOLUME_IMAGE}

    YOLO3D_VOLUME=apollo_yolo3d_volume
    docker stop ${YOLO3D_VOLUME} > /dev/null 2>&1

    YOLO3D_VOLUME_IMAGE=${DOCKER_REPO}:yolo3d_volume-${ARCH}-latest
    docker pull ${YOLO3D_VOLUME_IMAGE}
    docker run -it -d --rm --name ${YOLO3D_VOLUME} ${YOLO3D_VOLUME_IMAGE}

    info "Starting docker container \"apollo_dev\" ..."
    docker run -it \
        -d \
        --privileged \
        --name apollo_dev \
        ${MAP_VOLUME_CONF} \
        --volumes-from ${LOCALIZATION_VOLUME} \
        --volumes-from ${YOLO3D_VOLUME} \
        -e DISPLAY=$display \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP=$GRP \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        $(local_volumes) \
        --net host \
        -w /apollo \
        --add-host in_dev_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_dev_docker \
        --shm-size 2G \
        $IMG \
        /bin/bash

    if [ $? -ne 0 ];then
        error "Failed to start docker container \"apollo_dev\" based on image: $IMG"
        exit 1
    fi

    if [ "${USER}" != "root" ]; then
        docker exec apollo_dev bash -c '/apollo/scripts/docker_adduser.sh'
    fi

    ok "Finished setting up Apollo docker environment. Now you can enter with: \nbash docker/scripts/dev_into.sh"
    ok "Enjoy!"
}

main
