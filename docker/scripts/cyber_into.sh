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

DOCKER_USER="${USER}"
CYBER_CONTAINER="apollo_cyber_${USER}"

APOLLO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh" CYBER_ONLY

xhost +local:root 1>/dev/null 2>&1

ARCH="$(uname -m)"
if [ "${ARCH}" == "x86_64" ]; then
    docker exec \
        -u "${DOCKER_USER}" \
        -it "${CYBER_CONTAINER}" \
        /bin/bash
elif [ "${ARCH}" == "aarch64" ]; then
    warning "!!! For the first time after starting the Cyber RT container, please run the following two commands: !!!"
    warning "!!!   1) /apollo/scripts/docker_start_user.sh # with root or sudo permissions!!!"
    warning "!!!   2) su ${DOCKER_USER} !!!"
    warning "! To exit, please use 'ctrl+p ctrl+q' !"

    docker attach "${CYBER_CONTAINER}"

else
    echo "Unsupported architecture: ${ARCH}"
    exit 0
fi

xhost -local:root 1>/dev/null 2>&1
