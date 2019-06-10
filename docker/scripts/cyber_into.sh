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

ARCH=$(uname -m)
CMD=""

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

source ${APOLLO_ROOT_DIR}/scripts/apollo_base.sh CYBER_ONLY

xhost +local:root 1>/dev/null 2>&1

if [ ${ARCH} == "x86_64" ]; then
    docker exec \
        -u $USER \
        -it apollo_cyber_$USER \
        /bin/bash
elif [ ${ARCH} == "aarch64" ]; then
    warning "!!! For the first time after starting the Cyber RT container, please run the following two commands: !!!"
    warning "!!!   1) /apollo/scripts/docker_adduser.sh !!!"
    warning "!!!   2) su $USER !!!"
    warning "! To exit, please use 'ctrl+p ctrl+q' !"

    docker attach apollo_cyber_$USER

else
    echo "Unknown architecture: ${ARCH}"
    exit 0
fi

xhost -local:root 1>/dev/null 2>&1
