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
DEV_CONTAINER_PREFIX='apollo_dev_'
DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}"

function parse_arguments {
    local container_name=''

    while [ $# -gt 0 ]; do
        local opt="$1"
        shift
        case "${opt}" in
            -n | --name)
                container_name="$1"
                shift
                ;;

            --user)
                export CUSTOM_USER="$1"
                shift
                ;;
        esac
    done

    [[ ! -z "${container_name}" ]] && DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${container_name}"
    [[ ! -z "${CUSTOM_USER}" ]] && DOCKER_USER="${CUSTOM_USER}"
}

function restart_stopped_container {
    if docker ps -f status=exited -f name="${DEV_CONTAINER}" | grep "${DEV_CONTAINER}"; then
        docker start "${DEV_CONTAINER}"
    fi
}

xhost +local:root 1>/dev/null 2>&1

parse_arguments "$@"

restart_stopped_container

docker exec \
    -u "${DOCKER_USER}" \
    -e HISTFILE=/apollo/.dev_bash_hist \
    -it "${DEV_CONTAINER}" \
    /bin/bash

xhost -local:root 1>/dev/null 2>&1
