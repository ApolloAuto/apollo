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


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start_driver() {
    LOG="${APOLLO_ROOT_DIR}/data/log/pandora.out"
    CMD="roslaunch pandora_driver pandora_driver.launch"
    NUM_PROCESSES="$(pgrep -c -f "pandora_node")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function start_compensator() {
    LOG="${APOLLO_ROOT_DIR}/data/log/pandora.out"
    CMD="roslaunch pandora_pointcloud compensator_node.launch"
    NUM_PROCESSES="$(pgrep -c -f "compensator_node")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
       eval "nohup ${CMD} </dev/null >${LOG} 2>&1 &"
    fi
}

function stop() {
    pkill -SIGKILL -f pandora_driver
    pkill -SIGKILL -f compensator_node
}

# run command_name module_name
function run() {
    case $1 in
        start)
            start_driver
            sleep 2
            start_compensator
            ;;
        stop)
            stop
            ;;
        *)
            start_driver
	    sleep 2
            start_compensator
            ;;
    esac
}

run "$1"
