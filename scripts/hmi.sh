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

function start_now() {
    ROS_PROCESS="$(pgrep -c -f roscore)"
    if [ "${ROS_PROCESS}" -eq 0 ]; then
        echo "Start roscore..."
        ROSCORELOG="${APOLLO_ROOT_DIR}/data/log/roscore.out"
        nohup roscore </dev/null >"${ROSCORELOG}" 2>&1 &
    else
        echo "roscore is already in running..."
    fi

    echo "Start HMI ros bridge..."
    LOG="${APOLLO_ROOT_DIR}/data/log/hmi_ros_bridge.out"
    nohup ${APOLLO_BIN_PREFIX}/modules/hmi/ros_bridge/ros_bridge \
        --flagfile=modules/common/data/global_flagfile.txt \
        --v=3 \
        --log_dir=${APOLLO_ROOT_DIR}/data/log \
        >${LOG} 2>&1 &

    LOG="${APOLLO_ROOT_DIR}/data/log/hmi.out"
    nohup python modules/hmi/web/hmi_main.py >"${LOG}" 2>&1 &

    sleep 1
    HMI_PROCESS="$(pgrep -c -f modules/hmi/web/hmi_main.py)"
    if [ "${HMI_PROCESS}" -eq 1 ]; then
        echo "HMI is running at http://localhost:8887"
    else
        echo "Failed to start HMI."
        cat "${LOG}"
    fi
}

function start() {
    echo "****************************************"
    echo "* We have integrated HMI with Dreamview."
    echo "* This entrypoint is going to be retired soon."
    echo "* Please use scripts/bootstrap.sh to start the system."
    echo "* Or wait for 3s to continue..."
    echo "****************************************"
    echo ""
    sleep 3

    start_now
}

function stop() {
    pkill -f -9 hmi_main.py
    pkill -f ros_bridge
    pkill -f roscore
}

case $1 in
  start_now)
    start_now
    ;;
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    start
    ;;
esac
