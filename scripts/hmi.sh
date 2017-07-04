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

source "${DIR}/apollo_init.sh"

function start() {
    echo "Start roscore..."
    ROSCORELOG="${APOLLO_ROOT_DIR}/data/log/roscore.out"
    nohup roscore </dev/null >"${ROSCORELOG}" 2>&1 &

    echo "HMI ros node service running at localhost:8887"
    LOG="${APOLLO_ROOT_DIR}/data/log/hmi_ros_node_service.out"
    nohup ${APOLLO_BIN_PREFIX}/modules/hmi/ros_node/ros_node_service \
        --v=3 \
        --log_dir=${APOLLO_ROOT_DIR}/data/log \
        >${LOG} 2>&1 &

    echo "HMI running at http://localhost:8887"
    LOG="${APOLLO_ROOT_DIR}/data/log/hmi.out"
    nohup python modules/hmi/web/hmi_main.py \
        --conf=modules/hmi/conf/config.pb.txt >"${LOG}" 2>&1 &
}

function stop() {
    pkill -f hmi_main.py
    pkill -f ros_node_service
    pkill -f roscore
}

case $1 in
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
