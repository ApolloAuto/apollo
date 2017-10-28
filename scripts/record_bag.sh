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

source "${DIR}/apollo_base.sh"

if [ ! -d "${APOLLO_ROOT_DIR}/data/bag" ]; then
    mkdir -p "${APOLLO_ROOT_DIR}/data/bag"
fi

cd "${APOLLO_ROOT_DIR}/data/bag"

function start() {
    LOG="/tmp/apollo_record.out"
    NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
        nohup rosbag record -b 2048  \
            /apollo/sensor/gnss/gnss_status \
            /apollo/sensor/gnss/odometry \
            /apollo/sensor/gnss/ins_stat \
            /apollo/sensor/gnss/corrected_imu \
            /apollo/sensor/mobileye \
            /apollo/sensor/delphi_esr \
            /apollo/canbus/chassis \
            /apollo/canbus/chassis_detail \
            /apollo/control \
            /apollo/control/pad \
            /apollo/perception/obstacles \
            /apollo/perception/traffic_light \
            /apollo/planning \
            /apollo/prediction \
            /apollo/routing_request \
            /apollo/routing_response \
            /apollo/localization/pose \
            /apollo/monitor </dev/null >"${LOG}" 2>&1 &
    fi
}

function stop() {
    pkill -SIGINT -f rosbag
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
