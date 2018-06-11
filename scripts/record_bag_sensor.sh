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

function start() {
  decide_task_dir $@
  cd "${TASK_DIR}"

  # Start recording.
  record_bag_env_log
  LOG="/tmp/apollo_record.out"
  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    nohup rosbag record --split --duration=1m -b 2048  \
        /apollo/sensor/camera/traffic/image_short \
        /apollo/sensor/camera/traffic/image_long \
        /apollo/sensor/conti_radar \
        /apollo/sensor/delphi_esr \
        /apollo/sensor/gnss/best_pose \
        /apollo/sensor/gnss/corrected_imu \
        /apollo/sensor/gnss/gnss_status \
        /apollo/sensor/gnss/imu \
        /apollo/sensor/gnss/raw_data \
        /apollo/sensor/gnss/ins_stat \
        /apollo/sensor/gnss/odometry \
        /apollo/sensor/gnss/rtk_eph \
        /apollo/sensor/gnss/rtk_obs \
        /apollo/sensor/mobileye \
        /apollo/sensor/velodyne64/compensator/PointCloud2 \
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
        /apollo/drive_event \
        /tf \
        /tf_static \
        /apollo/monitor \
        /apollo/monitor/static_info </dev/null >"${LOG}" 2>&1 &
    fi
}

function stop() {
  pkill -SIGINT -f record
}

function help() {
  echo "Usage:"
  echo "$0 [start]                     Record bag to data/bag."
  echo "$0 stop                        Stop recording."
  echo "$0 help                        Show this help message."
}

case $1 in
  start)
    shift
    start $@
    ;;
  stop)
    shift
    stop $@
    ;;
  help)
    shift
    help $@
    ;;
  *)
    start $@
    ;;
esac
