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
  LOG="/tmp/apollo_record.out"
  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    nohup rosbag record --split --duration=1m -b 2048  \
        /apollo/sensor/gnss/best_pose \
        /apollo/sensor/gnss/corrected_imu \
        /apollo/sensor/gnss/gnss_status \
        /apollo/sensor/gnss/imu \
        /apollo/sensor/gnss/ins_stat \
        /apollo/sensor/gnss/odometry \
        /apollo/sensor/gnss/rtk_eph \
        /apollo/sensor/gnss/rtk_obs \
        /apollo/sensor/velodyne64/compensator/PointCloud2 \
        /apollo/localization/pose \
        /apollo/localization/msf_gnss \
        /apollo/localization/msf_lidar \
        /apollo/localization/msf_status \
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
  echo "$0 [start] --portable-disk     Record bag to the largest portable disk."
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
