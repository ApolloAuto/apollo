#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
    nohup rosbag record --split --duration=1m -b 2048 \
        /apollo/sensor/camera/traffic/image_long \
        /apollo/sensor/camera/traffic/image_short \
        /apollo/sensor/gnss/best_pose \
        /apollo/sensor/gnss/imu \
        /apollo/sensor/gnss/odometry \
        /apollo/sensor/gnss/raw_data \
        /apollo/sensor/velodyne16/VelodyneScanUnified \
        /apollo/sensor/velodyne16/PointCloud2 \
        /apollo/sensor/velodyne16/compensator/PointCloud2 \
        /apollo/sensor/velodyne64/VelodyneScanUnified \
        /apollo/sensor/velodyne64/PointCloud2 \
        /apollo/sensor/velodyne64/compensator/PointCloud2 \
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
