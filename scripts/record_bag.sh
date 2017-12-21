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
  BAG_DIR="${APOLLO_ROOT_DIR}/data/bag"

  # Record bag to the largest portable-disk.
  if [ "$1" = "--portable-disk" ]; then
    LARGEST_DISK="$(df | grep "/media/${DOCKER_USER}" | sort -nr -k 4 | \
        awk '{print substr($0, index($0, $6))}')"
    if [ ! -z "${LARGEST_DISK}" ]; then
      REAL_BAG_DIR="${LARGEST_DISK}/data/bag"
      if [ ! -d "${REAL_BAG_DIR}" ]; then
        mkdir -p "${REAL_BAG_DIR}"
      fi
      BAG_DIR="${APOLLO_ROOT_DIR}/data/bag/portable"
      rm -fr "${BAG_DIR}"
      ln -s "${REAL_BAG_DIR}" "${BAG_DIR}"
    else
      echo "Cannot find portable disk."
      echo "Please make sure your container was started AFTER inserting the disk."
    fi
  fi

  # Create and enter into bag dir.
  if [ ! -e "${BAG_DIR}" ]; then
    mkdir -p "${BAG_DIR}"
  fi
  cd "${BAG_DIR}"
  echo "Recording bag to: $(pwd)"

  # Start recording.
  LOG="/tmp/apollo_record.out"
  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    nohup rosbag record --split --duration=10m -b 2048  \
        /apollo/sensor/conti_radar \
        /apollo/sensor/delphi_esr \
        /apollo/sensor/gnss/best_pose \
        /apollo/sensor/gnss/corrected_imu \
        /apollo/sensor/gnss/gnss_status \
        /apollo/sensor/gnss/imu \
        /apollo/sensor/gnss/ins_stat \
        /apollo/sensor/gnss/odometry \
        /apollo/sensor/gnss/rtk_eph \
        /apollo/sensor/gnss/rtk_obs \
        /apollo/sensor/mobileye \
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
        /apollo/localization/msf_gnss \
        /apollo/localization/msf_lidar \
        /apollo/drive_event \
        /tf \
        /tf_static \
        /apollo/monitor </dev/null >"${LOG}" 2>&1 &
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
