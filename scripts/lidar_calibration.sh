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

function pack() {
  if [ -f lidar_calib_data.tar ]; then
    rm lidar_calib_data.tar
  fi
  
  md5sum lidar_calib.bag > bag_md5
  tar -cvf lidar_calib_data.tar ./lidar_calib.bag ./bag_md5 > /dev/null
}

function start_record() {
  LOG="${APOLLO_ROOT_DIR}/data/log/lidar_calibration.out"
  MODULE="republish_msg"
  
  # check if the module has started
  NUM_PROCESSES="$(pgrep -c -f "modules/calibration/${MODULE}")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    eval "nohup ${APOLLO_BIN_PREFIX}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      --log_dir=${APOLLO_ROOT_DIR}/data/log $@ </dev/null >${LOG} 2>&1 &"
  fi

  sleep 1
  
  if [ -f lidar_calib.bag ]; then
    rm lidar_calib.bag
  fi

  # start to record lidar calibration data
  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
    if [ "${NUM_PROCESSES}" -eq 0 ]; then
      nohup rosbag record -b 2048 -O lidar_calib.bag  \
        /apollo/sensor/gnss/ins_stat \
        /apollo/sensor/velodyne64/VelodyneScanUnified \
        /apollo/calibration/relative_odometry \
        </dev/null >"${LOG}" 2>&1 &
    fi
}

function stop_record() {
  pkill -SIGINT -f republish_msg
  pkill -SIGINT -f rosbag

  sleep 1

  pack
}

function start_check_extrin() {
  LOG="${APOLLO_ROOT_DIR}/data/log/lidar_calibration.out"
  MODULE="lidar_ex_checker"
  
  # check if the module has started
  NUM_PROCESSES="$(pgrep -c -f "modules/calibration/${MODULE}")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    eval "${APOLLO_BIN_PREFIX}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      --log_dir=${APOLLO_ROOT_DIR}/data/log $@ 2>${LOG}"
    echo "Program started, Ctrl+C to exit."
  fi
}

function stop_check_extrin() {
  pkill -SIGINT -f lidar_ex_checker
}

case $1 in
  start_record)
    start_record
    ;;
  stop_record)
    stop_record
    ;;
  start_check_extrin)
    start_check_extrin
    ;;
  stop_check_extrin)
    stop_check_extrin
    ;;
  *)
    record_start
    ;;
esac
