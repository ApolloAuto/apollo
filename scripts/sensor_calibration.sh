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

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${DIR}/apollo_base.sh"

function calibrate_camera_camera() {
  LOG="${APOLLO_ROOT_DIR}/data/log/camera_camera_calibrator.out"
  MODULE="camera_camera_calibrator"

  # check if the module has started
  NUM_PROCESSES="$(pgrep -f "${MODULE}" | grep -cv '^1$')"

  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    echo "Start to calibrate Camera-Camera extrinsics, Ctrl+C to exit."
    eval "${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      2>&1 | tee ${LOG}"

  fi
}

function calibrate_lidar_camera() {
  LOG="${APOLLO_ROOT_DIR}/data/log/camera_lidar_calibrator.out"
  MODULE="lidar_camera_calibrator"

  # check if the module has started
  NUM_PROCESSES="$(pgrep -f "${MODULE}" | grep -cv '^1$')"

  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    echo "Start to calibrate LiDAR-Camera extrinsics, Ctrl+C to exit."
    eval "${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      2>&1 | tee ${LOG}"
  fi
}

function calibrate_radar_camera() {
  LOG="${APOLLO_ROOT_DIR}/data/log/camera_radar_calibrator.out"
  MODULE="radar_camera_calibrator"

  # check if the module has started
  NUM_PROCESSES="$(pgrep -f "${MODULE}" | grep -cv '^1$')"

  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    echo "Start to calibrate Radar-Camera extrinsics, Ctrl+C to exit."
    eval "${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      2>&1 | tee ${LOG}"
  fi
}

function visualize_radar_lidar() {
  LOG="${APOLLO_ROOT_DIR}/data/log/radar_lidar_visualizer.out"
  MODULE="radar_lidar_visualizer"

  # check if the module has started
  NUM_PROCESSES="$(pgrep -f "${MODULE}" | grep -cv '^1$')"

  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    echo "Visualize Radar and LiDAR data, Ctrl+C to exit."
    eval "${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      2>&1 |tee ${LOG}"
  fi
}

function calibrate_imu_vehicle() {
  LOG="${APOLLO_ROOT_DIR}/data/log/imu_car_calibrator.out"
  MODULE="imu_car_calibrator"

  # check if the module has started
  NUM_PROCESSES="$(pgrep -f "${MODULE}" | grep -cv '^1$')"

  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    echo "Start to calibrate Imu-Vehicle extrinsics, Ctrl+C to exit."
    eval "${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/${MODULE} \
      --flagfile=${APOLLO_ROOT_DIR}/modules/calibration/${MODULE}/conf/${MODULE}.conf \
      2>&1 | tee ${LOG}"
  fi
}

case $1 in
  all)
    calibrate_camera_camera
    calibrate_lidar_camera
    calibrate_radar_camera
    visualize_radar_lidar
    calibrate_imu_vehicle
    ;;
  camera_camera)
    calibrate_camera_camera
    ;;
  lidar_camera)
    calibrate_lidar_camera
    ;;
  radar_camera)
    calibrate_radar_camera
    ;;
  visualize)
    visualize_radar_lidar
    ;;
  imu_vehicle)
    calibrate_imu_vehicle
    ;;
  *)
    calibrate_camera_camera
    calibrate_lidar_camera
    calibrate_radar_camera
    visualize_radar_lidar
    calibrate_imu_vehicle
    ;;
esac
