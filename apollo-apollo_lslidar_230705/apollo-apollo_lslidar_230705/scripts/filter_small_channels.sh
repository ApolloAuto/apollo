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
#
# Use after "apollo.sh build":
#    ./filter_small_channels.sh <input_record> <output_record>
#
# The output_record will only contain channels with small data.

INPUT_RECORD=$1
OUTPUT_RECORD=$2

source "$(dirname "${BASH_SOURCE[0]}")/apollo_base.sh"

mkdir -p "$(dirname "${OUTPUT_RECORD}")"

cyber_recorder split -f "${INPUT_RECORD}" -o "${OUTPUT_RECORD}" \
  -c "/apollo/canbus/chassis" \
  -c "/apollo/canbus/chassis_detail" \
  -c "/apollo/control" \
  -c "/apollo/control/pad" \
  -c "/apollo/drive_event" \
  -c "/apollo/guardian" \
  -c "/apollo/localization/pose" \
  -c "/apollo/localization/msf_gnss" \
  -c "/apollo/localization/msf_lidar" \
  -c "/apollo/localization/msf_status" \
  -c "/apollo/hmi/status" \
  -c "/apollo/monitor" \
  -c "/apollo/monitor/system_status" \
  -c "/apollo/navigation" \
  -c "/apollo/perception/obstacles" \
  -c "/apollo/perception/traffic_light" \
  -c "/apollo/planning" \
  -c "/apollo/prediction" \
  -c "/apollo/relative_map" \
  -c "/apollo/routing_request" \
  -c "/apollo/routing_response" \
  -c "/apollo/routing_response_history" \
  -c "/apollo/sensor/conti_radar" \
  -c "/apollo/sensor/delphi_esr" \
  -c "/apollo/sensor/gnss/best_pose" \
  -c "/apollo/sensor/gnss/corrected_imu" \
  -c "/apollo/sensor/gnss/gnss_status" \
  -c "/apollo/sensor/gnss/imu" \
  -c "/apollo/sensor/gnss/ins_stat" \
  -c "/apollo/sensor/gnss/odometry" \
  -c "/apollo/sensor/gnss/raw_data" \
  -c "/apollo/sensor/gnss/rtk_eph" \
  -c "/apollo/sensor/gnss/rtk_obs" \
  -c "/apollo/sensor/mobileye" \
  -c "/tf" \
  -c "/tf_static"
