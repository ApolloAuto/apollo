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

if [ $# -lt 1 ]; then
  echo "Filter rosbag and keep PnC topics to reduce bag size."
  echo "Usage: $0 rosbag_name "
  exit
fi

source "${DIR}/apollo_base.sh"

INPUT_BAG=$1
FN_NAME=`basename $INPUT_BAG`
DIR_NAME=`dirname $INPUT_BAG`
OUTPUT_BAG="${DIR_NAME}/pnc_${FN_NAME}"

rosbag filter $INPUT_BAG $OUTPUT_BAG \
  "topic == '/apollo/sensor/gnss/gnss_status' or \
  topic == '/apollo/sensor/gnss/odometry' or  \
  topic == '/apollo/sensor/gnss/ins_stat' or  \
  topic == '/apollo/sensor/gnss/corrected_imu' or  \
  topic == '/apollo/sensor/mobileye' or  \
  topic == '/apollo/sensor/delphi_esr' or  \
  topic == '/apollo/canbus/chassis' or  \
  topic == '/apollo/canbus/chassis_detail' or  \
  topic == '/apollo/control' or  \
  topic == '/apollo/control/pad' or  \
  topic == '/apollo/perception/obstacles' or  \
  topic == '/apollo/perception/traffic_light' or  \
  topic == '/apollo/planning' or  \
  topic == '/apollo/prediction' or  \
  topic == '/apollo/routing_request' or  \
  topic == '/apollo/routing_response' or  \
  topic == '/apollo/localization/pose' or  \
  topic == '/apollo/monitor' "

echo "output bag at $OUTPUT_BAG"
