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

source "$(dirname "${BASH_SOURCE[0]}")/apollo_base.sh"

if [ $# -lt 1 ]; then
  echo "$0 record_file"
  exit
fi

cyber_recorder play \
  -c /apollo/perception/obstacles \
  -c /apollo/control \
  -c /apollo/canbus/chassis \
  -c /apollo/localization/pose \
  -c /apollo/routing_request \
  -c /apollo/routing_response \
  -c /apollo/prediction \
  -c /apollo/planning \
  -c /apollo/canbus/chassis \
  -c /apollo/guardian \
  -c /apollo/perception/traffic_light \
  -c /apollo/monitor/system_status \
  -c /tf_static \
  -c /apollo/control/pad \
  -c /apollo/drive_event \
  -c /apollo/monitor \
  -c /tf \
  -c /apollo/sensor/gnss/best_pose \
  -f $*
