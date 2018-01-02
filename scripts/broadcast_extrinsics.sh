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

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

EXTRINSICS_FILES=(
  "modules/perception/data/params/long_camera_extrinsics.yaml"
  "modules/perception/data/params/short_camera_extrinsics.yaml"
  "modules/perception/data/params/radar_extrinsics.yaml"
  "modules/perception/data/params/radar_front_extrinsics.yaml"
  "${ROS_ROOT}/../velodyne_pointcloud/params/velodyne64_novatel_extrinsics_example.yaml"
  "${ROS_ROOT}/../velodyne_pointcloud/params/velodyne16_novatel_extrinsics_example.yaml"
)

bash modules/tools/extrinsics_broadcaster/extrinsics_broadcaster.sh clean
for i in "${EXTRINSICS_FILES[@]}"
do
  bash modules/tools/extrinsics_broadcaster/extrinsics_broadcaster.sh publish "${i}"
done
