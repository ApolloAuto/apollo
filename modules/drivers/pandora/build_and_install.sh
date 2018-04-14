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

ROS_PATH=/home/tmp/ros
source ${ROS_PATH}/setup.bash

cd /tmp

catkin_make_isolated --install \
    --source /apollo/modules/drivers/pandora/pandora_driver \
    --install-space ${ROS_PATH} -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli

catkin_make_isolated --install \
    --source /apollo/modules/drivers/pandora/pandora_pointcloud \
    --install-space ${ROS_PATH} -DCMAKE_BUILD_TYPE=Release \
    --cmake-args --no-warn-unused-cli

echo "Build pandora dirver successfully."
