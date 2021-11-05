#!/usr/bin/env bash

###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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

OBSTACLE_X=$1
OBSTACLE_Y=$2
OBSTACLE_PHI=$3
TARGET_FILE=$4

set -e

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash

if [ -z "$5" ]; then
    MAP_DIR="sunnyvale"
else
    MAP_DIR=$5
fi

/apollo/bazel-bin/modules/prediction/pipeline/vector_net_feature \
    --map_dir=/apollo/modules/map/data/${MAP_DIR} \
    --prediction_target_file=${TARGET_FILE} \
    --obstacle_x=${OBSTACLE_X} \
    --obstacle_y=${OBSTACLE_Y} \
    --obstacle_phi=${OBSTACLE_PHI}
