#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

SRC_DIR=$1
TARGET_DIR=$2

set -e

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash

sudo mkdir -p ${TARGET_DIR}

if [ -z "$3" ]; then
    MAP_DIR="sunnyvale_with_two_offices"
else
    MAP_DIR=$3
fi

/apollo/bazel-bin/modules/planning/planning_base/pipeline/record_to_learning_data \
    --flagfile=/apollo/modules/planning/planning_component/conf/planning.conf \
    --map_dir=/apollo/modules/map/data/${MAP_DIR} \
    --planning_offline_learning=true \
    --planning_offline_bags=${SRC_DIR} \
    --planning_data_dir=${TARGET_DIR}
