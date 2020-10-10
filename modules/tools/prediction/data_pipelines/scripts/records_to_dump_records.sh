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

set -e

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash

if [ ! -z "$3" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$3
fi

if [ -z "$2" ]; then
    MAP_DIR="sunnyvale"
else
    MAP_DIR=$2
fi

/apollo/bazel-bin/modules/prediction/pipeline/records_to_offline_data \
    --flagfile=/apollo/modules/prediction/conf/prediction.conf \
    --map_dir=/apollo/modules/map/data/${MAP_DIR} \
    --prediction_offline_mode=6 \
    --prediction_offline_bags=${SRC_DIR} \
    --noenable_multi_thread \
    --noenable_async_draw_base_image \
    --enable_all_pedestrian_caution_in_front \
    --noenable_rank_caution_obstacles
