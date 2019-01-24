#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
# sudo bash /apollo/modules/tools/prediction/mlp_train/scripts/generate_feature_proto.sh <input_dir> <output_dir>
#
# The output feature.X.bin will be in Features proto.

SRC_DIR=$1
TARGET_DIR=$2

set -e

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash 

FLAGFILE=/apollo/modules/prediction/conf/prediction.conf
echo "--prediction_offline_mode" >> ${FLAGFILE}
echo "--prediction_offline_bags=${SRC_DIR}" >> ${FLAGFILE}
echo "--prediction_data_dir=${TARGET_DIR}" >> ${FLAGFILE}
echo "--junction_distance_threshold=30.0" >> ${FLAGFILE}
echo "--noenable_prioritize_obstacles" >> ${FLAGFILE}

sudo mkdir -p ${TARGET_DIR}
# sudo chown apollo:apollo ${TARGET_DIR}
cyber_launch start /apollo/modules/prediction/launch/prediction.launch
