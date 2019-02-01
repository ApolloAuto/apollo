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

INPUT_FILE=$1
OUTPUT_FILE=$2
FEATURE_TYPE=$3

set -e

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash 

FLAGFILE=/apollo/modules/prediction/conf/offline_prediction.conf

/apollo/bazel-bin/modules/prediction/feature_proto_file_to_model_features \
--flagfile=${FLAGFILE} \
--offline_feature_proto_file_name=${INPUT_FILE} \
--output_filename=${OUTPUT_FILE} \
--extract_feature_type=${FEATURE_TYPE}
