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
# Usage:
# sudo bash /apollo/modules/tools/prediction/mlp_train/scripts/generate_labels.sh <input_feature.bin>
#
# The input feature.X.bin will generate furture_status.label, cruise.label, junction.label

SRC_FILE=$1

set -e

source /apollo/scripts/apollo_base.sh
source /apollo/cyber/setup.bash

/apollo/bazel-bin/modules/tools/prediction/data_pipelines/data_preprocessing/merge_label_dicts ${SRC_FILE}
