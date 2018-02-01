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

# Task dir which contains bag data. E.g.:
# - 2018-01-26-13-56-21
#   |-- 2018-01-26-13-56-23_0.bag
#   `-- 2018-01-26-13-57-23_1.bag
TASK_DIR=$1

# Set a small pos_sample_min_distance to skip parking period.
python modules/data/warehouse/web_server/view_single_task.py \
    --pos_sample_duration=0 \
    --pos_sample_min_distance=0.1 \
    --task_dir="${TASK_DIR}"
