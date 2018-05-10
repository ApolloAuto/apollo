#!/usr/bin/env python

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

parameters = {
    'mlp':{
        'train_data_rate': 0.8,
        'size_obstacle_feature': 22,
        'size_lane_sequence_feature': 40,
        'dim_input': 22 + 40,
        'dim_hidden_1': 30,
        'dim_hidden_2': 15,
        'dim_output': 1
    },

    'feature': {
        'threshold_label_time_delta': 1.0,
        'prediction_label_timeframe':3.0
    }
}

labels = {
    'go_false': 0,
    'go_true': 1,
    'cutin_false': -1,
    'cutin_true': 2
}
