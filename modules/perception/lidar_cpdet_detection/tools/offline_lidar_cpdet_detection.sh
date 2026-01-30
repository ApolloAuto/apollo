#!/usr/bin/env bash

##############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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

pcd_path="/apollo/data/pcd/"
output_path="/apollo/data/output/"

mkdir -p $output_path && rm -rf $output_path/*

/apollo/bazel-bin/modules/perception/lidar_cpdet_detection/tools/offline_lidar_cpdet_detection \
        --pcd_path=$pcd_path \
        --output_path=$output_path \
        --detector_name=CPDetection \
        --config_file=cnnseg16_param.pb.txt \
        2>&1 | tee /apollo/data/log/offline_lidar_cpdet_detection.log
