/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/tool/data_generator/common/data_generator_gflags.h"

DEFINE_string(
    data_generator_adapter_config_filename,
    "/apollo/modules/perception/tool/data_generator/conf/adapter.conf",
    "The data generator adapter config filename.");

DEFINE_string(
    data_generator_config_file,
    "/apollo/modules/perception/tool/data_generator/conf/config.pb.txt",
    "The data generator config file.");

DEFINE_string(data_file_prefix,
              "/apollo/modules/perception/tool/data_generator/data/",
              "The collected data file location.");
DEFINE_string(data_file_name, "sensor_data", "The data file name.");

DEFINE_string(novatel_frame_name, "novatel", "The frame name for novatel.");
DEFINE_string(velodyne64_frame_name, "velodyne64",
              "The frame name for velodyne64.");
