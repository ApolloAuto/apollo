/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar_velodyne/tools/tools_gflags.h"

DEFINE_string(tools_module_name, "velodyne_tools", "Velodyne tool module name");
DEFINE_string(tools_adapter_config_filename,
              "modules/drivers/lidar_velodyne/tools/conf/adapter.conf",
              "The tools adapter config file");
DEFINE_string(tools_conf_file,
    "modules/drivers/lidar_velodyne/tools/conf/velodyne_tools_conf.pb.txt",
    "Velodyne tools conf file");

DEFINE_bool(open_pointcloud_dump, true, "pointcloud dump");
DEFINE_bool(open_pointcloud_convert, false, "raw data 2 pointcloud");
DEFINE_bool(open_pointcloud_compensate, false, "raw pointcloud compensate");
