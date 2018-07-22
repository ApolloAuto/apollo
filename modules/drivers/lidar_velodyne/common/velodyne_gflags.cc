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

#include "modules/drivers/lidar_velodyne/common/velodyne_gflags.h"

DEFINE_string(velodyne_module_name, "velodyne", "Velodyne driver module name");
DEFINE_string(velodyne_adapter_config_filename,
              "modules/drivers/lidar_velodyne/conf/adapter.conf",
              "The adapter config file");
DEFINE_string(velodyne_conf_file,
              "modules/drivers/lidar_velodyne/conf/velodyne_conf.pb.txt",
              "Velodyne conf file");

DEFINE_bool(publish_raw_data, true, "VELODYNE_RAW_DATA");
DEFINE_bool(publish_raw_pointcloud, true, "RAW_POINT_CLOUD");
DEFINE_bool(publish_compensator_pointcloud, true, "POINT_CLOUD");

DEFINE_bool(pipeline_mode, false, "exec point one by one");
