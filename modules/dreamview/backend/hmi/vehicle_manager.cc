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

#include "modules/dreamview/backend/hmi/vehicle_manager.h"

#include "gflags/gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"

DEFINE_string(vehicle_param_pb_file, "vehicle_param.pb.txt",
              "Param proto path relative to the vehicle data path, which will "
              "be placed to FLAGS_vehicle_config_path.");

DEFINE_string(calibration_table_file, "calibration_table.pb.txt",
              "Calibration table path relative to the vehicle data path, which "
              "will be placed to FLAGS_control_conf_file.");

DEFINE_string(velodyne_launch_file, "start_velodyne.launch",
              "Velodyne launch file path relative to the vehicle data path.");
DEFINE_string(velodyne_launch_path,
              "<ros>/share/velodyne/launch/start_velodyne.launch",
              "Velodyne launch file path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");

DEFINE_string(velodyne_intrinsics_file,
              "velodyne_params/64E_S3_calibration_example.yaml",
              "Velodyne intrinsics path relative to the vehicle data path.");
DEFINE_string(velodyne_intrinsics_path,
              "<ros>/share/velodyne_pointcloud/params/"
              "64E_S3_calibration_example.yaml",
              "Velodyne intrinsic path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");

DEFINE_string(velodyne_extrinsics_file,
              "velodyne_params/velodyne64_novatel_extrinsics_example.yaml",
              "Velodyne extrinsics path relative to the vehicle data path.");
DEFINE_string(velodyne_extrinsics_path,
              "<ros>/share/velodyne_pointcloud/params/"
              "velodyne64_novatel_extrinsics_example.yaml",
              "Velodyne extrinsic path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");

DEFINE_string(gnss_launch_file, "gnss_params/gnss_driver.launch",
              "GNSS launch file path relative to the vehicle data path.");
DEFINE_string(gnss_launch_path,
              "<ros>/share/gnss_driver/launch/gnss_driver.launch",
              "GNSS launch file path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");

DEFINE_string(gnss_conf_file, "gnss_params/gnss_conf_mkz.txt",
              "GNSS config file path relative to the vehicle data path.");
DEFINE_string(gnss_conf_path, "<ros>/share/gnss_driver/conf/gnss_conf_mkz.txt",
              "GNSS config file path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");

namespace apollo {
namespace dreamview {

using apollo::common::util::CopyFile;
using apollo::common::util::StrCat;
using apollo::common::util::TranslatePath;

bool VehicleManager::UseVehicle(const std::string &vehicle_data_path) {
  if (!apollo::common::util::DirectoryExists(vehicle_data_path)) {
    AERROR << "Cannot find vehicle data: " << vehicle_data_path;
    return false;
  }

  // Copy vehicle_param_pb.
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_vehicle_param_pb_file),
           FLAGS_vehicle_config_path);
  // Copy calibration_table.
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_calibration_table_file),
           FLAGS_control_conf_file);
  // Copy velodyne_params.
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_velodyne_launch_file),
           TranslatePath(FLAGS_velodyne_launch_path));
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_velodyne_intrinsics_file),
           TranslatePath(FLAGS_velodyne_intrinsics_path));
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_velodyne_extrinsics_file),
           TranslatePath(FLAGS_velodyne_extrinsics_path));
  // Copy gnss_conf.
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_gnss_launch_file),
           TranslatePath(FLAGS_gnss_launch_path));
  CopyFile(StrCat(vehicle_data_path, "/", FLAGS_gnss_conf_file),
           TranslatePath(FLAGS_gnss_conf_path));

  return true;
}

}  // namespace dreamview
}  // namespace apollo
