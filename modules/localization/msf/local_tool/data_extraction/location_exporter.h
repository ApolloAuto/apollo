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

#pragma once

#include <string>

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class LocationExporter
 * @brief Export info about localziation in rosbag.
 */
class LocationExporter {
 public:
  explicit LocationExporter(const std::string &loc_file_folder);
  ~LocationExporter();

  void GnssLocCallback(const std::string &msg);
  void LidarLocCallback(const std::string &msg);
  void FusionLocCallback(const std::string &msg);
  void OdometryLocCallback(const std::string &msg);

 private:
  std::string gnss_loc_file_;
  std::string lidar_loc_file_;
  std::string fusion_loc_file_;
  std::string odometry_loc_file_;

  FILE *gnss_loc_file_handle_;
  FILE *lidar_loc_file_handle_;
  FILE *fusion_loc_file_handle_;
  FILE *odometry_loc_file_handle_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
