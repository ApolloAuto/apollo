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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_LOCATION_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_LOCATION_EXPORTER_H

#include <memory>
#include <string>
#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class LocationExporter
 * @brief Export info about localziation in rosbag.
 */
class LocationExporter : public BaseExporter {
 public:
  typedef std::shared_ptr<LocationExporter> Ptr;
  typedef std::shared_ptr<LocationExporter const> ConstPtr;

  explicit LocationExporter(const std::string &loc_file_folder);
  ~LocationExporter();

  void GnssLocCallback(const rosbag::MessageInstance &msg);
  void LidarLocCallback(const rosbag::MessageInstance &msg);
  void FusionLocCallback(const rosbag::MessageInstance &msg);

 private:
  std::string gnss_loc_file_;
  std::string lidar_loc_file_;
  std::string fusion_loc_file_;

  FILE *gnss_loc_file_handle_;
  FILE *lidar_loc_file_handle_;
  FILE *fusion_loc_file_handle_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_LOCATION_EXPORTER_H
