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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H

#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <string>
#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class PCDExporter
 * @brief Export pcd from rosbag.
 */
class PCDExporter : public BaseExporter {
 public:
  typedef std::shared_ptr<PCDExporter> Ptr;
  typedef std::shared_ptr<PCDExporter const> ConstPtr;

  explicit PCDExporter(const std::string &pcd_folder);
  ~PCDExporter();

  void CompensatedPcdCallback(const rosbag::MessageInstance &msg);

 private:
  void WritePcdFile(const std::string &filename,
                    const sensor_msgs::PointCloud2::ConstPtr &msg);

  std::string pcd_folder_;
  FILE *stamp_file_handle_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_PCD_EXPORTER_H
