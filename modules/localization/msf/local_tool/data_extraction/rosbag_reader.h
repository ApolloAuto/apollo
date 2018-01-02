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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ROSBAG_READER_H
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ROSBAG_READER_H
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "modules/localization/msf/local_tool/data_extraction/base_exporter.h"

namespace apollo {
namespace localization {
namespace msf {

/**
 * @class RosbagReader
 * @brief Read messages from rosbag.
 */
class RosbagReader {
 public:
  RosbagReader();
  ~RosbagReader();

  void Subscribe(const std::string& topic,
                 BaseExporter::OnRosmsgCallback call_back,
                 BaseExporter::Ptr exporter);
  void Read(const std::string& file_name);

 private:
  std::vector<std::string> topics_;
  std::unordered_map<
      std::string, std::pair<BaseExporter::Ptr, BaseExporter::OnRosmsgCallback>>
      call_back_map_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_ROSBAG_READER_H
