/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/common_msgs/dreamview_msgs/hmi_config.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_mode.pb.h"

#include "modules/common/util/future.h"

namespace apollo {
namespace dreamview {
namespace util {

class HMIUtil {
 public:
  /**
   * @brief Load HMIConfig.
   * @param config_path: the hmi modes conf path.
   * @return HMIConfig.
   */
  static apollo::dreamview::HMIConfig LoadConfig(
      const std::string config_path = "");
  /**
   * @brief Load HMIMode.
   * @return HMIMode.
   */
  static apollo::dreamview::HMIMode LoadMode(
      const std::string& mode_config_path);
  /**
   * @brief transfer the mode's cyber modules to modules.
   * @param mode: the mode to be transfered.
   */
  static void TranslateCyberModules(HMIMode* mode);
  /**
   * @brief Convert a string to be title-like. E.g.: "hello_world" -> "Hello
   * World".
   * @param origin: the origin string to be convert
   * @return the string after convert
   */
  static std::string TitleCase(std::string_view origin);
  /**
   * @brief List all directory as a dict in a directory.
   * @param dir: the directory to be listed.
   * @return the listed result.
   */
  static google::protobuf::Map<std::string, std::string> ListDirAsDict(
      const std::string& dir);
};

}  // namespace util
}  // namespace dreamview
}  // namespace apollo
