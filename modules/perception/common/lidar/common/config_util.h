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
namespace perception {
namespace lidar {

class ConfigUtil {
 public:
  /**
   * @brief Given the class name of perception module, combine the namespace
   * "apollo::perception::lidar::" with it to create an instance of the class.
   *
   * @param class_name The class name without namespace.
   * @return The perception class name with namespace.
   */
  static std::string GetFullClassName(const std::string& class_name);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
