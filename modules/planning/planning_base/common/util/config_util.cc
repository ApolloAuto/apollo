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

#include "modules/planning/planning_base/common/util/config_util.h"

#include <ctype.h>

#include "cyber/common/environment.h"

namespace apollo {
namespace planning {

std::string ConfigUtil::TransformToPathName(const std::string& name) {
  std::string output = name;
  std::transform(name.begin(), name.end(), output.begin(), ::tolower);
  return output;
}

/// @brief ConfigUtil 类的一个静态成员函数，用于获取完整的规划器类名
/// @param class_name 
/// @return 
std::string ConfigUtil::GetFullPlanningClassName(
    const std::string& class_name) {
  // 静态常量字符串
  static const std::string kNameSpace = "apollo::planning::";
  // If the class name already has namespace in it, do nothing.
  // 如果 class_name 里面已经有 ::，说明它是完整的类名，则直接返回，不做任何修改
  if (class_name.find("::") != std::string::npos) {
    return class_name;
  }
  return kNameSpace + class_name;
}

}  // namespace planning
}  // namespace apollo
