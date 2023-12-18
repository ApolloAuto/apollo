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

std::string ConfigUtil::GetFullPlanningClassName(
    const std::string& class_name) {
  static const std::string kNameSpace = "apollo::planning::";
  // If the class name already has namespace in it, do nothing.
  if (class_name.find("::") != std::string::npos) {
    return class_name;
  }
  return kNameSpace + class_name;
}

}  // namespace planning
}  // namespace apollo
