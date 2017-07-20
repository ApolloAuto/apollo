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

/**
 * @file logic_info.h
 **/

#ifndef MODULES_PLANNING_COMMON_LOGIC_INFO_H
#define MODULES_PLANNING_COMMON_LOGIC_INFO_H

#include <vector>

#include "modules/planning/common/logic_point.h"

namespace apollo {
namespace planning {

class LogicInfo {
 public:
  LogicInfo() = default;

  const std::vector<LogicPoint> &reference_line_1() const;
  const std::vector<LogicPoint> &reference_line_2() const;

  std::vector<LogicPoint> *mutable_reference_line_1();
  std::vector<LogicPoint> *mutable_reference_line_2();

 private:
  std::vector<LogicPoint> _reference_line_1;
  std::vector<LogicPoint> _reference_line_2;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_LOGIC_INFO_H
