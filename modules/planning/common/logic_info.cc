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
 * @file logic_info.cc
 **/

#include "modules/planning/common/logic_info.h"

namespace apollo {
namespace planning {

const std::vector<LogicPoint> &LogicInfo::reference_line_1() const {
  return _reference_line_1;
}

const std::vector<LogicPoint> &LogicInfo::reference_line_2() const {
  return _reference_line_2;
}

std::vector<LogicPoint> *LogicInfo::mutable_reference_line_1() {
  return &_reference_line_1;
}

std::vector<LogicPoint> *LogicInfo::mutable_reference_line_2() {
  return &_reference_line_2;
}

}  // namespace planning
}  // namespace apollo
