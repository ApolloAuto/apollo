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
 * @file trajectory1d_info.cpp
 **/
#include "modules/planning/lattice/trajectory1d/trajectory1d_info.h"

namespace apollo {
namespace planning {

Trajectory1dInfo::Trajectory1dInfo() {
  type_ = Type::QUINTIC_POLYNOMIAL;
}

void Trajectory1dInfo::set_type(const Type& type) {
  type_ = type;
}

std::shared_ptr<Trajectory1d> Trajectory1dInfo::create() const {
  std::shared_ptr<Trajectory1d> ptr_traj;

  switch(type_) {
  case Type::QUINTIC_POLYNOMIAL:
    break;
  case Type::QUARTIC_POLYNOMIAL:
    break;
  default:
    break;
  }
  return ptr_traj;
}

} // namespace planning
} // namespace apollo
