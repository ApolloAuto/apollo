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

#include "modules/planning/lattice/behavior_decider/feasible_region.h"

#include <cmath>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"

namespace apollo {
namespace planning {

FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s,
                               const double speed_limit) {
  Setup(init_s, speed_limit);
}

void FeasibleRegion::Setup(const std::array<double, 3>& init_s,
                           const double speed_limit) {
  init_s_ = init_s;
  speed_limit_ = speed_limit;
  // TODO(all) Check v should be positive
  double v = init_s[1];
  time_to_zero_speed_ = v / max_deceleration;
  s_to_zero_speed_ = init_s[0] + v * v / (2.0 * max_deceleration);
  double delta_v = std::abs(v - speed_limit);
  if (v < speed_limit) {
    time_to_speed_limit_ = delta_v / max_acceleration;
    s_to_speed_limit_ = init_s_[0] + v * time_to_speed_limit_ +
        0.5 * max_acceleration * time_to_speed_limit_ * time_to_speed_limit_;
  } else {
    time_to_speed_limit_ = delta_v / max_deceleration;
    s_to_speed_limit_ = init_s_[0] + v * time_to_speed_limit_ +
        0.5 * max_deceleration * time_to_speed_limit_ * time_to_speed_limit_;
  }
}

double FeasibleRegion::SUpper(const double t) {
  if (t <= time_to_speed_limit_) {
    if (init_s_[1] < speed_limit_) {
      return init_s_[0] + init_s_[1] * t + 0.5 * max_acceleration * t * t;
    }
    return init_s_[0] + init_s_[1] * t + 0.5 * max_deceleration * t * t;
  }
  return s_to_speed_limit_ + speed_limit_ * (t - time_to_speed_limit_);
}

double FeasibleRegion::SLower(const double t) {
  if (t <= time_to_zero_speed_) {
    return init_s_[0] + init_s_[1] * t + 0.5 * max_deceleration * t * t;
  }
  return s_to_zero_speed_;
}

double FeasibleRegion::VUpperAbsolute(const double t) {
  return init_s_[1] + max_acceleration * t;
}

double FeasibleRegion::VLowerAbsolute(const double t) {
  return t <= time_to_zero_speed_ ?
         init_s_[1] + max_deceleration * t : 0.0;
}

double FeasibleRegion::VUpperRelative(const double t) {
  return t <= time_to_speed_limit_ ?
         init_s_[1] + max_acceleration * t : speed_limit_;
}

double FeasibleRegion::VLowerRelative(const double t) {
  return t <= time_to_zero_speed_ ?
         init_s_[1] + max_deceleration * t : 0.0;
}

}  // namespace planning
}  // namespace apollo
