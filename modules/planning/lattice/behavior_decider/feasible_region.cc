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

#include "glog/logging.h"
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

  double v = init_s[1];
  CHECK(v >= 0.0);

  t_at_zero_speed_ = v / max_deceleration;
  s_at_zero_speed_ = init_s[0] + v * v / (2.0 * max_deceleration);

  double delta_v = std::abs(v - speed_limit);
  if (v < speed_limit) {
    t_at_speed_limit_ = delta_v / max_acceleration;
    s_at_speed_limit_ =
        init_s_[0] + v * t_at_speed_limit_ +
        0.5 * max_acceleration * t_at_speed_limit_ * t_at_speed_limit_;
  } else {
    t_at_speed_limit_ = 0.0;
    s_at_speed_limit_ = init_s_[0];
  }
}

double FeasibleRegion::SUpper(const double t) const {
  CHECK(t >= 0.0);

  if (init_s_[1] < speed_limit_) {
    if (t < t_at_speed_limit_) {
      return init_s_[0] + init_s_[1] * t + 0.5 * max_acceleration * t * t;
    } else {
      return s_at_speed_limit_ + speed_limit_ * (t - t_at_speed_limit_);
    }
  } else {
    return init_s_[0] + init_s_[1] * t;
  }
}

double FeasibleRegion::SLower(const double t) const {
  if (t < t_at_zero_speed_) {
    return init_s_[0] + init_s_[1] * t - 0.5 * max_deceleration * t * t;
  }
  return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
  return std::max(init_s_[1],
      std::min(init_s_[1] + max_acceleration * t, speed_limit_));
}

double FeasibleRegion::VLower(const double t) const {
  return t < t_at_zero_speed_ ? init_s_[1] - max_deceleration * t : 0.0;
}

}  // namespace planning
}  // namespace apollo
