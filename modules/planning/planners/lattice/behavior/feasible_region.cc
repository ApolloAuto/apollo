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
 * @file
 **/

#include "modules/planning/planners/lattice/behavior/feasible_region.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s) {
  init_s_ = init_s;

  double v = init_s[1];
  CHECK_GE(v, 0.0);

  const double max_deceleration = -FLAGS_longitudinal_acceleration_lower_bound;
  t_at_zero_speed_ = v / max_deceleration;
  s_at_zero_speed_ = init_s[0] + v * v / (2.0 * max_deceleration);
}

double FeasibleRegion::SUpper(const double t) const {
  ACHECK(t >= 0.0);
  return init_s_[0] + init_s_[1] * t +
         0.5 * FLAGS_longitudinal_acceleration_upper_bound * t * t;
}

double FeasibleRegion::SLower(const double t) const {
  if (t < t_at_zero_speed_) {
    return init_s_[0] + init_s_[1] * t +
           0.5 * FLAGS_longitudinal_acceleration_lower_bound * t * t;
  }
  return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
  return init_s_[1] + FLAGS_longitudinal_acceleration_upper_bound * t;
}

double FeasibleRegion::VLower(const double t) const {
  return t < t_at_zero_speed_
             ? init_s_[1] + FLAGS_longitudinal_acceleration_lower_bound * t
             : 0.0;
}

double FeasibleRegion::TLower(const double s) const {
  ACHECK(s >= init_s_[0]);

  double delta_s = s - init_s_[0];
  double v = init_s_[1];
  double a = FLAGS_longitudinal_acceleration_upper_bound;
  double t = (std::sqrt(v * v + 2.0 * a * delta_s) - v) / a;
  return t;
}

}  // namespace planning
}  // namespace apollo
