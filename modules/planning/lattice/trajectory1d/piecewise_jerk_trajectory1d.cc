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

/**
 * @file
 **/

#include "modules/planning/lattice/trajectory1d/piecewise_jerk_trajectory1d.h"

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

PiecewiseJerkTrajectory1d::PiecewiseJerkTrajectory1d(
    const double p, const double v, const double a) {
  last_p_ = p;
  last_v_ = v;
  last_a_ = a;
  param_.push_back(0.0);
}

void PiecewiseJerkTrajectory1d::AppendSegment(
    const double jerk, const double param) {
  CHECK_GT(param, FLAGS_lattice_epsilon);

  param_.push_back(param_.back() + param);

  segments_.emplace_back(last_p_, last_v_, last_a_, jerk, param);

  last_p_ = segments_.back().end_position();

  last_v_ = segments_.back().end_velocity();

  last_a_ = segments_.back().end_acceleration();
}

double PiecewiseJerkTrajectory1d::Evaluate(const std::uint32_t order,
    const double param) const {
  CHECK(param >= 0.0 && param <= param_.back());

  auto it_lower = std::lower_bound(param_.begin(), param_.end(), param);

  if (it_lower == param_.begin()) {
    return segments_.front().Evaluate(order, param);
  } else {
    auto index = std::distance(param_.begin(), it_lower) - 1;
    return segments_[index].Evaluate(order, param - param_[index]);
  }
}

double PiecewiseJerkTrajectory1d::ParamLength() const {
  return param_.back();
}

std::string PiecewiseJerkTrajectory1d::ToString() const {
  return "";
}

/**
void PiecewiseJerkTrajectory1d::AppendSegment(
    const double p1, const double v1, const double a1,
    const double param) {
  CHECK_GT(param, FLAGS_lattice_epsilon);
  const auto& last_segment = segments_.back();
  double last_p = last_segment.end_position();
  double last_v = last_segment.end_velocity();
  double last_a = last_segment.end_acceleration();
  double jerk = (a1 - last_a) / param;
  segments_.emplace_back(last_p, last_v, last_a, jerk, param);
}
**/

}  // namespace planning
}  // namespace apollo
