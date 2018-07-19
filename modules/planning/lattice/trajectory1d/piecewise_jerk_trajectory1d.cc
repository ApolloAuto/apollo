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

void PiecewiseJerkTrajectory1d::AppendSegment(
    const double jerk, const double param) {
  CHECK(!segments_.empty());
  const auto& last_segment = segments_.back();
  double last_p = last_segment.GetEndState();
  double last_v = last_segment.GetEndVelocity();
  double last_a = last_segment.GetEndAcceleration();
  segments_.emplace_back(last_p, last_v, last_a, jerk, param);
}

void PiecewiseJerkTrajectory1d::AppendSegment(
    const double p1, const double v1, const double a1,
    const double param) {
  CHECK_GT(param, FLAGS_lattice_epsilon);
  const auto& last_segment = segments_.back();
  double last_p = last_segment.GetEndState();
  double last_v = last_segment.GetEndVelocity();
  double last_a = last_segment.GetEndAcceleration();
  double jerk = (a1 - last_a) / param;
  segments_.emplace_back(last_p, last_v, last_a, jerk, param);
}

}  // namespace planning
}  // namespace apollo
