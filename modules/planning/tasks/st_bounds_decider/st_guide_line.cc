/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/st_bounds_decider/st_guide_line.h"

namespace apollo {
namespace planning {

constexpr double kSpeedGuideLineResolution = 0.1;

void STGuideLine::Init(double desired_v) {
  s0_ = 0.0;
  t0_ = 0.0;
  v0_ = desired_v;
}

void STGuideLine::Init(
    double desired_v,
    const std::vector<common::TrajectoryPoint> &speed_reference) {
  s0_ = 0.0;
  t0_ = 0.0;
  v0_ = desired_v;
  DiscretizedTrajectory discrete_speed_reference(speed_reference);
  double total_time = discrete_speed_reference.GetTemporalLength();
  guideline_speed_data_.clear();
  for (double t = 0; t <= total_time; t += kSpeedGuideLineResolution) {
    const common::TrajectoryPoint trajectory_point =
        discrete_speed_reference.Evaluate(t);
    guideline_speed_data_.AppendSpeedPoint(
        trajectory_point.path_point().s(), trajectory_point.relative_time(),
        trajectory_point.v(), trajectory_point.a(), trajectory_point.da());
  }
}

double STGuideLine::GetGuideSFromT(double t) {
  common::SpeedPoint speed_point;
  if (t < guideline_speed_data_.TotalTime() &&
      guideline_speed_data_.EvaluateByTime(t, &speed_point)) {
    s0_ = speed_point.s();
    t0_ = t;
    return speed_point.s();
  }
  return s0_ + (t - t0_) * v0_;
}

void STGuideLine::UpdateBlockingInfo(const double t, const double s_block,
                                     const bool is_lower_block) {
  if (is_lower_block) {
    if (GetGuideSFromT(t) < s_block) {
      s0_ = s_block;
      t0_ = t;
    }
  } else {
    if (GetGuideSFromT(t) > s_block) {
      s0_ = s_block;
      t0_ = t;
    }
  }
}

}  // namespace planning
}  // namespace apollo
