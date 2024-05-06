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

#include "modules/planning/planning_base/common/trajectory1d/piecewise_trajectory1d.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

double PiecewiseTrajectory1d::Evaluate(const std::uint32_t order,
                                       const double param) const {
  auto it_lower = std::lower_bound(accumulated_param_lengths_.begin(),
                                   accumulated_param_lengths_.end(), param);
  ACHECK(it_lower != accumulated_param_lengths_.end());

  size_t index = (size_t)(it_lower - accumulated_param_lengths_.begin());

  double param_overhead = 0.0;
  if (index != 0) {
    param_overhead = accumulated_param_lengths_[index - 1];
  }
  return trajectory_segments_[index]->Evaluate(order, param - param_overhead);
}

double PiecewiseTrajectory1d::ParamLength() const {
  if (accumulated_param_lengths_.empty()) {
    return 0.0;
  }
  return accumulated_param_lengths_.back();
}

std::string PiecewiseTrajectory1d::ToString() const { return ""; }

void PiecewiseTrajectory1d::AppendSegment(
    const std::shared_ptr<Curve1d> trajectory) {
  if (trajectory_segments_.empty()) {
    trajectory_segments_.push_back(trajectory);
  } else {
    double s1 = trajectory->Evaluate(0, 0.0);
    double v1 = trajectory->Evaluate(1, 0.0);
    double a1 = trajectory->Evaluate(2, 0.0);
    double j1 = trajectory->Evaluate(3, 0.0);

    auto last_trajectory = trajectory_segments_.back();
    double last_param_length = last_trajectory->ParamLength();
    double s0 = last_trajectory->Evaluate(0, last_param_length);
    double v0 = last_trajectory->Evaluate(1, last_param_length);
    double a0 = last_trajectory->Evaluate(2, last_param_length);
    double j0 = last_trajectory->Evaluate(3, last_param_length);

    if (std::fabs(s0 - s1) > 1.0e-4) {
      AWARN << "The appended segment is not smooth in order 0";
    }

    if (std::fabs(v0 - v1) > 1.0e-4) {
      AWARN << "The appended segment is not smooth in order 1";
    }

    if (std::fabs(a0 - a1) > 1.0e-4) {
      AWARN << "The appended segment is not smooth in order 2";
    }

    if (std::fabs(j0 - j1) > 1.0e-4) {
      AWARN << "The appended segment is not smooth in order 3";
    }
    trajectory_segments_.push_back(trajectory);
  }

  double accumulated_param_length = trajectory->ParamLength();
  if (accumulated_param_lengths_.size() > 0) {
    accumulated_param_length += accumulated_param_lengths_.back();
  }
  accumulated_param_lengths_.push_back(accumulated_param_length);
}

void PiecewiseTrajectory1d::PopSegment() {
  if (trajectory_segments_.empty()) {
    return;
  }
  trajectory_segments_.pop_back();
  accumulated_param_lengths_.pop_back();
}

size_t PiecewiseTrajectory1d::NumOfSegments() const {
  return trajectory_segments_.size();
}

}  // namespace planning
}  // namespace apollo
