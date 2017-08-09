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
 * @file speed_limit.cc
 **/

#include "modules/planning/common/speed_limit.h"

#include <algorithm>
#include <iostream>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using common::SpeedPoint;

void SpeedLimit::AddSpeedLimit(const SpeedPoint& speed_point) {
  if (!speed_points_.empty()) {
    DCHECK_GE(speed_point.s(), speed_points_.back().s());
    DCHECK_GE(speed_point.t(), speed_points_.back().t());
  }
  speed_points_.push_back(std::move(speed_point));
}

const std::vector<SpeedPoint>& SpeedLimit::speed_points() const {
  return speed_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  DCHECK_GE(speed_points_.size(), 2);
  DCHECK_GE(s, speed_points_.front().s());

  auto compare_s = [](const SpeedPoint& sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(speed_points_.begin(), speed_points_.end(),
                                   s, compare_s);
  return it_lower->v();
}

double SpeedLimit::GetSpeedLimitByT(const double t) const {
  DCHECK_GE(speed_points_.size(), 2);
  DCHECK_GE(t, speed_points_.front().t());

  auto compare_t = [](const SpeedPoint& sp, const double t) {
    return sp.t() < t;
  };

  auto it_lower = std::lower_bound(speed_points_.begin(), speed_points_.end(),
                                   t, compare_t);
  return it_lower->v();
}

}  // namespace planning
}  // namespace apollo
