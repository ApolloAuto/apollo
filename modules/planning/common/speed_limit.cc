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

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using common::SpeedPoint;

void SpeedLimit::AddSpeedLimit(const SpeedPoint& speed_point) {
  speed_points_.push_back(speed_point);
}

const std::vector<SpeedPoint>& SpeedLimit::speed_points() const {
  return speed_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  if (speed_points_.size() == 0) {
    AWARN << "Speed points is empty.";
    return 0.0;
  }
  if (speed_points_.size() == 1) {
    return speed_points_.front().v();
  }
  if (s > speed_points_.back().s()) {
    AWARN << "s is larger than the last point of Speed points.";
    return speed_points_.back().v();
  }
  if (s < speed_points_.front().s()) {
    return speed_points_.front().v();
  }

  auto func = [](const SpeedPoint& sp, const double s) { return sp.s() < s; };

  auto it_lower =
      std::lower_bound(speed_points_.begin(), speed_points_.end(), s, func);
  if (it_lower == speed_points_.begin()) {
    return speed_points_.front().v();
  }

  double weight = 0.0;
  double range = (*it_lower).s() - (*(it_lower - 1)).s();
  if (range > 0) {
    weight = (s - (*(it_lower - 1)).s()) / range;
  }
  return util::interpolate(*(it_lower - 1), *it_lower, weight).v();
}

double SpeedLimit::GetSpeedLimitByT(const double t) const {
  if (speed_points_.size() == 0) {
    return 0.0;
  }

  if (speed_points_.size() == 1) {
    return speed_points_.front().v();
  }

  if (t > speed_points_.back().t()) {
    return speed_points_.back().v();
  }

  if (t < speed_points_.front().t()) {
    return speed_points_.front().v();
  }

  auto func = [](const SpeedPoint& sp, const double t) { return sp.t() < t; };

  auto it_lower =
      std::lower_bound(speed_points_.begin(), speed_points_.end(), t, func);
  if (it_lower == speed_points_.begin()) {
    return speed_points_.front().v();
  } else if (it_lower == speed_points_.end()) {
    return speed_points_.back().v();
  }

  double v0 = (it_lower - 1)->v();
  double t0 = (it_lower - 1)->t();

  double v1 = it_lower->v();
  double t1 = it_lower->t();

  return common::math::lerp(v0, t0, v1, t1, t);
}

}  // namespace planning
}  // namespace apollo
