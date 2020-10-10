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

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

void SpeedLimit::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    DCHECK_GE(s, speed_limit_points_.back().first);
  }
  speed_limit_points_.emplace_back(s, v);
}

const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()
    const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  CHECK_GE(speed_limit_points_.size(), 2);
  DCHECK_GE(s, speed_limit_points_.front().first);

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);

  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

void SpeedLimit::Clear() { speed_limit_points_.clear(); }

}  // namespace planning
}  // namespace apollo
