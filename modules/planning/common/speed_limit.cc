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

void SpeedLimit::AppendSpeedLimitInfo(std::pair<double, double> speed_limit_info) {
  if (!speed_limit_info_.empty()) {
    CHECK(speed_limit_info_.back().first < speed_limit_info.first);
  }
  speed_limit_info_.push_back(std::move(speed_limit_info));
}

const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_info() const {
  return speed_limit_info_;
}

double SpeedLimit::GetSpeedLimit(const double s) const {
  if (speed_limit_info_.size() == 0) {
    return 0.0;
  }

  if (speed_limit_info_.size() == 1) {
    return speed_limit_info_.front().second;
  }

  auto func = [](const std::pair<double, double>& sp, const double s)
      { return sp.first < s; };

  auto it_lower =
      std::lower_bound(speed_limit_info_.begin(), speed_limit_info_.end(), s, func);
  if (it_lower == speed_limit_info_.begin()) {
    return speed_limit_info_.front().second;
  } else if (it_lower == speed_limit_info_.end()) {
    return speed_limit_info_.back().second;
  }

  double s0 = (it_lower - 1)->first;
  double v0 = (it_lower - 1)->second;
  double s1 = it_lower->first;
  double v1 = it_lower->second;
  return common::math::lerp(v0, s0, v1, s1, s);
}

}  // namespace planning
}  // namespace apollo
