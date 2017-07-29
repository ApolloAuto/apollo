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
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using common::SpeedPoint;

void SpeedLimit::add_speed_limit(const SpeedPoint& speed_point) {
  _speed_point.push_back(speed_point);
}

void SpeedLimit::add_speed_limit(const double s, const double t, const double v,
                                 const double a, const double da) {
  SpeedPoint speed_point;
  speed_point.set_s(s);
  speed_point.set_t(t);
  speed_point.set_v(v);
  speed_point.set_a(a);
  speed_point.set_da(da);
  _speed_point.push_back(speed_point);
}

const std::vector<SpeedPoint>& SpeedLimit::speed_limits() const {
  return _speed_point;
}

double SpeedLimit::get_speed_limit(const double s) const {
  double ref_s = s;
  if (_speed_point.size() == 0) {
    return 0.0;
  }

  if (_speed_point.size() == 1) {
    return _speed_point.front().v();
  }

  if (ref_s > _speed_point.back().s()) {
    return _speed_point.back().v();
  }

  if (ref_s < _speed_point.front().s()) {
    return _speed_point.front().v();
  }

  auto func = [](const SpeedPoint & sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(_speed_point.begin(), _speed_point.end(), s, func);
  if (it_lower == _speed_point.begin()) {
    return _speed_point.front().v();
  }

  double weight = 0.0;
  double range = (*it_lower).s() - (*(it_lower - 1)).s();
  if (range > 0) {
    weight = (s - (*(it_lower - 1)).s()) / range;
  }
  return util::interpolate(*(it_lower - 1), *it_lower, weight).v();
}
}  // namespace planning
}  // namespace apollo
