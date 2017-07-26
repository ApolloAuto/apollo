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

namespace apollo {
namespace planning {

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

}  // namespace planning
}  // namespace apollo
