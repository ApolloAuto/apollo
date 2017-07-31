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
 * @file speed_limit.h
 **/

#include <vector>

#include "modules/common/proto/path_point.pb.h"

#ifndef MODULES_PLANNING_COMMON_SPEED_LIMIT_H_
#define MODULES_PLANNING_COMMON_SPEED_LIMIT_H_

namespace apollo {
namespace planning {

class SpeedLimit {
 public:
  SpeedLimit() = default;
  void add_speed_limit(const common::SpeedPoint& speed_point);
  void add_speed_limit(const double s, const double t, const double v,
                       const double a, const double da);
  const std::vector<common::SpeedPoint>& speed_limits() const;
  double get_speed_limit(const double s) const;

 private:
  std::vector<common::SpeedPoint> _speed_point;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_SPEED_LIMIT_H_
