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

#pragma once

#include <utility>
#include <vector>

namespace apollo {
namespace planning {

class SpeedLimit {
 public:
  SpeedLimit() = default;

  void AppendSpeedLimit(const double s, const double v);

  const std::vector<std::pair<double, double>>& speed_limit_points() const;

  double GetSpeedLimitByS(const double s) const;

  void Clear();

 private:
  // use a vector to represent speed limit
  // the first number is s, the second number is v
  // It means at distance s from the start point, the speed limit is v.
  std::vector<std::pair<double, double>> speed_limit_points_;
};

}  // namespace planning
}  // namespace apollo
