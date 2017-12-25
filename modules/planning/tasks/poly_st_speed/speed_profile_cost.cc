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

#include "modules/planning/tasks/poly_st_speed/speed_profile_cost.h"

#include <limits>

namespace apollo {
namespace planning {
namespace {
constexpr auto kInfCost = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-6;
}

using apollo::common::TrajectoryPoint;

SpeedProfileCost::SpeedProfileCost(
    const PolyStSpeedConfig &config,
    const std::vector<const PathObstacle *> &obstacles,
    const SpeedLimit &speed_limit)
    : config_(config), obstacles_(obstacles), speed_limit_(speed_limit) {}

double SpeedProfileCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                   const double end_time) const {
  double cost = 0.0;
  constexpr double kDeltaT = 0.5;
  for (double t = kDeltaT; t < end_time + kEpsilon; t += kDeltaT) {
    cost += CalculatePointCost(curve, t);
  }
  return cost;
}

double SpeedProfileCost::CalculatePointCost(
    const QuinticPolynomialCurve1d &curve, const double t) const {
  const double s = curve.Evaluate(0, t);
  const double v = curve.Evaluate(1, t);
  const double a = curve.Evaluate(2, t);
  const double da = curve.Evaluate(3, t);

  if (s < 0.0) {
    return kInfCost;
  }

  const double speed_limit = speed_limit_.GetSpeedLimitByS(s);
  if (v < 0.0 || v > speed_limit * (1.0 + config_.speed_limit_buffer())) {
    return kInfCost;
  }
  if (a > config_.preferred_accel() || a < config_.preferred_decel()) {
    return kInfCost;
  }
  for (const auto *obstacle : obstacles_) {
    auto boundary = obstacle->st_boundary();
    if (boundary.IsPointInBoundary(STPoint(s, t))) {
      return kInfCost;
    }
  }
  double cost = 0.0;
  cost += config_.speed_weight() * std::pow((v - speed_limit), 2);
  cost += config_.jerk_weight() * std::pow(da, 2);
  return cost;
}

}  // namespace planning
}  // namespace apollo
