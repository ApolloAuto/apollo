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

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace {
constexpr auto kInfCost = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-6;
}  // namespace

using apollo::common::TrajectoryPoint;

SpeedProfileCost::SpeedProfileCost(
    const PolyStSpeedConfig &config,
    const std::vector<const PathObstacle *> &obstacles,
    const SpeedLimit &speed_limit, const common::TrajectoryPoint &init_point)
    : config_(config),
      obstacles_(obstacles),
      speed_limit_(speed_limit),
      init_point_(init_point) {}

double SpeedProfileCost::Calculate(const QuarticPolynomialCurve1d &curve,
                                   const double end_time,
                                   const double curr_min_cost) const {
  double cost = 0.0;
  constexpr double kDeltaT = 0.5;
  for (double t = kDeltaT; t < end_time + kEpsilon; t += kDeltaT) {
    if (cost > curr_min_cost) {
      return cost;
    }
    cost += CalculatePointCost(curve, t);
  }
  return cost;
}

double SpeedProfileCost::CalculatePointCost(
    const QuarticPolynomialCurve1d &curve, const double t) const {
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

  double cost = 0.0;
  for (const auto *obstacle : obstacles_) {
    auto boundary = obstacle->st_boundary();
    const double kIgnoreDistance = 100.0;
    if (boundary.min_s() > kIgnoreDistance) {
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    if (obstacle->IsBlockingObstacle() &&
        boundary.IsPointInBoundary(STPoint(s, t))) {
      return kInfCost;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;
    boundary.GetBoundarySRange(t, &s_upper, &s_lower);
    if (s < s_lower) {
      const double len = v * FLAGS_follow_time_buffer;
      if (s + len < s_lower) {
        continue;
      } else {
        cost += config_.obstacle_weight() * std::pow((len - s_lower + s), 2);
      }
    } else if (s > s_upper) {
      const double kSafeDistance = 15.0;  // or calculated from velocity
      if (s > s_upper + kSafeDistance) {
        continue;
      } else {
        cost += config_.obstacle_weight() *
                std::pow((kSafeDistance + s_upper - s), 2);
      }
    } else {
      if (!obstacle->IsBlockingObstacle()) {
        cost += config_.unblocking_obstacle_cost();
      }
    }
  }
  cost += config_.speed_weight() * std::pow((v - speed_limit), 2);
  cost += config_.jerk_weight() * std::pow(da, 2);
  ADEBUG << "t = " << t << ", s = " << s << ", v = " << v << ", a = " << a
         << ", da = " << da << ", cost = " << cost;

  return cost;
}

}  // namespace planning
}  // namespace apollo
