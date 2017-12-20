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
 * @file trajectory_cost.h
 **/

#ifndef MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
#define MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class ComparableCost {
 public:
  ComparableCost() = default;
  ComparableCost(const bool has_collision_, const bool out_of_boundary_,
                 const double safety_cost_, const double smoothness_cost_)
      : has_collision(has_collision_),
        out_of_boundary(out_of_boundary_),
        safety_cost(safety_cost_),
        smoothness_cost(smoothness_cost_) {}
  ComparableCost(const ComparableCost &) = default;

  int CompareTo(const ComparableCost &other) const {
    constexpr double kEpsilon = 1e-12;
    if ((has_collision || out_of_boundary) &&
        !(other.has_collision || other.out_of_boundary)) {
      return 1;
    } else if (!(has_collision || out_of_boundary) &&
               (other.has_collision || other.out_of_boundary)) {
      return -1;
    } else if (std::fabs(safety_cost + smoothness_cost - other.safety_cost -
                         other.smoothness_cost) < kEpsilon) {
      return 0;
    } else if (safety_cost + smoothness_cost >
               other.safety_cost + other.smoothness_cost) {
      return 1;
    } else {
      return -1;
    }
  }
  ComparableCost &operator+(const ComparableCost &other) {
    has_collision = has_collision || other.has_collision;
    out_of_boundary = out_of_boundary || other.out_of_boundary;
    safety_cost += other.safety_cost;
    smoothness_cost += other.smoothness_cost;
    return *this;
  }
  ComparableCost &operator+=(const ComparableCost &other) {
    has_collision = has_collision || other.has_collision;
    out_of_boundary = out_of_boundary || other.out_of_boundary;
    safety_cost += other.safety_cost;
    smoothness_cost += other.smoothness_cost;
    return *this;
  }
  bool operator>(const ComparableCost &other) const {
    return this->CompareTo(other) > 0;
  }
  bool operator>=(const ComparableCost &other) const {
    return this->CompareTo(other) >= 0;
  }
  bool operator<(const ComparableCost &other) const {
    return this->CompareTo(other) < 0;
  }
  bool operator<=(const ComparableCost &other) const {
    return this->CompareTo(other) <= 0;
  }

  bool has_collision = false;
  bool out_of_boundary = false;

  // cost from distance to obstacles or boundaries
  double safety_cost = 0.0;
  // cost from deviation from lane center, path curvature etc
  double smoothness_cost = 0.0;
};

class TrajectoryCost {
 public:
  explicit TrajectoryCost(const DpPolyPathConfig &config,
                          const ReferenceLine &reference_line,
                          const bool is_change_lane_path,
                          const std::vector<const PathObstacle *> &obstacles,
                          const common::VehicleParam &vehicle_param,
                          const SpeedData &heuristic_speed_data,
                          const common::SLPoint &init_sl_point);
  ComparableCost Calculate(const QuinticPolynomialCurve1d &curve,
                           const double start_s, const double end_s,
                           const uint32_t curr_level,
                           const uint32_t total_level) const;
  double RiskDistanceCost(const double distance) const;
  double RegularDistanceCost(const double distance) const;

 private:
  ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                   const double start_s, const double end_s,
                                   const uint32_t curr_level,
                                   const uint32_t total_level) const;
  ComparableCost CalculateStaticObstacleCost(
      const QuinticPolynomialCurve1d &curve, const double start_s,
      const double end_s) const;
  ComparableCost CalculateDynamicObstacleCost(
      const QuinticPolynomialCurve1d &curve, const double start_s,
      const double end_s) const;
  ComparableCost GetCostBetweenObsBoxes(
      const common::math::Box2d &ego_box,
      const common::math::Box2d &obstacle_box) const;

  ComparableCost GetCostFromObsSL(const double adc_s, const double adc_l,
                                  const SLBoundary &obs_sl_boundary) const;

  common::math::Box2d GetBoxFromSLPoint(const common::SLPoint &sl,
                                        const double dl) const;

  const DpPolyPathConfig config_;
  const ReferenceLine *reference_line_ = nullptr;
  bool is_change_lane_path_ = false;
  const common::VehicleParam vehicle_param_;
  SpeedData heuristic_speed_data_;
  const common::SLPoint init_sl_point_;
  uint32_t num_of_time_stamps_ = 0;
  std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;
  std::vector<double> obstacle_probabilities_;

  std::vector<SLBoundary> static_obstacle_sl_boundaries_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
