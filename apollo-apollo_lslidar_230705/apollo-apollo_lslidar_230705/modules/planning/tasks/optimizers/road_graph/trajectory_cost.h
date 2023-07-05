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

#pragma once

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/optimizers/road_graph/comparable_cost.h"

namespace apollo {
namespace planning {

class TrajectoryCost {
 public:
  TrajectoryCost() = default;
  TrajectoryCost(const DpPolyPathConfig &config,
                 const ReferenceLine &reference_line,
                 const bool is_change_lane_path,
                 const std::vector<const Obstacle *> &obstacles,
                 const common::VehicleParam &vehicle_param,
                 const SpeedData &heuristic_speed_data,
                 const common::SLPoint &init_sl_point,
                 const SLBoundary &adc_sl_boundary);
  ComparableCost Calculate(const QuinticPolynomialCurve1d &curve,
                           const double start_s, const double end_s,
                           const uint32_t curr_level,
                           const uint32_t total_level);

 private:
  ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                   const double start_s, const double end_s,
                                   const uint32_t curr_level,
                                   const uint32_t total_level);
  ComparableCost CalculateStaticObstacleCost(
      const QuinticPolynomialCurve1d &curve, const double start_s,
      const double end_s);
  ComparableCost CalculateDynamicObstacleCost(
      const QuinticPolynomialCurve1d &curve, const double start_s,
      const double end_s) const;
  ComparableCost GetCostBetweenObsBoxes(
      const common::math::Box2d &ego_box,
      const common::math::Box2d &obstacle_box) const;

  FRIEND_TEST(AllTrajectoryTests, GetCostFromObsSL);
  ComparableCost GetCostFromObsSL(const double adc_s, const double adc_l,
                                  const SLBoundary &obs_sl_boundary);

  common::math::Box2d GetBoxFromSLPoint(const common::SLPoint &sl,
                                        const double dl) const;

  bool IsOffRoad(const double ref_s, const double l, const double dl,
                 const bool is_change_lane_path);

  const DpPolyPathConfig config_;
  const ReferenceLine *reference_line_ = nullptr;
  bool is_change_lane_path_ = false;
  const common::VehicleParam vehicle_param_;
  SpeedData heuristic_speed_data_;
  const common::SLPoint init_sl_point_;
  const SLBoundary adc_sl_boundary_;
  uint32_t num_of_time_stamps_ = 0;
  std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;
  std::vector<double> obstacle_probabilities_;

  std::vector<SLBoundary> static_obstacle_sl_boundaries_;
};

}  // namespace planning
}  // namespace apollo
