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

class TrajectoryCost {
 public:
  explicit TrajectoryCost(const DpPolyPathConfig &config,
                          const ReferenceLine &reference_line,
                          const std::vector<const PathObstacle *> &obstacles,
                          const common::VehicleParam &vehicle_param,
                          const SpeedData &heuristic_speed_data,
                          const common::SLPoint &init_sl_point);
  double Calculate(const QuinticPolynomialCurve1d &curve, const double start_s,
                   const double end_s) const;
  double RiskDistanceCost(const double distance) const;
  double RegularDistanceCost(const double distance) const;

 private:
  double CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                           const double start_s, const double end_s) const;
  double CalculateStaticObstacleCost(const QuinticPolynomialCurve1d &curve,
                                     const double start_s,
                                     const double end_s) const;
  double CalculateDynamicObstacleCost(const QuinticPolynomialCurve1d &curve,
                                      const double start_s,
                                      const double end_s) const;
  double GetCostBetweenObsBoxes(const common::math::Box2d &ego_box,
                                const common::math::Box2d &obstacle_box) const;

  common::math::Box2d GetBoxFromSLPoint(const common::SLPoint &sl,
                                        const double dl) const;

  const DpPolyPathConfig config_;
  const ReferenceLine *reference_line_ = nullptr;
  const common::VehicleParam vehicle_param_;
  SpeedData heuristic_speed_data_;
  const common::SLPoint init_sl_point_;
  uint32_t num_of_time_stamps_ = 0;
  std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;
  std::vector<double> obstacle_probabilities_;

  std::vector<common::math::Box2d> static_obstacle_boxes_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
