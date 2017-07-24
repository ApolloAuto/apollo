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
 * @file trjactory_cost.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_TRAJECTORY_COST_H
#define MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_TRAJECTORY_COST_H

#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/planning/common/decision_data.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class TrajectoryCost {
 public:
  explicit TrajectoryCost(const DpPolyPathConfig &config,
                          const SpeedData &heuristic_speed_data,
                          const DecisionData &decision_data);
  double calculate(const QuinticPolynomialCurve1d &curve, const double start_s,
                   const double end_s, const double length, const double width,
                   const ReferenceLine &reference_line) const;

 private:
  DpPolyPathConfig _config;
  SpeedData _heuristic_speed_data;
  std::vector<std::vector<::apollo::common::math::Box2d>> _obstacle_trajectory;
  std::vector<double> _obstacle_probability;
  uint32_t _evaluate_times;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_TRAJECTORY_COST_H
