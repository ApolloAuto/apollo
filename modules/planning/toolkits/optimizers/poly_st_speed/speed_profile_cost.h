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

#ifndef MODULES_PLANNING_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_
#define MODULES_PLANNING_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/poly_st_speed_config.pb.h"

#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

class SpeedProfileCost {
 public:
  explicit SpeedProfileCost(const PolyStSpeedConfig &config,
                            const std::vector<const PathObstacle *> &obstacles,
                            const SpeedLimit &speed_limit,
                            const common::TrajectoryPoint &init_point);

  double Calculate(const QuarticPolynomialCurve1d &curve, const double end_time,
                   const double curr_min_cost) const;

 private:
  double CalculatePointCost(const QuarticPolynomialCurve1d &curve,
                            const double t) const;

  const PolyStSpeedConfig config_;
  const std::vector<const PathObstacle *> &obstacles_;
  const SpeedLimit &speed_limit_;
  const common::TrajectoryPoint &init_point_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_
