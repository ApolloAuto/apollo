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
 * @file dp_st_cost.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_COST_H_
#define MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_COST_H_

#include <vector>

#include "modules/common/proto/path_point.pb.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/optimizer/dp_st_speed/dp_st_configuration.h"
#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"

namespace apollo {
namespace planning {

class DpStCost {
 public:
  explicit DpStCost(const DpStConfiguration& dp_st_configuration);

  double obstacle_cost(const STPoint& point,
                       const std::vector<StGraphBoundary>& obs_boundary) const;

  double reference_cost(const STPoint& point,
                        const STPoint& reference_point) const;

  double speed_cost(const STPoint& first, const STPoint& second,
                    const double speed_limit) const;

  double accel_cost(const double accel) const;
  double accel_cost_by_two_points(const double pre_speed, const STPoint& first,
                                  const STPoint& second) const;
  double accel_cost_by_three_points(const STPoint& first, const STPoint& second,
                                    const STPoint& third) const;

  double jerk_cost(const double jerk) const;
  double jerk_cost_by_two_points(const double pre_speed, const double pre_acc,
                                 const STPoint& pre_point,
                                 const STPoint& curr_point) const;
  double jerk_cost_by_three_points(const double first_speed,
                                   const STPoint& first_point,
                                   const STPoint& second_point,
                                   const STPoint& third_point) const;

  double jerk_cost_by_four_points(const STPoint& first, const STPoint& second,
                                  const STPoint& third,
                                  const STPoint& forth) const;

 private:
  DpStConfiguration _dp_st_configuration;
  double _unit_s = 0.0;
  double _unit_t = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_COST_H_
