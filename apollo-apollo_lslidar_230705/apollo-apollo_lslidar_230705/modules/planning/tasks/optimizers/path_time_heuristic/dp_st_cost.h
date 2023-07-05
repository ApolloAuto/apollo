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

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/st_drivable_boundary.pb.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/st_graph_point.h"

namespace apollo {
namespace planning {

class DpStCost {
 public:
  DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
           const double total_s, const std::vector<const Obstacle*>& obstacles,
           const STDrivableBoundary& st_drivable_boundary,
           const common::TrajectoryPoint& init_point);

  double GetObstacleCost(const StGraphPoint& point);

  double GetSpatialPotentialCost(const StGraphPoint& point);

  double GetReferenceCost(const STPoint& point,
                          const STPoint& reference_point) const;

  double GetSpeedCost(const STPoint& first, const STPoint& second,
                      const double speed_limit,
                      const double cruise_speed) const;

  double GetAccelCostByTwoPoints(const double pre_speed, const STPoint& first,
                                 const STPoint& second);
  double GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                   const STPoint& third);

  double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                const STPoint& pre_point,
                                const STPoint& curr_point);
  double GetJerkCostByThreePoints(const double first_speed,
                                  const STPoint& first_point,
                                  const STPoint& second_point,
                                  const STPoint& third_point);

  double GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                 const STPoint& third, const STPoint& fourth);

 private:
  double GetAccelCost(const double accel);
  double JerkCost(const double jerk);

  void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<double, double>>* keep_clear_range_);
  bool InKeepClearRange(double s) const;

  const DpStSpeedOptimizerConfig& config_;
  const std::vector<const Obstacle*>& obstacles_;

  STDrivableBoundary st_drivable_boundary_;

  const common::TrajectoryPoint& init_point_;

  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  std::vector<std::pair<double, double>> keep_clear_range_;

  std::array<double, 200> accel_cost_;
  std::array<double, 400> jerk_cost_;
};

}  // namespace planning
}  // namespace apollo
