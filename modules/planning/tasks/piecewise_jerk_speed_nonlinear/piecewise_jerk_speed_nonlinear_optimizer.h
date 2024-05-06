/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file piecewise_jerk_speed_nonlinear_optimizer.h
 **/

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/tasks/piecewise_jerk_speed_nonlinear/proto/piecewise_jerk_speed_nonlinear.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/planning_interface_base/task_base/common/speed_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkSpeedNonlinearOptimizer : public SpeedOptimizer {
 public:
  PiecewiseJerkSpeedNonlinearOptimizer();

  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  virtual ~PiecewiseJerkSpeedNonlinearOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;

  common::Status SetUpStatesAndBounds(const PathData& path_data,
                                      const SpeedData& speed_data);

  bool CheckSpeedLimitFeasibility();

  common::Status SmoothSpeedLimit();

  common::Status SmoothPathCurvature(const PathData& path_data);

  common::Status OptimizeByQP(SpeedData* const speed_data,
                              std::vector<double>* distance,
                              std::vector<double>* velocity,
                              std::vector<double>* acceleration);

  common::Status OptimizeByNLP(std::vector<double>* distance,
                               std::vector<double>* velocity,
                               std::vector<double>* acceleration);

  // st problem dimensions
  double delta_t_ = 0.0;
  double total_length_ = 0.0;
  double total_time_ = 0.0;
  int num_of_knots_ = 0;

  // st initial values
  double s_init_ = 0.0;
  double s_dot_init_ = 0.0;
  double s_ddot_init_ = 0.0;

  // st states dynamically feasibility bounds
  double s_dot_max_ = 0.0;
  double s_ddot_min_ = 0.0;
  double s_ddot_max_ = 0.0;
  double s_dddot_min_ = 0.0;
  double s_dddot_max_ = 0.0;

  // st safety bounds
  std::vector<std::pair<double, double>> s_bounds_;
  std::vector<std::pair<double, double>> s_soft_bounds_;

  // speed limits
  SpeedLimit speed_limit_;
  PiecewiseJerkTrajectory1d smoothed_speed_limit_;

  // smoothed path curvature profile as a function of traversal distance
  PiecewiseJerkTrajectory1d smoothed_path_curvature_;

  // reference speed profile
  SpeedData reference_speed_data_;

  // reference cruise speed
  double cruise_speed_;
  PiecewiseJerkNonlinearSpeedOptimizerConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::PiecewiseJerkSpeedNonlinearOptimizer, Task)

}  // namespace planning
}  // namespace apollo
