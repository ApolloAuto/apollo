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
 * @file piecewise_jerk_path_optimizer.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/planning/tasks/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  explicit PiecewiseJerkPathOptimizer(const TaskConfig& config);

  virtual ~PiecewiseJerkPathOptimizer() = default;

 private:
  common::Status Process(const SpeedData& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         const bool path_reusable,
                         PathData* const path_data) override;

  common::TrajectoryPoint InferFrontAxeCenterFromRearAxeCenter(
      const common::TrajectoryPoint& traj_point);

  std::vector<common::PathPoint> ConvertPathPointRefFromFrontAxeToRearAxe(
      const PathData& path_data);

  bool OptimizePath(
      const std::array<double, 3>& init_state,
      const std::array<double, 3>& end_state, const double delta_s,
      const std::vector<std::pair<double, double>>& lat_boundaries,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      const std::array<double, 5>& w, std::vector<double>* ptr_x,
      std::vector<double>* ptr_dx, std::vector<double>* ptr_ddx,
      const int max_iter);

  FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
                                      const std::vector<double>& dl,
                                      const std::vector<double>& ddl,
                                      const double delta_s,
                                      const double start_s) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;
};

}  // namespace planning
}  // namespace apollo
