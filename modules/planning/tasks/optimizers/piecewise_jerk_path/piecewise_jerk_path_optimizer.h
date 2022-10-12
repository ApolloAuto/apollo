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

#include <memory>
#include <utility>
#include <vector>

#include "modules/planning/tasks/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  PiecewiseJerkPathOptimizer(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

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

  /**
   * @brief
   *
   * @param init_state path start point
   * @param end_state path end point
   * @param path_reference_l_ref: a vector with default value 0.0
   * @param path_reference_size: length of learning model output
   * @param delta_s: path point spatial distance
   * @param is_valid_path_reference: whether using learning model output or not
   * @param lat_boundaries: path boundaries
   * @param ddl_bounds: constains
   * @param w: weighting scales
   * @param max_iter: optimization max interations
   * @param ptr_x: optimization result of x
   * @param ptr_dx: optimization result of dx
   * @param ptr_ddx: optimization result of ddx
   * @return true
   * @return false
   */
  bool OptimizePath(
      const std::array<double, 3>& init_state,
      const std::array<double, 3>& end_state,
      std::vector<double> path_reference_l_ref,
      const size_t path_reference_size, const double delta_s,
      const bool is_valid_path_reference,
      const std::vector<std::pair<double, double>>& lat_boundaries,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      const std::array<double, 5>& w, const int max_iter,
      std::vector<double>* ptr_x, std::vector<double>* ptr_dx,
      std::vector<double>* ptr_ddx);

  FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
                                      const std::vector<double>& dl,
                                      const std::vector<double>& ddl,
                                      const double delta_s,
                                      const double start_s) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;

  double GaussianWeighting(const double x, const double peak_weighting,
                           const double peak_weighting_x) const;
};

}  // namespace planning
}  // namespace apollo
