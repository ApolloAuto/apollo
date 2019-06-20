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
 * @file
 **/

#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

#include <limits>

#include "cyber/common/log.h"
#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_ipopt_interface.h"
#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.h"

namespace apollo {
namespace planning {
FemPosDeviationSmoother::FemPosDeviationSmoother(
    const FemPosDeviationSmootherConfig& config)
    : config_(config) {}

bool FemPosDeviationSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (config_.apply_curvature_constraint()) {
    return SolveWithIpopt(raw_point2d, bounds, opt_x, opt_y);
  } else {
    return SolveWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  }
  return true;
}

bool FemPosDeviationSmoother::SolveWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  CHECK_NOTNULL(opt_x);
  CHECK_NOTNULL(opt_y);

  FemPosDeviationOsqpInterface solver;

  solver.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation());
  solver.set_weight_path_length(config_.weight_path_length());
  solver.set_weight_ref_deviation(config_.weight_ref_deviation());
  solver.set_max_iter(config_.max_iter());
  solver.set_time_limit(config_.time_limit());
  solver.set_verbose(config_.verbose());
  solver.set_scaled_termination(config_.scaled_termination());
  solver.set_warm_start(config_.warm_start());

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve()) {
    return false;
  }

  *opt_x = solver.opt_x();
  *opt_y = solver.opt_y();
  return true;
}

bool FemPosDeviationSmoother::SolveWithIpopt(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  // const double weight_fem_pos_deviation =
  // config_.weight_fem_pos_deviation();
  // const double weight_ref_deviation = config_.weight_ref_deviation();
  // const double weight_path_length = config_.weight_path_length();
  // const int max_iter = config_.max_iter();
  // const double time_limit = config_.time_limit();
  // const bool verbose = config_.verbose();
  // const bool scaled_termination = config_.scaled_termination();
  // const bool warm_start = config_.warm_start();
  return false;
}

}  // namespace planning
}  // namespace apollo
