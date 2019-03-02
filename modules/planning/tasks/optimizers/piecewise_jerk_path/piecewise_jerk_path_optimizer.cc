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
 * @file piecewise_jerk_path_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"

#include <memory>
#include <utility>
#include <vector>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/finite_element_qp/fem_1d_qp_problem.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer(const TaskConfig& config)
    : PathOptimizer(config) {
  SetName("PiecewiseJerkPathOptimizer");
  CHECK(config_.has_piecewise_jerk_path_config());
}

common::Status PiecewiseJerkPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, PathData* const path_data) {
  const auto frenet_point =
      reference_line.GetFrenetPoint(init_point.path_point());

  const auto& piecewise_jerk_path_config = config_.piecewise_jerk_path_config();

  std::vector<std::pair<double, double>> lateral_boundaries;
  double start_s = 0.0;
  double delta_s = 0.0;
  reference_line_info_->GetPathBoundaries(&lateral_boundaries, &start_s,
                                          &delta_s);

  if (lateral_boundaries.size() < 2) {
    AERROR << "lateral boundary size < 2";
    return Status(ErrorCode::PLANNING_ERROR, "invalid lateral bounds provided");
  }

  // TODO(all): clean this debug messages after the branch is stable
  /**
  AERROR << "Init point:";
  AERROR << "\tl:\t" << frenet_point.l();
  AERROR << "\tdl:\t" << frenet_point.dl();
  AERROR << "\tddl:\t" << frenet_point.ddl();
  **/

  /**
  AERROR << "Weights:";
  AERROR << "\tl:\t" << piecewise_jerk_path_config.l_weight();
  AERROR << "\tdl:\t" << piecewise_jerk_path_config.dl_weight();
  AERROR << "\tddl:\t" << piecewise_jerk_path_config.ddl_weight();
  AERROR << "\tdddl:\t" << piecewise_jerk_path_config.dddl_weight();
  **/

  auto num_of_points = lateral_boundaries.size();

  std::array<double, 5> w = {
      piecewise_jerk_path_config.l_weight(),
      piecewise_jerk_path_config.dl_weight(),
      piecewise_jerk_path_config.ddl_weight(),
      piecewise_jerk_path_config.dddl_weight(),
      0.0
  };

  std::array<double, 3> init_lateral_state{frenet_point.l(), frenet_point.dl(),
                                           frenet_point.ddl()};

  std::unique_ptr<Fem1dQpProblem> fem_1d_qp(
      new Fem1dQpProblem(num_of_points, init_lateral_state, delta_s, w,
          FLAGS_lateral_jerk_bound));

  auto start_time = std::chrono::system_clock::now();

  fem_1d_qp->SetZeroOrderBounds(lateral_boundaries);

  FLAGS_lateral_derivative_bound_default = 1.0;
  double first_order_bounds = AdjustLateralDerivativeBounds(init_point.v(),
      frenet_point.dl(), frenet_point.ddl(),
      FLAGS_lateral_derivative_bound_default);
  AERROR << "adjusted lateral bound from \t"
      << FLAGS_lateral_derivative_bound_default << "\t" << first_order_bounds;
  fem_1d_qp->SetFirstOrderBounds(first_order_bounds);
  fem_1d_qp->SetSecondOrderBounds(FLAGS_lateral_derivative_bound_default);

  bool success = fem_1d_qp->Optimize();

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "piecewise jerk path optimizer failed";
    return Status(ErrorCode::PLANNING_ERROR,
        "piecewise jerk path optimizer failed");
  }

  const auto& x = fem_1d_qp->x();
  const auto& dx = fem_1d_qp->x_derivative();
  const auto& ddx = fem_1d_qp->x_second_order_derivative();

  CHECK(!x.empty() && x.size() == num_of_points);
  CHECK(!dx.empty() && dx.size() == num_of_points);
  CHECK(!ddx.empty() && ddx.size() == num_of_points);

  /*
  // TODO(all): an ad-hoc check for path feasibility since osqp
  //            cannot return the correct status
  const double numerical_buffer = 0.05;
  for (std::size_t i = 0; i < num_of_points; ++i) {
    if (x[i] < lateral_boundaries[i].first - numerical_buffer
        || x[i] > lateral_boundaries[i].second + numerical_buffer) {
      AERROR << "piecewise jerk path optimizer finds a infeasible solution";
      AERROR << "index\t" << i << ":\t" << x[i] <<
          "\t" << lateral_boundaries[i].first <<
          "\t" << lateral_boundaries[i].second;
      return Status(ErrorCode::PLANNING_ERROR,
          "piecewise jerk path optimizer failed");
    }
  }
  */

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(),
      dx.front(), ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  path_data->SetReferenceLine(&reference_line);
  path_data->SetFrenetPath(FrenetFramePath(frenet_frame_path));

  return Status::OK();
}

double PiecewiseJerkPathOptimizer::AdjustLateralDerivativeBounds(
    const double s_dot, const double dl, const double ddl,
    const double l_dot_bounds) const {
  double s = std::fmax(FLAGS_vehicle_low_speed_threshold, s_dot);
  double l_prime_adjusted = l_dot_bounds / s;
  if (l_prime_adjusted < std::fabs(dl)) {
    l_prime_adjusted = std::fabs(dl) + 0.1;
  }
  return l_prime_adjusted;
}

}  // namespace planning
}  // namespace apollo
