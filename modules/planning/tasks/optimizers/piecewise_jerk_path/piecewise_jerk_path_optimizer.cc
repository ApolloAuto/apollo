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
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/piecewise_jerk/fem_1d_qp_problem.h"

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
    const common::TrajectoryPoint& init_point,
    PathData* const final_path_data) {
  const auto init_frenet_state = reference_line.ToFrenetFrame(init_point);

  const auto& piecewise_jerk_path_config = config_.piecewise_jerk_path_config();
  std::array<double, 5> w = {
      piecewise_jerk_path_config.l_weight(),
      piecewise_jerk_path_config.dl_weight() *
          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],
                    1.0),
      piecewise_jerk_path_config.ddl_weight(),
      piecewise_jerk_path_config.dddl_weight(), 0.0};

  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";

  std::vector<PathData> candidate_path_data;
  for (const auto& path_boundary : path_boundaries) {
    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary.boundary().size() < 2) {
      continue;
    }

    int max_iter = 4000;
    // lower max_iter for regular/self/
    if (path_boundary.label().find("self") != std::string::npos) {
      max_iter = 4000;
    }

    CHECK_GT(path_boundary.boundary().size(), 1);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;

    const auto& pull_over_info =
        PlanningContext::Instance()->planning_status().pull_over();

    std::array<double, 3> end_state = {0.0, 0.0, 0.0};
    // Set end lateral to be at the desired pull over destination if enter into
    // pull over scenario.
    if (pull_over_info.exist_pull_over_position()) {
      end_state[0] = pull_over_info.pull_over_l();
    }

    bool res_opt = OptimizePath(
        init_frenet_state, end_state, path_boundary.delta_s(),
        path_boundary.boundary(), w, &opt_l, &opt_dl, &opt_ddl, max_iter);

    if (res_opt) {
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());

      // TODO(all): double-check this;
      // final_path_data might carry info from upper stream
      PathData path_data = *final_path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(FrenetFramePath(frenet_frame_path));
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
  }
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

// TODO(Hongyi): deprecate "AdjustLateralDerivativeBounds" and change interface
// of init_state to array of double
bool PiecewiseJerkPathOptimizer::OptimizePath(
    const std::pair<const std::array<double, 3>, const std::array<double, 3>>&
        init_state,
    const std::array<double, 3>& end_state, const double delta_s,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::array<double, 5>& w, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx, const int max_iter) {
  std::unique_ptr<Fem1dQpProblem> fem_1d_qp(new Fem1dQpProblem());
  fem_1d_qp->InitProblem(lat_boundaries.size(), delta_s, w, init_state.second,
                         end_state);
  fem_1d_qp->SetThirdOrderBound(FLAGS_lateral_jerk_bound);

  auto start_time = std::chrono::system_clock::now();

  fem_1d_qp->SetZeroOrderBounds(lat_boundaries);

  double first_order_bounds = AdjustLateralDerivativeBounds(
      init_state.first[1], init_state.second[1], init_state.second[2],
      FLAGS_lateral_derivative_bound_default);
  ADEBUG << "adjusted lateral derivative bound from \t"
         << FLAGS_lateral_derivative_bound_default << "\t"
         << first_order_bounds;
  // TODO(all): temp. disable AdjustLateralDerivativeBounds, enable later
  // fem_1d_qp->SetFirstOrderBounds(-first_order_bounds, first_order_bounds);
  fem_1d_qp->SetFirstOrderBounds(-FLAGS_lateral_derivative_bound_default,
                                 FLAGS_lateral_derivative_bound_default);
  fem_1d_qp->SetSecondOrderBounds(-FLAGS_lateral_derivative_bound_default,
                                  FLAGS_lateral_derivative_bound_default);

  bool success = fem_1d_qp->Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = fem_1d_qp->x();
  *dx = fem_1d_qp->x_derivative();
  *ddx = fem_1d_qp->x_second_order_derivative();

  return true;
}

FrenetFramePath PiecewiseJerkPathOptimizer::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  CHECK(!x.empty());
  CHECK(!dx.empty());
  CHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

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

  return FrenetFramePath(frenet_frame_path);
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
