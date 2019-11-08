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
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_nonlinear_optimizer.h"

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/proto/ipopt_return_status.pb.h"
#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_nonlinear_ipopt_interface.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

PiecewiseJerkSpeedNonlinearOptimizer::PiecewiseJerkSpeedNonlinearOptimizer(
    const TaskConfig& config)
    : SpeedOptimizer(config) {
  CHECK(config_.has_piecewise_jerk_nonlinear_speed_config());
}

Status PiecewiseJerkSpeedNonlinearOptimizer::Process(
    const PathData& path_data, const TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  if (speed_data == nullptr) {
    std::string msg("Null speed_data pointer");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data.discretized_path().empty()) {
    std::string msg("Speed Optimizer receives empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }

  // Set st problem dimensions
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  const double delta_t = 0.1;
  const double total_length = st_graph_data.path_length();
  const double total_time = st_graph_data.total_time_by_conf();
  const int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

  // Set initial values
  const double s_init = 0.0;
  const double s_dot_init = st_graph_data.init_point().v();
  const double s_ddot_init = st_graph_data.init_point().a();

  // Set s boundary
  std::vector<std::pair<double, double>> s_bounds;
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    double s_lower_bound = 0.0;
    double s_upper_bound = total_length;
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      double s_lower = 0.0;
      double s_upper = 0.0;
      if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        continue;
      }
      switch (boundary->boundary_type()) {
        case STBoundary::BoundaryType::STOP:
        case STBoundary::BoundaryType::YIELD:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;
        case STBoundary::BoundaryType::FOLLOW:
          // TODO(Hongyi): unify follow buffer on decision side
          s_upper_bound = std::fmin(s_upper_bound, s_upper - 8.0);
          break;
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;
        default:
          break;
      }
    }
    if (s_lower_bound > s_upper_bound) {
      std::string msg("s_lower_bound larger than s_upper_bound on STGraph!");
      AERROR << msg;
      speed_data->clear();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }

  // Check s bound feasibility
  if (!CheckStBoundFeasibility(num_of_knots, delta_t,
                               {s_init, s_dot_init, s_ddot_init}, s_bounds)) {
    std::string msg(
        "Piecewise jerk speed nonlinear optimizer feasibility check failed!");
    AERROR << msg;
    speed_data->clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Set s_dot bounary
  const double max_velocity = std::fmax(FLAGS_planning_upper_speed_limit,
                                        st_graph_data.init_point().v());

  // Set s_ddot boundary
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double max_acceleration = veh_param.max_acceleration();
  double min_acceleration = -1.0 * std::abs(veh_param.max_deceleration());
  // TODO(Hongyi): delete this when ready to use vehicle_params
  max_acceleration = 2.0;
  min_acceleration = -4.0;

  // Set s_dddot boundary
  // TODO(Jinyun): allow the setting of jerk_lower_bound and move jerk config to
  // a better place
  const double max_abs_jerk = FLAGS_longitudinal_jerk_upper_bound;

  // Set optimizer instance
  auto ptr_interface = new PiecewiseJerkSpeedNonlinearIpoptInterface(
      s_init, s_dot_init, s_ddot_init, delta_t, num_of_knots, min_acceleration,
      max_acceleration, max_abs_jerk);
  // TODO(Jinyun): refactor state limits interface
  ptr_interface->set_constant_speed_limit(max_velocity);
  ptr_interface->set_s_max(total_length);
  ptr_interface->set_safety_bounds(s_bounds);
  // Set weights and reference values
  const auto& piecewise_jerk_nonlinear_speed_config =
      config_.piecewise_jerk_nonlinear_speed_config();

  PiecewiseJerkTrajectory1d smooth_curvature = SmoothPathCurvature(path_data);
  ptr_interface->set_curvature_curve(smooth_curvature);

  // TODO(Hongyi): add debug_info for speed_limit fitting curve
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();
  PiecewiseJerkTrajectory1d smooth_speed_limit = SmoothSpeedLimit(speed_limit);
  ptr_interface->set_speed_limit_curve(smooth_speed_limit);

  // TODO(Jinyun): evaluate the performance of piecewise linear warm start st
  // profile, will try piecewise polynomial st profile
  if (piecewise_jerk_nonlinear_speed_config.use_warm_start()) {
    std::vector<std::vector<double>> warm_start;
    SpeedData reference_speed_data = *speed_data;
    for (int i = 0; i < num_of_knots; ++i) {
      SpeedPoint sp;
      if (!reference_speed_data.EvaluateByTime(i * delta_t, &sp)) {
        std::string msg("retriving dp st data for speed optimization failed!");
        AERROR << msg;
        speed_data->clear();
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      warm_start.emplace_back(
          std::initializer_list<double>{sp.s(), sp.v(), sp.a()});
    }
    ptr_interface->set_warm_start(warm_start);
  }

  // TODO(Jinyun): will deprecate penalty towards dp st after refactoring on
  // safety distance keeping in speed decider is done
  std::vector<double> reference_spatial_distance;
  SpeedData reference_speed_data = *speed_data;
  for (int i = 0; i < num_of_knots; ++i) {
    SpeedPoint sp;
    if (!reference_speed_data.EvaluateByTime(i * delta_t, &sp)) {
      std::string msg("retriving dp st data for speed optimization failed!");
      AERROR << msg;
      speed_data->clear();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    reference_spatial_distance.push_back(sp.s());
  }
  ptr_interface->set_reference_spatial_distance(reference_spatial_distance);
  ptr_interface->set_w_reference_spatial_distance(
      piecewise_jerk_nonlinear_speed_config.ref_s_weight());

  ptr_interface->set_w_overall_a(
      piecewise_jerk_nonlinear_speed_config.acc_weight());
  ptr_interface->set_w_overall_j(
      piecewise_jerk_nonlinear_speed_config.jerk_weight());
  ptr_interface->set_w_overall_centripetal_acc(
      piecewise_jerk_nonlinear_speed_config.lat_acc_weight());
  ptr_interface->set_reference_speed(FLAGS_default_cruise_speed);
  ptr_interface->set_w_reference_speed(
      piecewise_jerk_nonlinear_speed_config.ref_v_weight());

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptr_interface;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("max_iter", 1000);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    std::string msg(
        "Piecewise jerk speed nonlinear optimizer failed during "
        "initialization!");
    AERROR << msg;
    speed_data->clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  status = app->OptimizeTNLP(problem);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "*** The optimization problem take time: " << diff.count() * 1000.0
         << " ms.";

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    ADEBUG << "*** The final value of the objective function is " << final_obj
           << '.';
  } else {
    const auto& ipopt_return_status =
        IpoptReturnStatus_Name(static_cast<IpoptReturnStatus>(status));
    if (ipopt_return_status.empty()) {
      AERROR << "Solver ends with unknown failure code: "
             << static_cast<int>(status);
    } else {
      AERROR << "Solver failure case: " << ipopt_return_status;
    }
    std::string msg("Piecewise jerk speed nonlinear optimizer failed!");
    AERROR << msg;
    speed_data->clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<double> s;
  std::vector<double> ds;
  std::vector<double> dds;
  ptr_interface->get_optimization_results(&s, &ds, &dds);

  for (int i = 0; i < num_of_knots; ++i) {
    ADEBUG << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i]
           << ", a = " << dds[i];
  }

  speed_data->clear();
  speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (int i = 1; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    if (ds[i] <= 0.0) {
      break;
    }
    speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i],
                                 (dds[i] - dds[i - 1]) / delta_t);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
  // Record speed_constraint
  auto speed_constraint =
      st_graph_data.mutable_st_graph_debug()->mutable_speed_constraint();
  for (int i = 0; i < num_of_knots; ++i) {
    double t = i * delta_t;
    speed_constraint->add_t(t);
    speed_constraint->add_upper_bound(smooth_speed_limit.Evaluate(0, s[i]));
  }
  return Status::OK();
}

PiecewiseJerkTrajectory1d
PiecewiseJerkSpeedNonlinearOptimizer::SmoothSpeedLimit(
    const SpeedLimit& speed_limit) {
  // using piecewise_jerk_path to fit a curve of speed_ref
  // TODO(Hongyi): move smooth configs to gflags
  double delta_s = 2.0;
  std::vector<double> speed_ref;
  for (int i = 0; i < 100; ++i) {
    double path_s = i * delta_s;
    double limit = speed_limit.GetSpeedLimitByS(path_s);
    speed_ref.emplace_back(limit);
  }
  std::array<double, 3> init_state = {speed_ref[0], 0.0, 0.0};
  PiecewiseJerkPathProblem piecewise_jerk_problem(speed_ref.size(), delta_s,
                                                  init_state);
  piecewise_jerk_problem.set_x_bounds(0.0, 50.0);
  piecewise_jerk_problem.set_dx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_ddx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_dddx_bound(-10.0, 10.0);

  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(10.0);
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);

  piecewise_jerk_problem.set_x_ref(1.0, speed_ref);
  bool success = piecewise_jerk_problem.Optimize(4000);
  std::vector<double> opt_x;
  std::vector<double> opt_dx;
  std::vector<double> opt_ddx;
  if (success) {
    opt_x = piecewise_jerk_problem.opt_x();
    opt_dx = piecewise_jerk_problem.opt_dx();
    opt_ddx = piecewise_jerk_problem.opt_ddx();
  }

  PiecewiseJerkTrajectory1d smooth_speed_limit(opt_x.front(), opt_dx.front(),
                                               opt_ddx.front());
  for (size_t i = 1; i < opt_ddx.size(); ++i) {
    double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
    smooth_speed_limit.AppendSegment(j, delta_s);
  }

  return smooth_speed_limit;
}

PiecewiseJerkTrajectory1d
PiecewiseJerkSpeedNonlinearOptimizer::SmoothPathCurvature(
    const PathData& path_data) {
  // using piecewise_jerk_path to fit a curve of path kappa profile
  // TODO(Jinyun): move smooth configs to gflags
  const auto& cartesian_path = path_data.discretized_path();
  const double delta_s = 0.5;
  std::vector<double> path_curvature;
  for (double path_s = cartesian_path.front().s();
       path_s < cartesian_path.back().s() + delta_s; path_s += delta_s) {
    const auto& path_point = cartesian_path.Evaluate(path_s);
    path_curvature.push_back(path_point.kappa());
  }
  const auto& path_init_point = cartesian_path.front();
  std::array<double, 3> init_state = {path_init_point.kappa(),
                                      path_init_point.dkappa(),
                                      path_init_point.ddkappa()};
  PiecewiseJerkPathProblem piecewise_jerk_problem(path_curvature.size(),
                                                  delta_s, init_state);
  piecewise_jerk_problem.set_x_bounds(-1.0, 1.0);
  piecewise_jerk_problem.set_dx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_ddx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_dddx_bound(-10.0, 10.0);

  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(10.0);
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);

  piecewise_jerk_problem.set_x_ref(10.0, path_curvature);
  bool success = piecewise_jerk_problem.Optimize(1000);
  std::vector<double> opt_x;
  std::vector<double> opt_dx;
  std::vector<double> opt_ddx;
  if (success) {
    opt_x = piecewise_jerk_problem.opt_x();
    opt_dx = piecewise_jerk_problem.opt_dx();
    opt_ddx = piecewise_jerk_problem.opt_ddx();
  }

  PiecewiseJerkTrajectory1d smooth_curvature(opt_x.front(), opt_dx.front(),
                                             opt_ddx.front());
  for (size_t i = 1; i < opt_ddx.size(); ++i) {
    double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
    smooth_curvature.AppendSegment(j, delta_s);
  }

  return smooth_curvature;
}

bool PiecewiseJerkSpeedNonlinearOptimizer::CheckStBoundFeasibility(
    const int num_of_knots, const double delta_t,
    const std::array<double, 3>& init_s,
    const std::vector<std::pair<double, double>>& s_bounds) {
  const auto start_timestamp = std::chrono::system_clock::now();

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);
  piecewise_jerk_problem.set_dx_bounds(
      0.0, std::fmax(FLAGS_planning_upper_speed_limit, init_s[1]));
  // TODO(Hongyi): delete this when ready to use vehicle_params
  piecewise_jerk_problem.set_ddx_bounds(-4.0, 2.0);
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                        FLAGS_longitudinal_jerk_upper_bound);
  piecewise_jerk_problem.set_x_bounds(s_bounds);

  // Set objective to be zeros to only utilize feasibility check feature
  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(0.0);
  piecewise_jerk_problem.set_weight_ddx(0.0);
  piecewise_jerk_problem.set_weight_dddx(0.0);

  // Solve the problem
  auto res = piecewise_jerk_problem.Optimize();

  const auto end_timestamp = std::chrono::system_clock::now();

  std::chrono::duration<double> time_diff_ms = end_timestamp - start_timestamp;

  ADEBUG << " feasibility_check takes : " << time_diff_ms.count() * 1000.0
         << " ms.";

  return res;
}
}  // namespace planning
}  // namespace apollo
