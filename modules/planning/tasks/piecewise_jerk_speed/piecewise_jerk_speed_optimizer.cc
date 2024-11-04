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

#include <algorithm>

#include <string>
#include <utility>
#include <vector>
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

bool PiecewiseJerkSpeedOptimizer::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!SpeedOptimizer::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config_ this task.
  return SpeedOptimizer::LoadConfig<PiecewiseJerkSpeedOptimizerConfig>(
      &config_);
}

Status PiecewiseJerkSpeedOptimizer::Process(const PathData& path_data,
                                            const TrajectoryPoint& init_point,
                                            SpeedData* const speed_data) {
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }

  ACHECK(speed_data != nullptr);
  SpeedData reference_speed_data = *speed_data;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
  PrintCurves print_debug;
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                                  st_graph_data.init_point().a()};
  const auto& vehicle_state = frame_->vehicle_state();
  if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
    init_s[1] = std::max(-init_s[1], 0.0);
    init_s[2] = -init_s[2];
    AINFO << "transfer reverse speed" << init_s[0] << "," << init_s[1] << ","
          << init_s[2];
  }
  double delta_t = 0.1;
  double total_length = st_graph_data.path_length();
  double total_time = st_graph_data.total_time_by_conf();
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
  print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
  print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
  print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);
  // Update STBoundary
  const double kEpsilon = 0.01;
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
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;
        default:
          break;
      }
    }
    s_upper_bound = std::fmax(s_upper_bound, s_lower_bound + kEpsilon);
    print_debug.AddPoint("st_bounds_lower", curr_t, s_lower_bound);
    print_debug.AddPoint("st_bounds_upper", curr_t, s_upper_bound);
    if (s_lower_bound > s_upper_bound) {
      const std::string msg =
          "s_lower_bound larger than s_upper_bound on STGraph";
      AERROR << msg;
      speed_data->clear();
      print_debug.PrintToLog();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }

  // Update SpeedBoundary and ref_s
  std::vector<double> x_ref(num_of_knots, total_length);
  std::vector<double> dx_ref(num_of_knots,
                             reference_line_info_->GetCruiseSpeed());
  std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());
  std::vector<double> penalty_dx;
  std::vector<std::pair<double, double>> s_dot_bounds;
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    // get path_s
    SpeedPoint sp;
    reference_speed_data.EvaluateByTime(curr_t, &sp);
    const double path_s = sp.s();
    x_ref[i] = path_s;
    // get curvature
    PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
    penalty_dx.push_back(std::fabs(path_point.kappa()) *
                         config_.kappa_penalty_weight());
    // get v_upper_bound
    const double v_lower_bound = 0.0;
    double v_upper_bound = FLAGS_planning_upper_speed_limit;
    v_upper_bound =
        std::fmin(speed_limit.GetSpeedLimitByS(path_s), v_upper_bound);
    dx_ref[i] = std::fmin(v_upper_bound, dx_ref[i]);
    s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
    print_debug.AddPoint("st_reference_line", curr_t, x_ref[i]);
    print_debug.AddPoint("st_penalty_dx", curr_t, penalty_dx.back());
    print_debug.AddPoint("vt_reference_line", curr_t, dx_ref[i]);
    print_debug.AddPoint("vt_weighting", curr_t, dx_ref_weight[i]);
    print_debug.AddPoint("vt_boundary_lower", curr_t, v_lower_bound);
    print_debug.AddPoint("sv_boundary_lower", path_s, v_lower_bound);
    print_debug.AddPoint("sk_curve", path_s, path_point.kappa());
    print_debug.AddPoint("vt_boundary_upper", curr_t, v_upper_bound);
    print_debug.AddPoint("sv_boundary_upper", path_s, v_upper_bound);
  }
  AdjustInitStatus(s_dot_bounds, delta_t, init_s);
  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);
  piecewise_jerk_problem.set_weight_ddx(config_.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(config_.jerk_weight());
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
  piecewise_jerk_problem.set_x_bounds(0.0, total_length);
  piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                        veh_param.max_acceleration());
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                        FLAGS_longitudinal_jerk_upper_bound);
  piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));
  piecewise_jerk_problem.set_dx_ref(dx_ref_weight, dx_ref);
  piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));
  piecewise_jerk_problem.set_penalty_dx(penalty_dx);
  piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    const std::string msg = "Piecewise jerk speed optimizer failed!";
    AERROR << msg << ".try to fallback.";
    piecewise_jerk_problem.set_dx_bounds(
        0.0, std::fmax(FLAGS_planning_upper_speed_limit,
                       st_graph_data.init_point().v()));
    if (!FLAGS_speed_optimize_fail_relax_velocity_constraint ||
        !piecewise_jerk_problem.Optimize()) {
      speed_data->clear();
      print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
      print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
      print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);
      AINFO << "jerk_bound: " << FLAGS_longitudinal_jerk_lower_bound << ","
            << FLAGS_longitudinal_jerk_upper_bound;
      AINFO << "acc bound: " << veh_param.max_deceleration() << ","
            << veh_param.max_acceleration();
      print_debug.PrintToLog();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  // Extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
  for (int i = 0; i < num_of_knots; ++i) {
    ADEBUG << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i]
           << ", a = " << dds[i];
    print_debug.AddPoint("optimize_st_curve", i * delta_t, s[i]);
    print_debug.AddPoint("optimize_vt_curve", i * delta_t, ds[i]);
    print_debug.AddPoint("optimize_at_curve", i * delta_t, dds[i]);
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
  print_debug.PrintToLog();
  return Status::OK();
}
void PiecewiseJerkSpeedOptimizer::AdjustInitStatus(
    const std::vector<std::pair<double, double>> s_dot_bound, double delta_t,
    std::array<double, 3>& init_s) {
  double v_min = init_s[1];
  double v_max = init_s[1];
  double a_min = init_s[2];
  double a_max = init_s[2];
  double last_a_min = 0;
  double last_a_max = 0;
  for (size_t i = 1; i < s_dot_bound.size(); i++) {
    last_a_min = a_min;
    last_a_max = a_max;
    a_min = a_min + delta_t * FLAGS_longitudinal_jerk_upper_bound;
    a_max = a_max + delta_t * FLAGS_longitudinal_jerk_lower_bound;
    v_min = v_min + 0.5 * delta_t * (a_min + last_a_min);
    v_max = v_max + 0.5 * delta_t * (a_max + last_a_max);
    if (v_min < s_dot_bound[i].first || v_max > s_dot_bound[i].second) {
      AWARN << "init state not appropriate in" << i << "," << v_min << ","
            << v_max << "adjust acc to 0 in init state " << init_s[0] << ","
            << init_s[1] << "," << init_s[2];
      init_s[2] = 0;
      return;
    }
  }
}
}  // namespace planning
}  // namespace apollo
