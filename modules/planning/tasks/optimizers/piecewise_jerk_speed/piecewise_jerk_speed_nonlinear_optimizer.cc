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

  // Set weights and reference values
  const auto& piecewise_jerk_nonlinear_speed_config =
      config_.piecewise_jerk_nonlinear_speed_config();

  ptr_interface->set_path(path_data);

  // TODO(Jinyun): add speed_limit fitting curve
  //   ptr_interface->set_speed_limit_curve(speed_limit);

  // TODO(Jinyun): add ref_s into optimizer
  ptr_interface->set_w_overall_a(
      piecewise_jerk_nonlinear_speed_config.acc_weight());
  ptr_interface->set_w_overall_j(
      piecewise_jerk_nonlinear_speed_config.jerk_weight());
  ptr_interface->set_w_overall_centripetal_acc(500.0);
  ptr_interface->set_reference_speed(FLAGS_default_cruise_speed);
  ptr_interface->set_w_reference_speed(
      piecewise_jerk_nonlinear_speed_config.ref_v_weight());

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptr_interface;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("max_iter", 5000);

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
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
