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

#include "modules/planning/planner/lattice/lattice_planner.h"

#include <memory>
#include <vector>

#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/behavior_decider/path_time_neighborhood.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/lattice/trajectory1d_generator/trajectory1d_generator.h"
#include "modules/planning/lattice/trajectory1d_generator/trajectory_evaluator.h"
#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"
#include "modules/planning/lattice/util/collision_checker.h"
#include "modules/planning/lattice/util/lattice_constraint_checker.h"
#include "modules/planning/lattice/util/lattice_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

LatticePlanner::LatticePlanner() {}

Status LatticePlanner::Init(const PlanningConfig& config) {
  return Status::OK();
}

Status LatticePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point,
    Frame* frame,
    ReferenceLineInfo* reference_line_info) {

  static std::size_t num_planning_cycles = 0;
  static std::size_t num_planning_succeeded_cycles = 0;

  AINFO << "";
  AINFO << "[BEGIN]-------------------------------------------------";
  AINFO << "Number of planning cycles:\t" << num_planning_cycles
      << "\t" << num_planning_succeeded_cycles;
  ++num_planning_cycles;

  // 1. obtain a reference line and transform it to the PathPoint format.
  auto discretized_reference_line = ToDiscretizedReferenceLine(
      reference_line_info->reference_line().reference_points());

  // 2. compute the matched point of the init planning point on the reference
  // line.
  PathPoint matched_point = ReferenceLineMatcher::MatchToReferenceLine(
      discretized_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());

  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;

  ComputeInitFrenetState(matched_point, planning_init_point, &init_s,
      &init_d);
  AINFO << "Step 1,2,3 Succeeded";

  // 4. parse the decision and get the planning target.
  PlanningTarget planning_target = decider_.Analyze(frame,
    planning_init_point,
    init_s,
    reference_line_info->reference_line(),
    discretized_reference_line);

  AINFO << "    [---planning_target---]: " << planning_target.decision_type();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator trajectory1d_generator;
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, init_s, init_d, &lon_trajectory1d_bundle,
      &lat_trajectory1d_bundle);

  // 6. first, evaluate the feasibility of the 1d trajectories according to
  // dynamic constraints.
  //   second, evaluate the feasible longitudinal and lateral trajectory pairs
  //   and sort them according to the cost.
  TrajectoryEvaluator trajectory_evaluator(
      planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle);

  AINFO << "number of trajectory pairs = "
        << trajectory_evaluator.num_of_trajectory_pairs();
  AINFO << "";

  AINFO << "Step 4,5,6 Succeeded";

  // Get instance of collision checker and constraint checker
  const std::vector<const Obstacle*>& obstacles = frame->obstacles();

  CollisionChecker collision_checker(obstacles);

  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  int constraint_failure_count = 0;
  int collision_failure_count = 0;

  planning_internal::Debug* ptr_debug = reference_line_info->mutable_debug();

  // put obstacles into debug data
  // Note : create prediction_obstacles since there is no longer original
  // data exposed. WTF
  // Hence, there might be obstacles with same id but different trajectory

  for (uint i = 0; i < obstacles.size(); ++i) {
    const Obstacle* obstacle_ptr = obstacles[i];
    apollo::prediction::PredictionObstacle* prediction_obstacle =
        ptr_debug->mutable_planning_data()->add_prediction_obstacle();
    prediction_obstacle->mutable_perception_obstacle()->CopyFrom(
        obstacle_ptr->Perception());
    prediction_obstacle->add_trajectory()->CopyFrom(obstacle_ptr->Trajectory());
  }

  AINFO << "Step Debug Succeeded";

  int num_lattice_traj = 0;
  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost = 0.0;
    trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    if (!LatticeConstraintChecker::IsValidTrajectoryPair(
            *lat_trajectory1d_bundle[trajectory_pair.second],
            *lon_trajectory1d_bundle[trajectory_pair.first])) {
      ++constraint_failure_count;
      AINFO << "------continued from ConstraintChecker";
      continue;
    }

    auto combined_trajectory = CombineTrajectory(discretized_reference_line,
        *lon_trajectory1d_bundle[trajectory_pair.first],
        *lat_trajectory1d_bundle[trajectory_pair.second],
        planning_init_point.relative_time());
    AINFO << "------(1)combined trajectory";

    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      AINFO << "------continued from CollisionChecker";
      continue;
    }

    // put combine trajectory into debug data
    ++num_lattice_traj;
    const std::vector<common::TrajectoryPoint>& combined_trajectory_points =
        combined_trajectory.trajectory_points();

    auto combined_trajectory_path =
        ptr_debug->mutable_planning_data()->add_trajectory_path();
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      combined_trajectory_path->add_trajectory_point()->CopyFrom(
          combined_trajectory_points[i]);
    }
    combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);

    AINFO << "------(2)set lattice trajectory";
    // AINFO << "trajectory not valid for constraint ["
    //          << constraint_failure_count << "] times";
    // AINFO << "trajectory not valid for collision ["
    //          << collision_failure_count << "] times";

    if (num_lattice_traj == 1) {
      reference_line_info->SetTrajectory(combined_trajectory);
    }

    if (num_lattice_traj == FLAGS_num_lattice_traj_to_plot) {
      break;
    }
  }

  AINFO << "Step CombineTrajectory Succeeded";

  AINFO << "trajectory not valid for constraint ["
            << constraint_failure_count << "] times";
  AINFO << "trajectory not valid for collision ["
            << collision_failure_count << "] times";
  if (num_lattice_traj > 0) {
    AINFO << "Planning succeeded";
    num_planning_succeeded_cycles += 1;
    AINFO << "[END]-------------------------------------------------";
    AINFO << "";
    reference_line_info->SetDriable(true);
    return Status::OK();
  } else {
    AINFO << "Planning failed";
    AINFO << "[END]-------------------------------------------------";
    AINFO << "";
    return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
  }
}

DiscretizedTrajectory LatticePlanner::CombineTrajectory(
    const std::vector<PathPoint>& reference_line, const Curve1d& lon_trajectory,
    const Curve1d& lat_trajectory, const double init_relative_time) const {
  DiscretizedTrajectory combined_trajectory;

  double s0 = lon_trajectory.Evaluate(0, 0.0);
  double s_ref_max = reference_line.back().s();
  double t_param_max = lon_trajectory.param_length();
  double s_param_max = lat_trajectory.param_length();

  double t_param = 0.0;
  while (t_param < planned_trajectory_time) {
    double s = 0.0;
    double s_dot = 0.0;
    double s_ddot = 0.0;

    if (t_param < t_param_max) {
      s = lon_trajectory.Evaluate(0, t_param);
      s_dot = lon_trajectory.Evaluate(1, t_param);
      s_ddot = lon_trajectory.Evaluate(2, t_param);
    } else {
      s_dot = lon_trajectory.Evaluate(1, t_param_max);
      s = lon_trajectory.Evaluate(0, t_param_max) +
          (t_param - t_param_max) * s_dot;
      s_ddot = 0.0;
    }

    if (s > s_ref_max) {
      break;
    }

    double s_param = s - s0;
    double d = 0.0;
    double d_prime = 0.0;
    double d_pprime = 0.0;

    if (s_param < s_param_max) {
      d = lat_trajectory.Evaluate(0, s_param);
      d_prime = lat_trajectory.Evaluate(1, s_param);
      d_pprime = lat_trajectory.Evaluate(2, s_param);
    } else {
      d = lat_trajectory.Evaluate(0, s_param_max);
      d_prime = 0.0;
      d_pprime = 0.0;
    }

    PathPoint matched_ref_point =
        ReferenceLineMatcher::MatchToReferenceLine(reference_line, s);

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s();
    const double rx = matched_ref_point.x();
    const double ry = matched_ref_point.y();
    const double rtheta = matched_ref_point.theta();
    const double rkappa = matched_ref_point.kappa();
    const double rdkappa = matched_ref_point.dkappa();

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
    CartesianFrenetConverter::frenet_to_cartesian(
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
        &theta, &kappa, &v, &a);

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(x);
    trajectory_point.mutable_path_point()->set_y(y);
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.mutable_path_point()->set_kappa(kappa);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(t_param + init_relative_time);

    combined_trajectory.AppendTrajectoryPoint(trajectory_point);

    t_param = t_param + trajectory_time_resolution;
  }
  return combined_trajectory;
}

}  // namespace planning
}  // namespace apollo
