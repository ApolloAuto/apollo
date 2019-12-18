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

/**
 * @file
 **/

#include "modules/planning/planner/lattice/lattice_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/lattice/behavior/path_time_graph.h"
#include "modules/planning/lattice/behavior/prediction_querier.h"
#include "modules/planning/lattice/trajectory_generation/backup_trajectory_generator.h"
#include "modules/planning/lattice/trajectory_generation/lattice_trajectory1d.h"
#include "modules/planning/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/lattice/trajectory_generation/trajectory_combiner.h"
#include "modules/planning/lattice/trajectory_generation/trajectory_evaluator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::common::time::Clock;

namespace {

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
      cartesian_state.path_point().x(), cartesian_state.path_point().y(),
      cartesian_state.v(), cartesian_state.a(),
      cartesian_state.path_point().theta(),
      cartesian_state.path_point().kappa(), ptr_s, ptr_d);
}

}  // namespace

Status LatticePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame,
                            ADCTrajectory* ptr_computed_trajectory) {
  size_t success_line_count = 0;
  size_t index = 0;
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (index != 0) {
      reference_line_info.SetPriorityCost(
          FLAGS_cost_non_priority_reference_line);
    } else {
      reference_line_info.SetPriorityCost(0.0);
    }
    auto status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    if (status != Status::OK()) {
      if (reference_line_info.IsChangeLanePath()) {
        AERROR << "Planner failed to change lane to "
               << reference_line_info.Lanes().Id();
      } else {
        AERROR << "Planner failed to " << reference_line_info.Lanes().Id();
      }
    } else {
      success_line_count += 1;
    }
    ++index;
  }

  if (success_line_count > 0) {
    return Status::OK();
  }
  return Status(ErrorCode::PLANNING_ERROR,
                "Failed to plan on any reference line.");
}

Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  static size_t num_planning_cycles = 0;
  static size_t num_planning_succeeded_cycles = 0;

  double start_time = Clock::NowInSeconds();
  double current_time = start_time;

  ADEBUG << "Number of planning cycles: " << num_planning_cycles << " "
         << num_planning_succeeded_cycles;
  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
  // 1. obtain a reference line and transform it to the PathPoint format.
  auto ptr_reference_line =
      std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
          reference_line_info->reference_line().reference_points()));

  // 2. compute the matched point of the init planning point on the reference
  // line.
  PathPoint matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());

  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  ADEBUG << "ReferenceLine and Frenet Conversion Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
      frame->obstacles(), ptr_reference_line);

  // 4. parse the decision and get the planning target.
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
      reference_line_info, init_s[0],
      init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
      FLAGS_trajectory_time_length, init_d);

  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(speed_limit);

  PlanningTarget planning_target = reference_line_info->planning_target();
  if (planning_target.has_stop_point()) {
    ADEBUG << "Planning target stop s: " << planning_target.stop_point().s()
           << "Current ego s: " << init_s[0];
  }

  ADEBUG << "Decision_Time = " << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  ADEBUG << "Trajectory_Generation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // 6. first, evaluate the feasibility of the 1d trajectories according to
  // dynamic constraints.
  //   second, evaluate the feasible longitudinal and lateral trajectory pairs
  //   and sort them according to the cost.
  TrajectoryEvaluator trajectory_evaluator(
      init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      ptr_path_time_graph, ptr_reference_line);

  ADEBUG << "Trajectory_Evaluator_Construction_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  ADEBUG << "number of trajectory pairs = "
         << trajectory_evaluator.num_of_trajectory_pairs()
         << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
         << "  number_lat_traj = " << lat_trajectory1d_bundle.size();

  // Get instance of collision checker and constraint checker
  CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
                                     *ptr_reference_line, reference_line_info,
                                     ptr_path_time_graph);

  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  size_t constraint_failure_count = 0;
  size_t collision_failure_count = 0;
  size_t combined_constraint_failure_count = 0;

  size_t lon_vel_failure_count = 0;
  size_t lon_acc_failure_count = 0;
  size_t lon_jerk_failure_count = 0;
  size_t curvature_failure_count = 0;
  size_t lat_acc_failure_count = 0;
  size_t lat_jerk_failure_count = 0;

  size_t num_lattice_traj = 0;

  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());

    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          lon_vel_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          lon_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          lon_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          curvature_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          lat_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          lat_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
      }
      continue;
    }

    // check collision with other obstacles
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }

    // put combine trajectory into debug data
    const auto& combined_trajectory_points = combined_trajectory;
    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost);
    reference_line_info->SetDrivable(true);

    // Print the chosen end condition and start condition
    ADEBUG << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1]
           << " dds = " << init_s[2];
    // cast
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      ADEBUG << "Dynamically casting trajectory1d ptr. failed.";
    }

    if (lattice_traj_ptr->has_target_position()) {
      ADEBUG << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
             << " ds = " << lattice_traj_ptr->target_velocity()
             << " t = " << lattice_traj_ptr->target_time();
    }

    ADEBUG << "InputPose";
    ADEBUG << "XY: " << planning_init_point.ShortDebugString();
    ADEBUG << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2]
           << ")";
    ADEBUG << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2]
           << ")";

    ADEBUG << "Reference_line_priority_cost = "
           << reference_line_info->PriorityCost();
    ADEBUG << "Total_Trajectory_Cost = " << trajectory_pair_cost;
    ADEBUG << "OutputTrajectory";
    for (uint i = 0; i < 10; ++i) {
      ADEBUG << combined_trajectory_points[i].ShortDebugString();
    }

    break;
    /*
    auto combined_trajectory_path =
        ptr_debug->mutable_planning_data()->add_trajectory_path();
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      combined_trajectory_path->add_trajectory_point()->CopyFrom(
          combined_trajectory_points[i]);
    }
    combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);
    */
  }

  ADEBUG << "Trajectory_Evaluation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;

  ADEBUG << "Step CombineTrajectory Succeeded";

  ADEBUG << "1d trajectory not valid for constraint ["
         << constraint_failure_count << "] times";
  ADEBUG << "Combined trajectory not valid for ["
         << combined_constraint_failure_count << "] times";
  ADEBUG << "Trajectory not valid for collision [" << collision_failure_count
         << "] times";
  ADEBUG << "Total_Lattice_Planning_Frame_Time = "
         << (Clock::NowInSeconds() - start_time) * 1000;

  if (num_lattice_traj > 0) {
    ADEBUG << "Planning succeeded";
    num_planning_succeeded_cycles += 1;
    reference_line_info->SetDrivable(true);
    return Status::OK();
  } else {
    AERROR << "Planning failed";
    if (FLAGS_enable_backup_trajectory) {
      AERROR << "Use backup trajectory";
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time(),
          std::make_shared<CollisionChecker>(collision_checker),
          &trajectory1d_generator);
      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

      reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
      reference_line_info->SetTrajectory(trajectory);
      reference_line_info->SetDrivable(true);
      return Status::OK();

    } else {
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    }
    return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
  }
}

}  // namespace planning
}  // namespace apollo
