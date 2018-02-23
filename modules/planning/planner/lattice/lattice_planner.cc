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

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "modules/planning/lattice/behavior_decider/path_time_graph.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/lattice/trajectory_generator/backup_trajectory_generator.h"
#include "modules/planning/lattice/trajectory_generator/trajectory1d_generator.h"
#include "modules/planning/lattice/trajectory_generator/trajectory_combiner.h"
#include "modules/planning/lattice/trajectory_generator/trajectory_evaluator.h"
#include "modules/planning/lattice/util/lattice_trajectory1d.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterManager;
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

    double dx = 0.0;
    double dy = 0.0;
    if (!path_points.empty()) {
      dx = path_point.x() - path_points.back().x();
      dy = path_point.y() - path_points.back().y();
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

LatticePlanner::LatticePlanner() {}

Status LatticePlanner::Init(const PlanningConfig& config) {
  return Status::OK();
}

Status LatticePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame) {
  auto status = Status::OK();
  double priority_cost = 0.0;
  bool first_reference_line = true;
  for (auto& reference_line_info : frame->reference_line_info()) {
    reference_line_info.SetPriorityCost(priority_cost);
    status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
    if (status != Status::OK()) {
      if (reference_line_info.IsChangeLanePath()) {
        AERROR << "Planner failed to change lane to "
               << reference_line_info.Lanes().Id();
      } else {
        AERROR << "Planner failed to " << reference_line_info.Lanes().Id();
      }
    }
    if (first_reference_line) {
      priority_cost += FLAGS_priority_cost_gap;
      first_reference_line = false;
    }
  }
  return status;
}

Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  static std::size_t num_planning_cycles = 0;
  static std::size_t num_planning_succeeded_cycles = 0;

  double start_time = Clock::NowInSeconds();
  double current_time = start_time;

  ADEBUG << "Number of planning cycles: " << num_planning_cycles << " "
         << num_planning_succeeded_cycles;
  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
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
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  ADEBUG << "ReferenceLine and Frenet Conversion Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // 4. parse the decision and get the planning target.
  std::shared_ptr<PathTimeGraph> path_time_neighborhood_ptr(new PathTimeGraph(
      frame->obstacles(), init_s[0], discretized_reference_line));

  decider_.UpdatePathTimeGraph(path_time_neighborhood_ptr);
  PlanningTarget planning_target =
      decider_.Analyze(frame, reference_line_info, planning_init_point, init_s,
                       discretized_reference_line);

  ADEBUG << "Decision_Time = " << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
  Trajectory1dGenerator trajectory1d_generator(init_s, init_d);
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
      planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      FLAGS_enable_auto_tuning, path_time_neighborhood_ptr);

  ADEBUG << "Trajectory_Evaluator_Construction_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  ADEBUG << "number of trajectory pairs = "
         << trajectory_evaluator.num_of_trajectory_pairs()
         << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
         << "  number_lat_traj = " << lat_trajectory1d_bundle.size();

  // Get instance of collision checker and constraint checker
  CollisionChecker collision_checker(frame->obstacles(), init_s, init_d,
                                     discretized_reference_line);

  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  std::size_t constraint_failure_count = 0;
  std::size_t collision_failure_count = 0;
  std::size_t combined_constraint_failure_count = 0;

  // planning_internal::Debug* ptr_debug = reference_line_info->mutable_debug();

  int num_lattice_traj = 0;
  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();
    // For auto tuning
    if (FLAGS_enable_auto_tuning) {
      std::vector<double> trajectory_pair_cost_components =
          trajectory_evaluator.top_trajectory_pair_component_cost();
      ADEBUG << "TrajectoryPairComponentCost";
      ADEBUG << "travel_cost = " << trajectory_pair_cost_components[0];
      ADEBUG << "jerk_cost = " << trajectory_pair_cost_components[1];
      ADEBUG << "obstacle_cost = " << trajectory_pair_cost_components[2];
      ADEBUG << "lateral_cost = " << trajectory_pair_cost_components[3];
    }
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = TrajectoryCombiner::Combine(
        discretized_reference_line, *trajectory_pair.first,
        *trajectory_pair.second, planning_init_point.relative_time());

    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    if (!ConstraintChecker::ValidTrajectory(combined_trajectory)) {
      ++combined_constraint_failure_count;
      continue;
    }

    // check collision with other obstacles
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }

    // put combine trajectory into debug data
    const auto& combined_trajectory_points =
        combined_trajectory.trajectory_points();
    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost);
    reference_line_info->SetDrivable(true);

    // Auto Tuning
    if (FLAGS_enable_auto_tuning) {
      if (AdapterManager::GetLocalization() == nullptr) {
        AERROR << "Auto tuning failed since no localization is available.";
      } else {
        // 1. Get future trajectory from localization
        DiscretizedTrajectory future_trajectory = GetFutureTrajectory();
        // 2. Map future trajectory to lon-lat trajectory pair
        std::vector<common::SpeedPoint> lon_future_trajectory;
        std::vector<common::FrenetFramePoint> lat_future_trajectory;
        if (!MapFutureTrajectoryToSL(future_trajectory, &lon_future_trajectory,
                                     &lat_future_trajectory,
                                     reference_line_info)) {
          AERROR << "Auto tuning failed since no mapping "
                 << "from future trajectory to lon-lat";
        }
        // 3. evaluate cost
        std::vector<double> future_trajectory_component_cost =
            trajectory_evaluator.evaluate_per_lonlat_trajectory(
                planning_target, lon_future_trajectory, lat_future_trajectory);

        // 4. emit
      }
    }

    // Print the chosen end condition and start condition
    ADEBUG << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1]
           << " dds = " << init_s[2];
    // cast
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      ADEBUG << "Dynamically casting trajectory1d ptr. failed.";
    }
    ADEBUG << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
           << " ds = " << lattice_traj_ptr->target_velocity()
           << " t = " << lattice_traj_ptr->target_time();

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
  current_time = Clock::NowInSeconds();

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
    if (FLAGS_enable_backup_trajectory &&
        !reference_line_info->IsChangeLanePath()) {
      AERROR << "Use backup trajectory";
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time(),
          &trajectory1d_generator);
      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(
              discretized_reference_line);
      reference_line_info->SetCost(FLAGS_backup_trajectory_cost);
      reference_line_info->SetTrajectory(trajectory);
      reference_line_info->SetDrivable(true);
      return Status::OK();

    } else {
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    }
    return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
  }
}

DiscretizedTrajectory LatticePlanner::GetFutureTrajectory() const {
  // localization
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();
  std::vector<TrajectoryPoint> traj_pts;
  for (const auto& traj_pt : localization.trajectory_point()) {
    traj_pts.emplace_back(traj_pt);
  }
  DiscretizedTrajectory ret(traj_pts);
  return ret;
}

bool LatticePlanner::MapFutureTrajectoryToSL(
    const DiscretizedTrajectory& future_trajectory,
    std::vector<apollo::common::SpeedPoint>* st_points,
    std::vector<apollo::common::FrenetFramePoint>* sl_points,
    ReferenceLineInfo* reference_line_info) {
  return false;
}

}  // namespace planning
}  // namespace apollo
