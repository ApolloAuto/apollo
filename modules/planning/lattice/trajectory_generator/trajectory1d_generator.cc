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

#include "modules/planning/lattice/trajectory_generator/trajectory1d_generator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/trajectory1d/constant_deceleration_trajectory1d.h"
#include "modules/planning/lattice/trajectory1d/standing_still_trajectory1d.h"
#include "modules/planning/lattice/trajectory1d/lattice_trajectory1d.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

Trajectory1dGenerator::Trajectory1dGenerator(
    const std::array<double, 3>& lon_init_state,
    const std::array<double, 3>& lat_init_state,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier)
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      end_condition_sampler_(lon_init_state, lat_init_state,
                             FLAGS_planning_upper_speed_limit,
                             ptr_path_time_graph,
                             ptr_prediction_querier) {
}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) {
  GenerateLongitudinalTrajectoryBundle(planning_target,
                                       ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
  return;
}

void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  ADEBUG << "cruise speed is  " << target_speed;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);

  for (const auto& end_condition : end_conditions) {
    // Only the last two elements in the end_condition are useful.
    std::array<double, 2> end_state;
    end_state[0] = end_condition.first[1];
    end_state[1] = end_condition.first[2];

    std::shared_ptr<LatticeTrajectory1d> lattice_traj_ptr(
        new LatticeTrajectory1d(
            std::shared_ptr<Curve1d>(new QuarticPolynomialCurve1d(
                init_lon_state_, end_state, end_condition.second))));

    lattice_traj_ptr->set_target_velocity(end_condition.first[1]);
    lattice_traj_ptr->set_target_time(end_condition.second);
    ptr_lon_trajectory_bundle->push_back(lattice_traj_ptr);
  }
}

void Trajectory1dGenerator::GenerateSpeedProfileForStopping(
    const double stop_point,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  ADEBUG << "stop point is " << stop_point;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForStopping(stop_point);

  for (const auto& end_condition : end_conditions) {
    std::shared_ptr<LatticeTrajectory1d> lattice_traj_ptr(
        new LatticeTrajectory1d(
            std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
                init_lon_state_, end_condition.first, end_condition.second))));

    lattice_traj_ptr->set_target_position(end_condition.first[0]);
    lattice_traj_ptr->set_target_velocity(end_condition.first[1]);
    lattice_traj_ptr->set_target_time(end_condition.second);
    ptr_lon_trajectory_bundle->push_back(lattice_traj_ptr);
  }
}

void Trajectory1dGenerator::GenerateSpeedProfilesForPathTimeObstacles(
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForPathTimePoints();

  for (const auto& end_condition : end_conditions) {
    std::shared_ptr<LatticeTrajectory1d> lattice_traj_ptr(
        new LatticeTrajectory1d(
            std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
                init_lon_state_, end_condition.first, end_condition.second))));

    lattice_traj_ptr->set_target_position(end_condition.first[0]);
    lattice_traj_ptr->set_target_velocity(end_condition.first[1]);
    lattice_traj_ptr->set_target_time(end_condition.second);
    ptr_lon_trajectory_bundle->push_back(lattice_traj_ptr);
  }
}

void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  // cruising trajectories are planned regardlessly.
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed(),
                                   ptr_lon_trajectory_bundle);

  GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);

  if (planning_target.has_stop_point()) {
    GenerateSpeedProfileForStopping(planning_target.stop_point().s(),
                                    ptr_lon_trajectory_bundle);
  }
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) const {
  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_.SampleLatEndConditions();

  for (const auto& end_condition : end_conditions) {
    std::shared_ptr<LatticeTrajectory1d> lattice_traj_ptr(
        new LatticeTrajectory1d(
            std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
                init_lat_state_, end_condition.first, end_condition.second))));

    lattice_traj_ptr->set_target_position(end_condition.first[0]);
    lattice_traj_ptr->set_target_velocity(end_condition.first[1]);
    lattice_traj_ptr->set_target_time(end_condition.second);
    ptr_lat_trajectory_bundle->push_back(lattice_traj_ptr);
  }
}

}  // namespace planning
}  // namespace apollo
