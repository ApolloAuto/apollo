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

#include "modules/planning/lattice/trajectory_generation/trajectory1d_generator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "lateral_trajectory_optimizer_interface.h"
#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/trajectory1d/constant_deceleration_trajectory1d.h"
#include "modules/planning/lattice/trajectory1d/standing_still_trajectory1d.h"

namespace apollo {
namespace planning {

// A common function for trajectory bundles generation with
// a given initial state and  end conditions.
typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

Trajectory1dGenerator::Trajectory1dGenerator(
    const State& lon_init_state,
    const State& lat_init_state,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier)
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      ptr_path_time_graph_(ptr_path_time_graph),
      end_condition_sampler_(lon_init_state, lat_init_state,
                             ptr_path_time_graph, ptr_prediction_querier) {}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle,
    Trajectory1DBundle* ptr_lat_trajectory_bundle) {
  GenerateLongitudinalTrajectoryBundle(planning_target,
                                       ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
  return;
}

void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  ADEBUG << "cruise speed is  " << target_speed;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);
  if (end_conditions.empty()) {
    return;
  }

  // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
  // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
  // invoke the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
    const double stop_point,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  ADEBUG << "stop point is " << stop_point;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForStopping(stop_point);
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForPathTimeObstacles(
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForPathTimePoints();
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  // cruising trajectories are planned regardlessly.
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed(),
                                   ptr_lon_trajectory_bundle);

  GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);

  if (planning_target.has_stop_point()) {
    GenerateSpeedProfilesForStopping(planning_target.stop_point().s(),
                                     ptr_lon_trajectory_bundle);
  }
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    Trajectory1DBundle* ptr_lat_trajectory_bundle) const {

  if (FLAGS_lateral_optimization) {
    auto end_conditions = end_condition_sampler_.SampleLatEndConditions();

    // Use the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<5>(init_lat_state_, end_conditions,
        ptr_lat_trajectory_bundle);
  } else {
//    double delta_s = 0.5;
//    auto lateral_bounds = ptr_path_time_graph_->GetLateralBounds(
//        0.0, 100.0, delta_s);
//    LateralTrajectoryOptimizerInterface lateral_trajectory_optimizer(
//        init_lat_state_[0], init_lat_state_[1], init_lat_state_[2],
//        delta_s, FLAGS_lateral_third_order_derivative_max, lateral_bounds);
//
//    lateral_trajectory_optimizer.set_objective_weights(FLAGS_weight_lateral_offset,
//        FLAGS_weight_lateral_derivative, FLAGS_weight_lateral_second_order_derivative,
//        FLAGS_weight_lateral_obstacle_distance);

  }
}

}  // namespace planning
}  // namespace apollo
