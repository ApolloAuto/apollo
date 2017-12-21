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

#include "modules/planning/lattice/trajectory1d_generator/trajectory1d_generator.h"

#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>

#include "modules/planning/lattice/trajectory1d/standing_still_trajectory1d.h"
#include "modules/planning/lattice/trajectory1d/constant_deceleration_trajectory1d.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/util/lattice_quartic_polynomial_curve1d.h"
#include "modules/planning/lattice/util/lattice_quintic_polynomial_curve1d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/log.h"

namespace apollo {
namespace planning {

Trajectory1dGenerator::Trajectory1dGenerator(
    const std::array<double, 3>& lon_init_state,
    const std::array<double, 3>& lat_init_state) : init_lon_state_(lon_init_state),
        init_lat_state_(lat_init_state) {
  end_condition_sampler_ = new EndConditionSampler(
      lon_init_state, lat_init_state, speed_limit);
}

Trajectory1dGenerator::~Trajectory1dGenerator() {
  delete end_condition_sampler_;
}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) {

  /**
  if (planning_target.decision_type() == PlanningTarget::STOP) {
    double stop_position = planning_target.stop_point();
    double distance = stop_position - init_lon_state_[0];
    double s_dot = init_lon_state_[1];

    // if the stop point is close enough and vehicle speed is close to zero.
    if (distance < stop_margin && s_dot < stop_speed_threshold) {
      AINFO << "Lattice planner stop handling: use standing still trajectory";
      std::shared_ptr<Curve1d> ptr_lon_trajectory =
          std::shared_ptr<Curve1d>(new StandingStillTrajectory1d(
              init_lon_state_[0], planned_trajectory_time));
      ptr_lon_trajectory_bundle->push_back(ptr_lon_trajectory);

      std::shared_ptr<Curve1d> ptr_lat_trajectory =
          std::shared_ptr<Curve1d>(new StandingStillTrajectory1d(
              init_lat_state_[0], planned_trajectory_time));
      ptr_lat_trajectory_bundle->push_back(ptr_lat_trajectory);
      return;
    }

    // if the stop point is close enough and vehicle speed is slow, e.g., < 0.5
    // m/s.
    if ((distance < stop_margin && s_dot < low_speed_threshold) || distance < 0.0) {
      AINFO << "Lattice planner stop handling: "
                "use constant deceleration trajectory";

      double deceleration = 0.0;
      if (distance <= 0.0) {
        deceleration =
            low_speed_threshold * low_speed_threshold / stop_margin * 0.5;
      } else {
        deceleration = std::min(
            s_dot * s_dot / distance * 0.5,
            low_speed_threshold * low_speed_threshold / stop_margin * 0.5);
      }

      std::shared_ptr<Curve1d> ptr_lon_trajectory =
          std::shared_ptr<Curve1d>(new ConstantDecelerationTrajectory1d(
              init_lon_state_[0], init_lon_state_[1], -deceleration));
      ptr_lon_trajectory_bundle->push_back(ptr_lon_trajectory);

      std::shared_ptr<Curve1d> ptr_lat_trajectory =
          std::shared_ptr<Curve1d>(new StandingStillTrajectory1d(
              init_lat_state_[0], planned_trajectory_time));
      ptr_lat_trajectory_bundle->push_back(ptr_lat_trajectory);
      return;
    }
    AINFO << "Lattice planner stop handling: use polynomial trajectory";
  }
  **/

  GenerateLongitudinalTrajectoryBundle(
    planning_target,
    ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(
    ptr_lat_trajectory_bundle);
  return;
}

void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  AINFO << "cruise speed is  " << target_speed;
  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_->SampleLonEndConditionsForCruising(target_speed);

  for (const auto& end_condition : end_conditions) {
    // Only the last two elements in the end_condition are useful.
    std::array<double, 2> end_state;
    end_state[0] = end_condition.first[1];
    end_state[1] = end_condition.first[2];

    std::shared_ptr<Curve1d> ptr_lon_trajectory =
        std::shared_ptr<Curve1d>(new LatticeQuarticPolynomialCurve1d(
            init_lon_state_, end_state, end_condition.second));

    ptr_lon_trajectory_bundle->push_back(ptr_lon_trajectory);
  }
}

/**
void Trajectory1dGenerator::GenerateSpeedProfilesForFollowing(
    const std::array<double, 3>& init_state,
    const LonSampleConfig& lon_sample_config,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  double target_position = lon_sample_config.lon_end_condition().s();
  double target_velocity = lon_sample_config.lon_end_condition().ds();

  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForFollowing(
          init_state, target_position, target_velocity);

  for (const auto& end_condition : end_conditions) {
    std::shared_ptr<Curve1d> ptr_lon_trajectory =
        std::shared_ptr<Curve1d>(new LatticeQuinticPolynomialCurve1d(
            init_state, end_condition.first, end_condition.second));

    ptr_lon_trajectory_bundle->push_back(ptr_lon_trajectory);
  }
}
**/

void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
    const double stop_position,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {
  AINFO << "stop position s = " << stop_position;
  AINFO << "init_state s = " << init_lon_state_[0]
    << " init_state ds = " << init_lon_state_[1]
    << " init_state dds = " << init_lon_state_[2];

  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_->SampleLonEndConditionsForStopping(stop_position);
  AINFO << "end condition size = " << end_conditions.size();
  for (const auto& end_condition : end_conditions) {
    AINFO << " --- end_conditiion " << end_condition.first[0]
      << " " << end_condition.first[1] << " " << end_condition.first[2]
      << " t="<< end_condition.second;
    std::shared_ptr<Curve1d> ptr_lon_trajectory =
        std::shared_ptr<Curve1d>(new LatticeQuinticPolynomialCurve1d(
            init_lon_state_, end_condition.first, end_condition.second));

    ptr_lon_trajectory_bundle->push_back(ptr_lon_trajectory);
  }
}

void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const {

  // cruising trajectories are planned regardlessly.
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed(),
    ptr_lon_trajectory_bundle);

  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_->SampleLonEndConditionsForPathTimeBounds(
                                  planning_target);

  for (const auto& end_condition : end_conditions) {
    std::shared_ptr<Curve1d> ptr_lon_trajectory =
        std::shared_ptr<Curve1d>(new LatticeQuinticPolynomialCurve1d(
        init_lon_state_, end_condition.first, end_condition.second));
    ptr_lon_trajectory_bundle->push_back(ptr_lon_trajectory);
  }
}

void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) const {
  std::vector<std::pair<std::array<double, 3>, double>> end_conditions =
      end_condition_sampler_->SampleLatEndConditions();

  for (const auto& end_condition : end_conditions) {
    std::shared_ptr<Curve1d> ptr_lat_trajectory =
        std::shared_ptr<Curve1d>(new LatticeQuinticPolynomialCurve1d(
            init_lat_state_, end_condition.first, end_condition.second));

    ptr_lat_trajectory_bundle->push_back(ptr_lat_trajectory);
  }
}

}  // namespace planning
}  // namespace apollo
