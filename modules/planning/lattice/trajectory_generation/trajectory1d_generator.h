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

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "modules/planning/proto/lattice_structure.pb.h"

#include "modules/planning/lattice/behavior/path_time_graph.h"
#include "modules/planning/lattice/behavior/prediction_querier.h"
#include "modules/planning/lattice/trajectory_generation/end_condition_sampler.h"
#include "modules/planning/lattice/trajectory_generation/lattice_trajectory1d.h"
#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator(
      const std::array<double, 3>& lon_init_state,
      const std::array<double, 3>& lat_init_state,
      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier);

  virtual ~Trajectory1dGenerator() = default;

  void GenerateTrajectoryBundles(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle);

  void GenerateLateralTrajectoryBundle(
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) const;

 private:
  void GenerateSpeedProfilesForCruising(
      const double target_speed,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForStopping(
      const double stop_point,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForPathTimeObstacles(
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateLongitudinalTrajectoryBundle(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  template <size_t N>
  void GenerateTrajectory1DBundle(
      const std::array<double, 3>& init_state,
      const std::vector<std::pair<std::array<double, 3>, double>>&
          end_conditions,
      std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const;

 private:
  std::array<double, 3> init_lon_state_;

  std::array<double, 3> init_lat_state_;

  EndConditionSampler end_condition_sampler_;

  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<4>(
    const std::array<double, 3>& init_state,
    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const {
  CHECK_NOTNULL(ptr_trajectory_bundle);
  CHECK(!end_conditions.empty());

  ptr_trajectory_bundle->reserve(ptr_trajectory_bundle->size() +
                                 end_conditions.size());
  for (const auto& end_condition : end_conditions) {
    auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(
        std::shared_ptr<Curve1d>(new QuarticPolynomialCurve1d(
            init_state, {end_condition.first[1], end_condition.first[2]},
            end_condition.second)));

    ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
    ptr_trajectory1d->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory1d);
  }
}

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<5>(
    const std::array<double, 3>& init_state,
    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const {
  CHECK_NOTNULL(ptr_trajectory_bundle);
  CHECK(!end_conditions.empty());

  ptr_trajectory_bundle->reserve(ptr_trajectory_bundle->size() +
                                 end_conditions.size());
  for (const auto& end_condition : end_conditions) {
    auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(
        std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
            init_state, end_condition.first, end_condition.second)));

    ptr_trajectory1d->set_target_position(end_condition.first[0]);
    ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
    ptr_trajectory1d->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory1d);
  }
}

}  // namespace planning
}  // namespace apollo
