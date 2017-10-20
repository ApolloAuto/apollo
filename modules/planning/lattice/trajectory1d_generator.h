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
 * @file trajectory1d_generator.h
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_H_

#include <memory>
#include <vector>

#include "modules/planning/proto/lattice_sampling_config.pb.h"
#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/lattice/end_condition_sampler.h"

namespace apollo {
namespace planning {

class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator() = default;

  virtual ~Trajectory1dGenerator() = default;

  void GenerateTrajectoryBundles(
      const PlanningTarget& planning_objective,
      const std::array<double, 3>& lon_init_state,
      const std::array<double, 3>& lat_init_state,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle);

 private:
  void GenerateLongitudinalTrajectoryBundle(
      const PlanningTarget& planning_objective,
      const std::array<double, 3>& lon_init_state,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForCruising(
      const std::array<double, 3>& lon_init_state,
      const LonSampleConfig& lon_sample_config,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForFollowing(
      const std::array<double, 3>& lon_init_state,
      const LonSampleConfig& lon_sample_config,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForStopping(
      const std::array<double, 3>& lon_init_state,
      const LonSampleConfig& lon_sample_config,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateLateralTrajectoryBundle(
      const std::array<double, 3>& lat_init_state,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) const;

  EndConditionSampler end_condition_sampler_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_H_
