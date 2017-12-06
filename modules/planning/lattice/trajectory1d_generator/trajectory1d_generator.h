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

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_TRAJECTORY1D_GENERATOR_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_TRAJECTORY1D_GENERATOR_H_

#include <memory>
#include <vector>

#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/lattice/trajectory1d_generator/end_condition_sampler.h"
#include "modules/planning/proto/lattice_structure.pb.h"

namespace apollo {
namespace planning {

class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator(const std::array<double, 3>& lon_init_state,
      const std::array<double, 3>& lat_init_state);

  virtual ~Trajectory1dGenerator();

  void GenerateTrajectoryBundles(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle);

 private:
  void GenerateSpeedProfilesForCruising(
      const double target_speed,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForStopping(
      const double stop_position,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateLongitudinalTrajectoryBundle(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateLateralTrajectoryBundle(
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) const;

  EndConditionSampler* end_condition_sampler_;

  std::array<double, 3> init_lon_state_;

  std::array<double, 3> init_lat_state_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_TRAJECTORY1D_GENERATOR_H_
