/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_LATERAL_TRAJECTORY_OPTIMIZER_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_LATERAL_TRAJECTORY_OPTIMIZER_H_

#include <vector>

#include "modules/planning/lattice/trajectory1d/piecewise_jerk_trajectory1d.h"

namespace apollo {
namespace planning {

class LateralTrajectoryOptimizer  {
 public:
  LateralTrajectoryOptimizer(const double d, const double d_prime,
      const double d_pprime,
      const double delta_s,
      const double d_ppprime_abs_max,
      std::vector<std::pair<double, double>> d_boundary);

  virtual ~LateralTrajectoryOptimizer();

 private:
  std::vector<std::pair<double, double>> d_boundary_;

  double d_init_;

  double d_prime_init_;

  double d_pprime_init_;

  double delta_s_;

  double d_ppprime_abs_max_;
};

} // namespace planning
} // namespace apollo

#endif /* MODULES_PLANNING_LATTICE_TRAJECTORY1D_LATERAL_TRAJECTORY_OPTIMIZER_H_ */
