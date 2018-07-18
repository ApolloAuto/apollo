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

#include "modules/planning/lattice/trajectory_generation/lateral_trajectory_optimizer.h"

namespace apollo {
namespace planning {

LateralTrajectoryOptimizer::LateralTrajectoryOptimizer(const double d,
    const double d_prime, const double d_pprime, const double delta_s,
    const double d_ppprime_abs_max,
    std::vector<std::pair<double, double>> d_boundary) :
    d_init_(d), d_prime_init_(d_prime), d_pprime_init_(d_pprime), delta_s_(
        delta_s), d_ppprime_abs_max_(d_ppprime_abs_max), d_boundary_(
        std::move(d_boundary)) {}

LateralTrajectoryOptimizer::~LateralTrajectoryOptimizer() {
  // TODO Auto-generated destructor stub
}

} // namespace planning
} // namespace apollo
