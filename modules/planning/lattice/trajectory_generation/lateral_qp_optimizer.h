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
 * @file: active_set_qp_solver.h
 * @brief: wrapper class for active set method in qpOases
 **/

#ifndef MODULES_PLANNING_LATTICE_LATERAL_QP_OPTIMIZER_H_
#define MODULES_PLANNING_LATTICE_LATERAL_QP_OPTIMIZER_H_

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "qpOASES.hpp"

#include "modules/planning/lattice/behavior/path_time_graph.h"
#include "modules/planning/lattice/trajectory1d/piecewise_jerk_trajectory1d.h"

namespace apollo {
namespace planning {

class LateralQPOptimizer {
 public:
  LateralQPOptimizer() = default;

  virtual ~LateralQPOptimizer() = default;

  bool optimize(
      const std::array<double, 3>& d_state,
      const double delta_s,
      const std::vector<std::pair<double, double>>& d_bounds);

  PiecewiseJerkTrajectory1d GetOptimalTrajectory() const;

 private:
  void Init(const std::array<double, 3>& d_state,
            const double delta_s,
            const std::vector<std::pair<double, double>>& d_bounds);

 private:
  double delta_s_;

  std::vector<double> opt_d_;

  std::vector<double> opt_d_prime_;

  std::vector<double> opt_d_pprime_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_LATERAL_QP_OPTIMIZER_H_
