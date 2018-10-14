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

#pragma once

#include <array>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "osqp/include/osqp.h"

#include "modules/planning/lattice/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/finite_element_qp/lateral_qp_optimizer.h"

namespace apollo {
namespace planning {

class OsqpLateralJerkQPOptimizer : public LateralQPOptimizer {
 public:
  OsqpLateralJerkQPOptimizer() = default;

  virtual ~OsqpLateralJerkQPOptimizer() = default;

  bool optimize(
      const std::array<double, 3>& d_state, const double delta_s,
      const std::vector<std::pair<double, double>>& d_bounds) override;

 private:
  void CalculateKernel(const std::array<double, 3>& d_state,
                       const double delta_s,
                       const std::vector<std::pair<double, double>>& d_bounds,
                       std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);

  double delta_s_sq_ = 0.0;
  double delta_s_tri_ = 0.0;
  double delta_s_hex_ = 0.0;
  double lateral_coeff_ = 0.0;
  double lateral_residual_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
