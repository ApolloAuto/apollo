/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <tuple>
#include <utility>
#include <vector>

#include "osqp/include/osqp.h"

namespace apollo {
namespace planning {

class FiniteElement1dOptimizer {
 public:
  /*
   * @param
   * x_init: the init status of x, x', x''
   * delta_s: s(k) - s(k-1)
   */
  FiniteElement1dOptimizer(const size_t num_var,
      const std::array<double, 3>& x_init, const double delta_s);

  virtual ~FiniteElement1dOptimizer() = default;

  void SetZeroOrderBounds(std::vector<std::pair<double, double>> x_bounds);

  void SetFirstOrderBounds(std::vector<std::pair<double, double>> dx_bounds);

  void SetSecondOrderBounds(std::vector<std::pair<double, double>> d2x_bounds);

  void SetZeroOrderBounds(const double x_bound);

  void SetFirstOrderBounds(const double dx_bound);

  void SetSecondOrderBounds(const double ddx_bound);

  void SetThirdOrderBound(const double dddx_bound) {
    dddx_bound_ = dddx_bound;
  }

  void SetWeightX(const double weight_x) {
    weight_x_ = weight_x;
  }

  void SetWeightXDerivative(const double weight_dx) {
    weight_dx_ = weight_dx;
  }

  void SetWeightXSecondOrderDerivative(const double weight_ddx) {
    weight_ddx_ = weight_ddx;
  }

  bool Solve(std::vector<double>* ptr_x, std::vector<double>* ptr_dx_,
      std::vector<double>* ptr_ddx_);

 protected:
  // naming convention follows osqp solver.
  virtual void CalculateKernel(std::vector<c_float>* P_data,
      std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr);

  virtual void CalculateOffset(std::vector<c_float>* q);

  virtual void CalculateAffineConstraint(std::vector<c_float>* A_data,
      std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds);

  bool Solve(const size_t kernel_dim, const size_t num_affine_constraint,
      std::vector<c_float>& P_data, std::vector<c_int>& P_indices,    // NOLINT
      std::vector<c_int>& P_indptr, std::vector<c_float>& A_data,     // NOLINT
      std::vector<c_int>& A_indices, std::vector<c_int>& A_indptr,    // NOLINT
      std::vector<c_float>& lower_bounds,                             // NOLINT
      std::vector<c_float>& upper_bounds,                             // NOLINT
      std::vector<c_float>& q, OSQPData* data, OSQPWorkspace** work,  // NOLINT
      OSQPSettings* settings);

  size_t num_of_knots_ = 0;

  double weight_x_ = 1.0;

  double weight_dx_ = 1.0;

  double weight_ddx_ = 1.0;

  std::array<double, 3> x_init_;
  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  double dddx_bound_ = 1.0;

  double delta_s_ = 1.0;
};

}  // namespace planning
}  // namespace apollo
