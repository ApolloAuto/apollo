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

#include <tuple>
#include <utility>
#include <vector>

#include "osqp/include/osqp.h"

namespace apollo {
namespace planning {

/*
 * @brief:
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkProblem {
 public:
  PiecewiseJerkProblem() = default;

  virtual ~PiecewiseJerkProblem() = default;

  /*
   * @param
   * x_init: the init status of x, x', x''
   * delta_s: s(k) - s(k-1)
   * x_bounds: x_bounds[i].first < x(i) < x_bounds[i].second
   * w: weight array
   * -- w[0]: x^2 term weight
   * -- w[1]: (x')^2 term weight
   * -- w[2]: (x'')^2 term weight
   * -- w[3]: (x''')^2 term weight
   * -- w[4]: default reference line weight, (x_bounds[k].first +
   * x_bounds[k].second)/2
   */
  virtual void InitProblem(const size_t num_of_knots, const double delta_s,
                           const std::array<double, 3>& x_init,
                           const std::array<double, 3>& x_end);

  virtual void SetInitCondition(const std::array<double, 3>& x_init) {
    x_init_ = x_init;
  }

  virtual void SetEndCondition(const std::array<double, 3>& x_end) {
    x_end_ = x_end;
  }

  void SetZeroOrderReference(std::vector<double> x_ref);

  void SetFirstOrderPenalty(std::vector<double> penalty_dx);


  void SetZeroOrderBounds(std::vector<std::pair<double, double>> x_bounds);

  void SetZeroOrderBounds(const double x_lower_bound,
                          const double x_upper_bound);

  void SetFirstOrderBounds(std::vector<std::pair<double, double>> dx_bounds);

  void SetFirstOrderBounds(const double dx_lower_bound,
                           const double dx_upper_bound);


  void SetSecondOrderBounds(std::vector<std::pair<double, double>> d2x_bounds);

  void SetSecondOrderBounds(const double ddx_lower_bound,
                            const double ddx_upper_bound);

  void SetThirdOrderBound(const double dddx_bound) {
    dddx_bound_ = dddx_bound;
  }

  void set_weight_x(const double weight_x) {
    weight_x_ = weight_x;
  }

  void set_weight_dx(const double weight_dx) {
    weight_dx_ = weight_dx;
  }

  void set_weight_ddx(const double weight_ddx) {
    weight_ddx_ = weight_ddx;
  }

  void set_weight_dddx(const double weight_dddx) {
    weight_dddx_ = weight_dddx;
  }

  void set_weight_x_reference(const double weight_x_reference) {
    weight_x_reference_ = weight_x_reference;
  }

  virtual bool Optimize(const int max_iter = 4000);

  const std::vector<double>& x() const { return x_; }

  const std::vector<double>& x_derivative() const { return dx_; }

  const std::vector<double>& x_second_order_derivative() const { return ddx_; }

 protected:
  // naming convention follows osqp solver.
  virtual void CalculateKernel(std::vector<c_float>* P_data,
                               std::vector<c_int>* P_indices,
                               std::vector<c_int>* P_indptr);

  virtual void CalculateOffset(std::vector<c_float>* q);

  virtual void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                         std::vector<c_int>* A_indices,
                                         std::vector<c_int>* A_indptr,
                                         std::vector<c_float>* lower_bounds,
                                         std::vector<c_float>* upper_bounds);

  bool OptimizeWithOsqp(
      const size_t kernel_dim, const size_t num_affine_constraint,
      std::vector<c_float>& P_data, std::vector<c_int>& P_indices,    // NOLINT
      std::vector<c_int>& P_indptr, std::vector<c_float>& A_data,     // NOLINT
      std::vector<c_int>& A_indices, std::vector<c_int>& A_indptr,    // NOLINT
      std::vector<c_float>& lower_bounds,                             // NOLINT
      std::vector<c_float>& upper_bounds,                             // NOLINT
      std::vector<c_float>& q, OSQPData* data, OSQPWorkspace** work,  // NOLINT
      OSQPSettings* settings);

 protected:
  size_t num_of_knots_ = 0;

  // output
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;

  std::array<double, 3> x_init_;
  std::array<double, 3> x_end_;
  std::vector<double> x_ref_;
  std::vector<double> penalty_dx_;

  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  double dddx_bound_ = 0.0;

  double weight_x_ = 0.0;

  double weight_dx_ = 0.0;

  double weight_ddx_ = 0.0;

  double weight_dddx_ = 0.0;

  double weight_x_reference_ = 0.0;

  double delta_s_ = 1.0;
};

}  // namespace planning
}  // namespace apollo
