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
  virtual void InitProblem(const size_t num_var, const double delta_s,
                           const std::array<double, 5>& w,
                           const std::array<double, 3>& x_init = {0.0, 0.0,
                                                                  0.0},
                           const std::array<double, 3>& x_end = {0.0, 0.0,
                                                                 0.0});

  virtual void ResetInitConditions(const std::array<double, 3>& x_init) {
    x_init_ = x_init;
  }

  virtual void ResetEndConditions(const std::array<double, 3>& x_end) {
    x_end_ = x_end;
  }

  void SetZeroOrderReference(std::vector<double> x_ref);

  void SetFirstOrderPenalty(std::vector<double> penalty_dx);

  void SetZeroOrderBounds(std::vector<std::pair<double, double>> x_bounds);

  void SetFirstOrderBounds(std::vector<std::pair<double, double>> dx_bounds);

  void SetSecondOrderBounds(std::vector<std::pair<double, double>> d2x_bounds);

  void SetZeroOrderBounds(const double x_lower_bound,
                          const double x_upper_bound);

  void SetFirstOrderBounds(const double dx_lower_bound,
                           const double dx_upper_bound);

  void SetSecondOrderBounds(const double ddx_lower_bound,
                            const double ddx_upper_bound);

  void SetThirdOrderBound(const double dddx_bound) {
    max_x_third_order_derivative_ = dddx_bound;
  }

  // x_bounds: tuple(s, lower_bounds, upper_bounds)
  // s doesn't need to be sorted
  virtual void SetVariableBounds(
      const std::vector<std::tuple<double, double, double>>& x_bounds);

  // dx_bounds: tuple(s, lower_bounds, upper_bounds)
  // s doesn't need to be sorted
  virtual void SetVariableDerivativeBounds(
      const std::vector<std::tuple<double, double, double>>& dx_bounds);

  // ddx_bounds: tuple(s, lower_bounds, upper_bounds)
  // s doesn't need to be sorted
  virtual void SetVariableSecondOrderDerivativeBounds(
      const std::vector<std::tuple<double, double, double>>& ddx_bounds);

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

  virtual void ProcessBound(
      const std::vector<std::tuple<double, double, double>>& src,
      std::vector<std::pair<double, double>>* dst);

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

  struct {
    double x_w = 0.0;
    double x_derivative_w = 0.0;
    double x_second_order_derivative_w = 0.0;
    double x_third_order_derivative_w = 0.0;
    double x_ref_w = 0.0;
  } weight_;

  double max_x_third_order_derivative_ = 0.0;

  double delta_s_ = 1.0;
  double delta_s_sq_ = 1.0;
};

}  // namespace planning
}  // namespace apollo
