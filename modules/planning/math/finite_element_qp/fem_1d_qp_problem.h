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
 * FEM stands for finite element method.
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

class Fem1dQpProblem {
 public:
  Fem1dQpProblem() = default;

  virtual ~Fem1dQpProblem() = default;

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
  virtual bool Init(const size_t num_var, const std::array<double, 3>& x_init,
                    const double delta_s, const std::array<double, 5>& w,
                    const double max_x_third_order_derivative);

  virtual void AddReferenceLineKernel(const std::vector<double>& ref_line,
                                      const double wweight) {}

  virtual void ResetInitConditions(const std::array<double, 3>& x_init) {
    x_init_ = x_init;
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

  virtual void PreSetKernel() {}

  virtual bool Optimize() = 0;

  virtual std::vector<double> x() const { return x_; }

  virtual std::vector<double> x_derivative() const { return x_derivative_; }

  virtual std::vector<double> x_second_order_derivative() const {
    return x_second_order_derivative_;
  }

  virtual std::vector<double> x_third_order_derivative() const {
    return x_third_order_derivative_;
  }

  // modify output resolution. If not set, the output resolution is by default
  // identical to the original resolution.
  virtual void SetOutputResolution(const double resolution);

 protected:
  // naming convention follows osqp solver.
  virtual void CalculateKernel(std::vector<c_float>* P_data,
                               std::vector<c_int>* P_indices,
                               std::vector<c_int>* P_indptr) = 0;

  virtual void CalculateOffset(std::vector<c_float>* q) = 0;

  virtual void CalculateAffineConstraint(
      std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
      std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds) = 0;

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
  bool is_init_ = false;

  size_t num_var_ = 0;

  // output
  std::vector<double> x_;
  std::vector<double> x_derivative_;
  std::vector<double> x_second_order_derivative_;
  std::vector<double> x_third_order_derivative_;

  std::array<double, 3> x_init_;
  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;

  struct {
    double x_w = 0.0;
    double x_derivative_w = 0.0;
    double x_second_order_derivative_w = 0.0;
    double x_third_order_derivative_w = 0.0;
    double x_mid_line_w = 0.0;
  } weight_;

  double max_x_third_order_derivative_ = 0.0;

  double delta_s_ = 1.0;
  double delta_s_sq_ = 1.0;
  double delta_s_tri_ = 1.0;    // delta_s^3
  double delta_s_tetra_ = 1.0;  // delta_s^4
  double delta_s_penta_ = 1.0;  // delta_s^5
  double delta_s_hex_ = 1.0;    // delta_s^6
};

}  // namespace planning
}  // namespace apollo
