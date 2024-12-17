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

#include "osqp/osqp.h"

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
  PiecewiseJerkProblem(const size_t num_of_knots, const double delta_s,
                       const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkProblem() = default;

  void set_x_bounds(std::vector<std::pair<double, double>> x_bounds);

  void set_x_bounds(const double x_lower_bound, const double x_upper_bound);

  void set_dx_bounds(std::vector<std::pair<double, double>> dx_bounds);

  void set_dx_bounds(const double dx_lower_bound, const double dx_upper_bound);

  void set_ddx_bounds(std::vector<std::pair<double, double>> ddx_bounds);

  void set_ddx_bounds(const double ddx_lower_bound,
                      const double ddx_upper_bound);

  void set_dddx_bound(const double dddx_bound) {
    set_dddx_bound(-dddx_bound, dddx_bound);
  }

  void set_dddx_bound(const double dddx_lower_bound,
                      const double dddx_upper_bound) {
    dddx_bound_.first = dddx_lower_bound;
    dddx_bound_.second = dddx_upper_bound;
  }

  void set_weight_x(const double weight_x) { weight_x_ = weight_x; }

  void set_weight_dx(const double weight_dx) { weight_dx_ = weight_dx; }

  void set_weight_ddx(const double weight_ddx) { weight_ddx_ = weight_ddx; }

  void set_weight_dddx(const double weight_dddx) { weight_dddx_ = weight_dddx; }

  void set_scale_factor(const std::array<double, 3>& scale_factor) {
    scale_factor_ = scale_factor;
  }

  /**
   * @brief Set the x ref object and the uniform x_ref weighting
   *
   * @param weight_x_ref: uniform weighting for x_ref
   * @param x_ref: objective value of x
   */
  void set_x_ref(const double weight_x_ref, std::vector<double> x_ref);

  /**
   * @brief Set the x ref object and piecewised x_ref weightings
   *
   * @param weight_x_ref_vec: piecewised x_ref weightings
   * @param x_ref: objective value of x
   */
  void set_x_ref(std::vector<double> weight_x_ref_vec,
                 std::vector<double> x_ref);

  /**
   * @brief Set towing x ref object and piecewised towing_x_ref weightings
   *
   * @param weight_towing_x_ref_vec: piecewised towing x_ref weightings
   * @param towing_x_ref: objective value of x
   */
  void set_towing_x_ref(const double weight_towing_x_ref,
                        std::vector<double> towing_x_ref);

  void set_towing_x_ref(std::vector<double> weight_towing_x_ref_vec,
                        std::vector<double> towing_x_ref);

  void set_end_state_ref(const std::array<double, 3>& weight_end_state,
                         const std::array<double, 3>& end_state_ref);

  virtual bool Optimize(const int max_iter = 4000);

  const std::vector<double>& opt_x() const { return x_; }

  const std::vector<double>& opt_dx() const { return dx_; }

  const std::vector<double>& opt_ddx() const { return ddx_; }

 protected:
  // naming convention follows osqp solver.
  virtual void CalculateKernel(std::vector<c_float>* P_data,
                               std::vector<c_int>* P_indices,
                               std::vector<c_int>* P_indptr) = 0;

  virtual void CalculateOffset(std::vector<c_float>* q) = 0;

  virtual void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                         std::vector<c_int>* A_indices,
                                         std::vector<c_int>* A_indptr,
                                         std::vector<c_float>* lower_bounds,
                                         std::vector<c_float>* upper_bounds);

  virtual OSQPSettings* SolverDefaultSettings();

  bool FormulateProblem(OSQPData* data);

  void FreeData(OSQPData* data);

  bool CheckLowUpperBound(const std::vector<c_float>& lower,
                          const std::vector<c_float>& upper);

  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

 protected:
  size_t num_of_knots_ = 0;

  // output
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;

  std::array<double, 3> x_init_;
  std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};

  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bound_;

  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;

  double delta_s_ = 1.0;

  bool has_x_ref_ = false;
  double weight_x_ref_ = 0.0;
  std::vector<double> x_ref_;
  // un-uniformed weighting
  std::vector<double> weight_x_ref_vec_;

  bool has_towing_x_ref_ = false;
  std::vector<double> towing_x_ref_;
  std::vector<double> weight_towing_x_ref_vec_;

  bool has_end_state_ref_ = false;
  std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> end_state_ref_;
};

}  // namespace planning
}  // namespace apollo
