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

#include "modules/planning/math/finite_element_qp/osqp_lateral_jerk_qp_optimizer.h"

#include <algorithm>

#include "cybertron/common/log.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;
using apollo::common::math::DenseToCSCMatrix;

bool OsqpLateralJerkQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  // clean up old results
  opt_d_.resize(d_bounds.size());

  CalcualteKernel(d_state, delta_s, d_bounds, &P_data_, &P_indices_,
                  &P_indptr_);

  const int kNumVariable = d_bounds.size();

  delta_s_ = delta_s;
  const int kNumConstraint = kNumVariable + kNumVariable;

  MatrixXd affine_constraint = MatrixXd::Zero(kNumConstraint, kNumVariable);
  c_float lower_bounds[kNumConstraint];
  c_float upper_bounds[kNumConstraint];
  std::fill(lower_bounds, lower_bounds + kNumConstraint, 0.0);
  std::fill(upper_bounds, upper_bounds + kNumConstraint, 0.0);

  int constraint_index = 0;

  for (int i = 0; i < kNumVariable; ++i) {
    affine_constraint(constraint_index, i) = 1.0;
    lower_bounds[constraint_index] = -FLAGS_lateral_third_order_derivative_max;
    upper_bounds[constraint_index] = FLAGS_lateral_third_order_derivative_max;
    ++constraint_index;
  }

  for (int i = 0; i < kNumVariable; ++i) {
    for (int j = 0; j < kNumVariable; ++j) {
      affine_constraint(i, j) = 1 / 6.0 * delta_s_tri_;
    }
    lower_bounds[constraint_index] = d_bounds[i].first - lateral_residual_;
    upper_bounds[constraint_index] = d_bounds[i].second - lateral_residual_;
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  // change affine_constraint to CSC format
  DenseToCSCMatrix(affine_constraint, &A_data_, &A_indices_, &A_indptr_);

  // offset
  const double offset_coeff = delta_s_tri_ * lateral_residual_ / 3.0;
  double q[kNumVariable];
  std::fill(q, q + kNumVariable, offset_coeff);

  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;  // Change alpha parameter
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polish = true;
  settings->verbose = true;

  // Populate data
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = kNumVariable;
  data->m = affine_constraint.rows();
  data->P = csc_matrix(data->n, data->n, P_data_.size(), P_data_.data(),
                       P_indices_.data(), P_indptr_.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data_.size(), A_data_.data(),
                       A_indices_.data(), A_indptr_.data());
  data->l = lower_bounds;
  data->u = upper_bounds;

  // Workspace
  OSQPWorkspace* work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  // extract primal results (don't care about opt_d_prime_ and opt_d_pprime_)
  for (int i = 0; i < kNumVariable; ++i) {
    double l = lateral_residual_;
    for (int j = 0; j <= i; ++j) {
      l += 1 / 6.0 * delta_s_tri_ * work->solution->x[j];
    }
    opt_d_[i] = l;
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void OsqpLateralJerkQPOptimizer::CalcualteKernel(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  const int kNumVariable = d_bounds.size();

  MatrixXd kernel = MatrixXd::Zero(kNumVariable, kNumVariable);

  // pre-calculate some const
  delta_s_sq_ = delta_s * delta_s;
  delta_s_tri_ = delta_s * delta_s * delta_s;
  delta_s_hex_ = delta_s_sq_ * delta_s_sq_ * delta_s_sq_;

  lateral_coeff_ = delta_s_hex_ / 18.0 * FLAGS_weight_lateral_offset;
  lateral_residual_ =
      0.5 * d_state[2] * delta_s_sq_ + d_state[1] * delta_s + d_state[0];

  for (int i = 0; i < kNumVariable; ++i) {
    // third order derivative
    kernel(i, i) += 2.0 * FLAGS_weight_lateral_third_order_derivative;

    // lateral offset
    kernel(i, i) += lateral_coeff_;
    for (int j = i + 1; j < kNumVariable; ++j) {
      kernel(i, j) += lateral_coeff_;
      kernel(j, i) += lateral_coeff_;
    }
  }

  DenseToCSCMatrix(kernel, P_data, P_indices, P_indptr);
}

}  // namespace planning
}  // namespace apollo
