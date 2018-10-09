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

#include "modules/planning/math/finite_element_qp/osqp_lateral_linear_qp_optimizer.h"

#include <algorithm>

#include "cybertron/common/log.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;
using apollo::common::math::DenseToCSCMatrix;

bool OsqpLateralLinearQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  // clean up old results
  opt_d_.clear();
  opt_d_prime_.clear();
  opt_d_pprime_.clear();

  // kernel
  std::vector<c_float> P_data_;
  std::vector<c_int> P_indices_;
  std::vector<c_int> P_indptr_;
  CalcualteKernel(d_bounds, delta_s, &P_data_, &P_indices_, &P_indptr_);

  const int kNumVariable = d_bounds.size();

  delta_s_ = delta_s;
  const int kNumConstraint = kNumVariable + (kNumVariable - 3);

  MatrixXd affine_constraint = MatrixXd::Zero(kNumConstraint, kNumVariable);
  c_float lower_bounds[kNumConstraint];
  c_float upper_bounds[kNumConstraint];
  std::fill(lower_bounds, lower_bounds + kNumConstraint, 0.0);
  std::fill(upper_bounds, upper_bounds + kNumConstraint, 0.0);

  int constraint_index = 0;

  for (int i = 0; i < kNumVariable; ++i) {
    affine_constraint(i, i) = 1.0;
    lower_bounds[constraint_index] = d_bounds[i].first;
    upper_bounds[constraint_index] = d_bounds[i].second;
    ++constraint_index;
  }

  const double third_order_derivative_max_coff =
      FLAGS_lateral_third_order_derivative_max * delta_s * delta_s * delta_s;

  for (int i = 0; i + 3 < kNumVariable; ++i) {
    affine_constraint(constraint_index, i) = -1.0;
    affine_constraint(constraint_index, i + 1) = 3.0;
    affine_constraint(constraint_index, i + 2) = -3.0;
    affine_constraint(constraint_index, i + 3) = 1.0;
    lower_bounds[constraint_index] = -third_order_derivative_max_coff;
    upper_bounds[constraint_index] = third_order_derivative_max_coff;
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  // change affine_constraint to CSC format
  std::vector<c_float> A_data_;
  std::vector<c_int> A_indices_;
  std::vector<c_int> A_indptr_;
  DenseToCSCMatrix(affine_constraint, &A_data_, &A_indices_, &A_indptr_);

  // offset
  double q[kNumVariable];
  for (int i = 0; i < kNumVariable; ++i) {
    q[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance *
           (d_bounds[i].first + d_bounds[i].second);
  }

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
  data->m = kNumConstraint;
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

  // extract primal results
  for (int i = 0; i < kNumVariable; ++i) {
    opt_d_.push_back(work->solution->x[i]);
    if (i > 0) {
      opt_d_prime_.push_back((work->solution->x[i] - work->solution->x[i - 1]) /
                             delta_s);
    }
    if (i > 1) {
      const double t = work->solution->x[i] - 2 * work->solution->x[i - 1] +
                       work->solution->x[i - 2];
      opt_d_pprime_.push_back(t / (delta_s * delta_s));
    }
  }
  opt_d_prime_.push_back(0.0);
  opt_d_pprime_.push_back(0.0);

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void OsqpLateralLinearQPOptimizer::CalcualteKernel(
    const std::vector<std::pair<double, double>>& d_bounds,
    const double delta_s, std::vector<c_float>* P_data,
    std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr) {
  const int kNumVariable = d_bounds.size();

  // const int kNumOfMatrixElement = kNumVariable * kNumVariable;
  MatrixXd kernel = MatrixXd::Zero(kNumVariable, kNumVariable);  // dense matrix

  // pre-calculate some const
  const double delta_s_sq = delta_s * delta_s;
  const double delta_s_quad = delta_s_sq * delta_s_sq;
  const double one_over_delta_s_sq_coeff =
      1.0 / delta_s_sq * FLAGS_weight_lateral_derivative;
  const double one_over_delta_s_quad_coeff =
      1.0 / delta_s_quad * FLAGS_weight_lateral_second_order_derivative;

  for (int i = 0; i < kNumVariable; ++i) {
    kernel(i, i) += 2.0 * FLAGS_weight_lateral_offset;
    kernel(i, i) += 2.0 * FLAGS_weight_lateral_obstacle_distance;

    // first order derivative
    if (i + 1 < kNumVariable) {
      kernel(i + 1, i + 1) += 2.0 * one_over_delta_s_sq_coeff;
      kernel(i, i) += 2.0 * one_over_delta_s_sq_coeff;
      kernel(i, i + 1) += 2.0 * one_over_delta_s_sq_coeff;
      kernel(i + 1, i) += 2.0 * one_over_delta_s_sq_coeff;
    }

    // second order derivative
    if (i + 2 < kNumVariable) {
      kernel(i + 2, i + 2) += 2.0 * one_over_delta_s_quad_coeff;
      kernel(i + 1, i + 1) += 8.0 * one_over_delta_s_quad_coeff;
      kernel(i, i) += 2.0 * one_over_delta_s_quad_coeff;

      kernel(i, i + 1) += -4.0 * one_over_delta_s_quad_coeff;
      kernel(i + 1, i) += -4.0 * one_over_delta_s_quad_coeff;

      kernel(i + 1, i + 2) += -4.0 * one_over_delta_s_quad_coeff;
      kernel(i + 2, i + 1) += -4.0 * one_over_delta_s_quad_coeff;

      kernel(i, i + 2) += 2.0 * one_over_delta_s_quad_coeff;
      kernel(i + 2, i) += 2.0 * one_over_delta_s_quad_coeff;
    }
  }

  DenseToCSCMatrix(kernel, P_data, P_indices, P_indptr);
}

}  // namespace planning
}  // namespace apollo
