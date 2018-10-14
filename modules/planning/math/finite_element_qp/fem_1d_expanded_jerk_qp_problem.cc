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

#include "modules/planning/math/finite_element_qp/fem_1d_expanded_jerk_qp_problem.h"

#include <chrono>

#include "Eigen/Core"
#include "cybertron/common/log.h"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;
using apollo::common::math::DenseToCSCMatrix;

bool Fem1dExpandedJerkQpProblem::Optimize() {
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalcualteKernel(&P_data, &P_indices, &P_indptr);

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalcualteAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  std::vector<c_float> q;
  CalcualteOffset(&q);

  // extract primal results
  x_.resize(num_var_);
  x_derivative_.resize(num_var_);
  x_third_order_derivative_.resize(num_var_);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPWorkspace* work = nullptr;

  const size_t kNumParam = 3 * num_var_;
  OptimizeWithOsqp(kNumParam, lower_bounds.size(), P_data, P_indices, P_indptr,
                   A_data, A_indices, A_indptr, lower_bounds, upper_bounds, q,
                   data, &work, settings);

  // extract primal results
  x_.resize(num_var_);
  x_derivative_.resize(num_var_);
  x_third_order_derivative_.resize(num_var_);
  for (size_t i = 0; i < num_var_; ++i) {
    x_.at(i) = work->solution->x[i];
    x_derivative_.at(i) = work->solution->x[i + num_var_];
    x_third_order_derivative_.at(i) = work->solution->x[i + 2 * num_var_];
  }
  x_derivative_.back() = 0.0;
  x_third_order_derivative_.back() = 0.0;

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void Fem1dExpandedJerkQpProblem::CalcualteKernel(std::vector<c_float>* P_data,
                                                 std::vector<c_int>* P_indices,
                                                 std::vector<c_int>* P_indptr) {
  const size_t kNumParam = 3 * x_bounds_.size();

  MatrixXd kernel = MatrixXd::Zero(kNumParam, kNumParam);  // dense matrix

  for (size_t i = 0; i < kNumParam; ++i) {
    if (i < num_var_) {
      kernel(i, i) = 2.0 * weight_.x_w + 2.0 * weight_.x_mid_line_w;
    } else if (i < 2 * num_var_) {
      kernel(i, i) = 2.0 * weight_.x_derivative_w;
    } else if (i < 3 * num_var_) {
      kernel(i, i) = 2.0 * weight_.x_second_order_derivative_w;
    } else {
      kernel(i, i) = 2.0 * weight_.x_third_order_derivative_w;
    }
  }
  DenseToCSCMatrix(kernel, P_data, P_indices, P_indptr);
}

void Fem1dExpandedJerkQpProblem::CalcualteOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const size_t kNumParam = 3 * x_bounds_.size();
  q->resize(kNumParam);
  for (size_t i = 0; i < kNumParam; ++i) {
    if (i < x_bounds_.size()) {
      q->at(i) = -2.0 * weight_.x_mid_line_w *
                 (std::get<1>(x_bounds_[i]) + std::get<2>(x_bounds_[i]));
    } else {
      q->at(i) = 0.0;
    }
  }
}

void Fem1dExpandedJerkQpProblem::CalcualteAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // N constraints on x
  // 2N constraints on x', x''' (large)
  // N constraints on x'''
  // 2(N-1) constraints on x, x'
  // 2 constraints on x_init_
  const size_t kNumParam = 3 * num_var_;
  const size_t kNumConstraint = kNumParam + num_var_ + 2 * (num_var_ - 1) + 2;
  MatrixXd affine_constraint = MatrixXd::Zero(kNumConstraint, kNumParam);
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  const int prime_offset = static_cast<int>(num_var_);
  const int ppprime_offset = static_cast<int>(2 * num_var_);

  int constraint_index = 0;

  const double LARGE_VALUE = 2.0;
  for (size_t i = 0; i < kNumParam; ++i) {
    affine_constraint(constraint_index, i) = 1.0;
    if (i < num_var_) {
      // x_bounds_[i].first <= x[i] <= x_bounds_[i].second
      lower_bounds->at(constraint_index) = std::get<1>(x_bounds_[i]);
      upper_bounds->at(constraint_index) = std::get<2>(x_bounds_[i]);
    } else {
      lower_bounds->at(constraint_index) = -LARGE_VALUE;
      upper_bounds->at(constraint_index) = LARGE_VALUE;
    }
    ++constraint_index;
  }

  // x(i) - x(i-1) - x(i-1)'delta_s - 0.5 * x(i-1)''delta_s^2 - 1/6.0 *
  // x(i-1)'''delta_s^3
  for (size_t i = 0; i < num_var_; ++i) {
    if (i == 0) {
      affine_constraint(constraint_index, i) = 1.0;
      lower_bounds->at(constraint_index) = x_init_[0];
      upper_bounds->at(constraint_index) = x_init_[0];
    } else {
      affine_constraint(constraint_index, i) = 1.0;
      affine_constraint(constraint_index, i - 1) = -1.0;
      affine_constraint(constraint_index, i + prime_offset - 1) = -delta_s_;
      for (size_t k = 0; k < i; ++k) {
        if (k + 1 == i) {
          affine_constraint(constraint_index, k + ppprime_offset) =
              -delta_s_tri_ / 6.0;
        } else {
          affine_constraint(constraint_index, k + ppprime_offset) =
              -0.5 * delta_s_tri_;
        }
      }
    }
    lower_bounds->at(constraint_index) = 0.5 * delta_s_sq_ * x_init_[2];
    upper_bounds->at(constraint_index) = 0.5 * delta_s_sq_ * x_init_[2];
    ++constraint_index;
  }

  // x(i)' - x(i-1)' - x(i-1)''delta_s - 0.5 * x(i-1)'''delta_s^2 = 0
  for (size_t i = 0; i < num_var_; ++i) {
    if (i == 0) {
      affine_constraint(constraint_index, i + prime_offset) = 1.0;
      lower_bounds->at(constraint_index) = x_init_[1];
      upper_bounds->at(constraint_index) = x_init_[1];
    } else {
      affine_constraint(constraint_index, i + prime_offset) = 1.0;
      affine_constraint(constraint_index, i + prime_offset - 1) = -1.0;
      for (size_t k = 0; k < i; ++k) {
        if (k + 1 == i) {
          affine_constraint(constraint_index, k + ppprime_offset) =
              -0.5 * delta_s_sq_;
        } else {
          affine_constraint(constraint_index, k + ppprime_offset) =
              -delta_s_sq_;
        }
      }
      lower_bounds->at(constraint_index) = delta_s_ * x_init_[2];
      upper_bounds->at(constraint_index) = delta_s_ * x_init_[2];
    }
    ++constraint_index;
  }

  // x''' bounds
  for (size_t i = 0; i < num_var_; ++i) {
    if (i + 1 < num_var_) {
      affine_constraint(constraint_index, i + ppprime_offset) = 1.0;
      lower_bounds->at(constraint_index) = -weight_.x_third_order_derivative_w;
      upper_bounds->at(constraint_index) = weight_.x_third_order_derivative_w;
    } else {
      affine_constraint(constraint_index, i + ppprime_offset) = 1.0;
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  DenseToCSCMatrix(affine_constraint, A_data, A_indices, A_indptr);
}

}  // namespace planning
}  // namespace apollo
