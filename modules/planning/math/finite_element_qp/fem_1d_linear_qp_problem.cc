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

#include "modules/planning/math/finite_element_qp/fem_1d_linear_qp_problem.h"

#include "Eigen/Core"
#include "cybertron/common/log.h"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;
using apollo::common::math::DenseToCSCMatrix;

bool Fem1dLinearQpProblem::Optimize() {
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

  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPWorkspace* work = nullptr;

  OptimizeWithOsqp(num_var_, lower_bounds.size(), P_data, P_indices, P_indptr,
                   A_data, A_indices, A_indptr, lower_bounds, upper_bounds, q,
                   data, &work, settings);

  x_.resize(num_var_ + 1);
  x_derivative_.resize(num_var_ + 1);
  x_second_order_derivative_.resize(num_var_ + 1);

  x_.front() = x_init_[0];
  x_derivative_.front() = x_init_[1];
  x_second_order_derivative_.front() = x_init_[2];

  for (size_t i = 0; i < num_var_; ++i) {
    x_.at(i + 1) = work->solution->x[i];
    // TODO(All): extract x_derivative_ and x_second_order_derivative_
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);
  return true;
}

void Fem1dLinearQpProblem::CalcualteKernel(std::vector<c_float>* P_data,
                                           std::vector<c_int>* P_indices,
                                           std::vector<c_int>* P_indptr) {
  MatrixXd kernel_x = MatrixXd::Zero(num_var_, num_var_);  // dense matrix
  MatrixXd kernel_x_mid_line =
      MatrixXd::Zero(num_var_, num_var_);                      // dense matrix
  MatrixXd kernel_x_p = MatrixXd::Zero(num_var_, num_var_);    // dense matrix
  MatrixXd kernel_x_pp = MatrixXd::Zero(num_var_, num_var_);   // dense matrix
  MatrixXd kernel_x_ppp = MatrixXd::Zero(num_var_, num_var_);  // dense matrix

  for (size_t i = 0; i < num_var_; ++i) {
    kernel_x(i, i) += 1.0;
    kernel_x_mid_line(i, i) += 1.0;

    // first order derivative
    if (i == 0) {
      kernel_x_p(i, i) += 1.0;
    } else {
      kernel_x_p(i, i) += 1.0;
      kernel_x_p(i - 1, i - 1) += 1.0;
      kernel_x_p(i, i - 1) += -1.0;
      kernel_x_p(i - 1, i) += -1.0;
    }

    // second order derivative
    if (i == 0) {
      kernel_x_pp(i, i) += 1.0;
    } else if (i == 1) {
      kernel_x_pp(i, i) += 1.0;
      kernel_x_pp(i - 1, i - 1) += 4.0;
      kernel_x_pp(i, i - 1) += -2.0;
      kernel_x_pp(i - 1, i) += -2.0;
    } else {
      kernel_x_pp(i, i) += 1.0;
      kernel_x_pp(i - 1, i - 1) += 4.0;
      kernel_x_pp(i - 2, i - 2) += 1.0;

      kernel_x_pp(i, i - 1) += -2.0;
      kernel_x_pp(i - 1, i) += -2.0;
      kernel_x_pp(i - 1, i - 2) += -2.0;
      kernel_x_pp(i - 2, i - 1) += -2.0;
      kernel_x_pp(i, i - 2) += 1.0;
      kernel_x_pp(i - 2, i) += 1.0;
    }

    // third order derivative
    if (i == 0) {
      kernel_x_ppp(0, 0) += 1.0;
    } else if (i == 1) {
      kernel_x_ppp(0, 0) += 9.0;
      kernel_x_ppp(1, 1) += 1.0;

      kernel_x_ppp(0, 1) += -3.0;
      kernel_x_ppp(1, 0) += -3.0;
    } else if (i == 2) {
      kernel_x_ppp(0, 0) += 9.0;
      kernel_x_ppp(1, 1) += 9.0;
      kernel_x_ppp(2, 2) += 1.0;

      kernel_x_ppp(1, 2) += -3.0;
      kernel_x_ppp(2, 1) += -3.0;
      kernel_x_ppp(0, 2) += 3.0;
      kernel_x_ppp(2, 0) += 3.0;

      kernel_x_ppp(0, 1) += -9.0;
      kernel_x_ppp(1, 0) += -9.0;
    } else {
      kernel_x_ppp(i - 3, i - 3) += 1.0;
      kernel_x_ppp(i - 2, i - 2) += 9.0;
      kernel_x_ppp(i - 1, i - 1) += 9.0;
      kernel_x_ppp(i, i) += 1.0;

      kernel_x_ppp(i - 1, i) += -3.0;
      kernel_x_ppp(i, i - 1) += -3.0;
      kernel_x_ppp(i - 2, i) += 3.0;
      kernel_x_ppp(i, i - 2) += 3.0;
      kernel_x_ppp(i - 3, i) += -1.0;
      kernel_x_ppp(i, i - 3) += -1.0;

      kernel_x_ppp(i - 2, i - 1) += -9.0;
      kernel_x_ppp(i - 1, i - 2) += -9.0;
      kernel_x_ppp(i - 3, i - 1) += 3.0;
      kernel_x_ppp(i - 1, i - 3) += 3.0;

      kernel_x_ppp(i - 3, i - 2) += -3.0;
      kernel_x_ppp(i - 2, i - 3) += -3.0;
    }
  }
  // pre-calculate some const
  const double one_over_delta_s_sq_coeff =
      1.0 / delta_s_sq_ * weight_.x_derivative_w;
  const double one_over_delta_s_tetra_coeff =
      1.0 / delta_s_tetra_ * weight_.x_second_order_derivative_w;
  const double one_over_delta_s_hex_coeff =
      1.0 / (delta_s_sq_ * delta_s_tetra_) * weight_.x_third_order_derivative_w;

  const MatrixXd kernel = kernel_x * 2.0 * weight_.x_w +
                          kernel_x_mid_line * 2.0 * weight_.x_mid_line_w +
                          kernel_x_p * 2.0 * one_over_delta_s_sq_coeff +
                          kernel_x_pp * 2.0 * one_over_delta_s_tetra_coeff +
                          kernel_x_ppp * 2.0 * one_over_delta_s_hex_coeff;

  DenseToCSCMatrix(kernel, P_data, P_indices, P_indptr);
}

void Fem1dLinearQpProblem::CalcualteOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  q->resize(num_var_);
  for (size_t i = 0; i < num_var_; ++i) {
    q->at(i) = -2.0 * weight_.x_mid_line_w *
               (x_bounds_[i].first + x_bounds_[i].second);
  }
  // first order derivative offset
  q->at(0) += (-2.0 * x_init_[0]) * weight_.x_derivative_w / delta_s_sq_;

  // second order derivative offset
  const double delta_s_quad_offset_coeff =
      weight_.x_second_order_derivative_w / delta_s_tri_;
  q->at(0) += (-6.0 * x_init_[0] - 2.0 * delta_s_ * x_init_[1]) *
              delta_s_quad_offset_coeff;
  q->at(1) += (2.0 * x_init_[0]) * delta_s_quad_offset_coeff;

  // third order derivative offset
  const double delta_s_hex_offset_coeff =
      weight_.x_third_order_derivative_w / delta_s_hex_;
  q->at(0) += (-20.0 * x_init_[0] - 8.0 * delta_s_ * x_init_[1] -
               2.0 * delta_s_sq_ * x_init_[2]) *
              delta_s_hex_offset_coeff;
  q->at(1) += (10.0 * x_init_[0] + 2.0 * delta_s_ * x_init_[1]) *
              delta_s_hex_offset_coeff;
  q->at(2) += (-2.0 * x_init_[0]) * delta_s_hex_offset_coeff;
}

void Fem1dLinearQpProblem::CalcualteAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  const int kNumConstraint = num_var_ + num_var_;
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  MatrixXd affine_constraint = MatrixXd::Zero(kNumConstraint, num_var_);

  int constraint_index = 0;
  for (size_t i = 0; i < num_var_; ++i) {
    affine_constraint(i, i) = 1.0;
    lower_bounds->at(constraint_index) = x_bounds_[i].first;
    upper_bounds->at(constraint_index) = x_bounds_[i].second;
    ++constraint_index;
  }

  const double third_order_derivative_max_coff =
      max_x_third_order_derivative_ * delta_s_tri_;

  for (size_t i = 0; i < num_var_; ++i) {
    if (i == 0) {
      affine_constraint(constraint_index, i) = 1.0;
      const double t =
          x_init_[0] + x_init_[1] * delta_s_ + x_init_[2] * delta_s_sq_;
      lower_bounds->at(constraint_index) = -third_order_derivative_max_coff + t;
      upper_bounds->at(constraint_index) = third_order_derivative_max_coff + t;
    } else if (i == 1) {
      affine_constraint(constraint_index, i) = 1.0;
      affine_constraint(constraint_index, i - 1) = -3.0;
      const double t = 2 * x_init_[0] + x_init_[1] * delta_s_;
      lower_bounds->at(constraint_index) = -third_order_derivative_max_coff - t;
      upper_bounds->at(constraint_index) = third_order_derivative_max_coff - t;
    } else if (i == 2) {
      affine_constraint(constraint_index, i) = 1.0;
      affine_constraint(constraint_index, i - 1) = -3.0;
      affine_constraint(constraint_index, i - 2) = 3.0;
      lower_bounds->at(constraint_index) =
          -third_order_derivative_max_coff + x_init_[0];
      upper_bounds->at(constraint_index) =
          third_order_derivative_max_coff + x_init_[0];
    } else {
      affine_constraint(constraint_index, i) = 1.0;
      affine_constraint(constraint_index, i - 1) = -3.0;
      affine_constraint(constraint_index, i - 2) = 3.0;
      affine_constraint(constraint_index, i - 3) = -1.0;
      lower_bounds->at(constraint_index) = -third_order_derivative_max_coff;
      upper_bounds->at(constraint_index) = third_order_derivative_max_coff;
    }
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  DenseToCSCMatrix(affine_constraint, A_data, A_indices, A_indptr);
}

}  // namespace planning
}  // namespace apollo
