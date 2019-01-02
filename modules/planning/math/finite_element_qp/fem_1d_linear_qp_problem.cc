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

#include <algorithm>
#include <chrono>

#include "cyber/common/log.h"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace planning {

bool Fem1dLinearQpProblem::Optimize() {
  auto start_time = std::chrono::system_clock::now();
  if (!is_const_kernel_) {
    PreSetKernel();
  }
  auto end_time1 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time1 - start_time;
  ADEBUG << "Set Kernel used time: " << diff.count() * 1000 << " ms.";

  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);
  auto end_time2 = std::chrono::system_clock::now();
  diff = end_time2 - end_time1;
  ADEBUG << "CalculateAffineConstraint used time: " << diff.count() * 1000
         << " ms.";

  std::vector<c_float> q;
  CalculateOffset(&q);
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "CalculateOffset used time: " << diff.count() * 1000 << " ms.";

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

  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Run OptimizeWithOsqp used time: " << diff.count() * 1000 << " ms.";

  return true;
}

void Fem1dLinearQpProblem::CalculateKernel(std::vector<c_float>* P_data,
                                           std::vector<c_int>* P_indices,
                                           std::vector<c_int>* P_indptr) {
  // pre-calculate some const
  const double one_over_delta_s_sq_coeff =
      1.0 / delta_s_sq_ * weight_.x_derivative_w;
  const double one_over_delta_s_tetra_coeff =
      1.0 / delta_s_tetra_ * weight_.x_second_order_derivative_w;
  const double one_over_delta_s_hex_coeff =
      1.0 / (delta_s_sq_ * delta_s_tetra_) * weight_.x_third_order_derivative_w;

  int N = static_cast<int>(num_var_);

  std::array<double, 7> k;
  k[0] = -1.0 * 2.0 * one_over_delta_s_hex_coeff;
  k[1] = 1.0 * 2.0 * one_over_delta_s_tetra_coeff +
         (3.0 + 3.0) * 2.0 * one_over_delta_s_hex_coeff;
  k[2] = (-1.0) * 2.0 * one_over_delta_s_sq_coeff +
         (-2.0 - 2.0) * 2.0 * one_over_delta_s_tetra_coeff +
         (-3.0 - 9.0 - 3.0) * 2.0 * one_over_delta_s_hex_coeff;
  k[3] = 1.0 * 2.0 * weight_.x_w + 1.0 * 2.0 * weight_.x_mid_line_w +
         (1.0 + 1.0) * 2.0 * one_over_delta_s_sq_coeff +
         (1.0 + 4.0 + 1.0) * 2.0 * one_over_delta_s_tetra_coeff +
         (1.0 + 9.0 + 9.0 + 1.0) * 2.0 * one_over_delta_s_hex_coeff;
  k[4] = k[2];
  k[5] = k[1];
  k[6] = k[0];

  for (int i = 0; i < N; ++i) {
    if (i + 3 < N) {
      for (int j = std::max(0, 3 - i); j < 7; ++j) {
        P_data->push_back(k[j]);
      }
    } else if (i + 3 == N) {
      P_data->push_back(k[0]);
      P_data->push_back(k[1]);
      P_data->push_back(k[2]);
      P_data->push_back(k[3] - 1.0 * 2.0 * one_over_delta_s_hex_coeff);
      P_data->push_back(k[4] - (-3.0 * 2.0 * one_over_delta_s_hex_coeff));
      P_data->push_back(k[5] - 3.0 * 2.0 * one_over_delta_s_hex_coeff);
    } else if (i + 2 == N) {
      P_data->push_back(k[0]);
      P_data->push_back(k[1]);
      P_data->push_back(k[2] - (-3.0 * 2.0 * one_over_delta_s_hex_coeff));
      P_data->push_back(k[3] - (1.0 * 2.0 * one_over_delta_s_tetra_coeff) -
                        ((9.0 + 1.0) * 2.0 * one_over_delta_s_hex_coeff));
      P_data->push_back(k[4] - (-2.0 * 2.0 * one_over_delta_s_tetra_coeff) -
                        (-9.0 - 3.0) * 2.0 * one_over_delta_s_hex_coeff);
    } else {  // i + 1 == N
      P_data->push_back(k[0]);
      P_data->push_back(k[1] - 3.0 * 2.0 * one_over_delta_s_hex_coeff);
      P_data->push_back(k[2] - (-2.0 * 2.0 * one_over_delta_s_tetra_coeff) -
                        (-9.0 - 3.0) * 2.0 * one_over_delta_s_hex_coeff);
      P_data->push_back(k[3] - (1.0 * 2.0 * one_over_delta_s_sq_coeff) -
                        (4.0 + 1.0) * 2.0 * one_over_delta_s_tetra_coeff -
                        (1.0 + 9.0 + 9.0) * 2.0 * one_over_delta_s_hex_coeff);
    }
  }

  // P_indices
  for (int i = 0; i < N; ++i) {
    for (int j = std::max(0, i - 3); j < std::min(i + 4, N); ++j) {
      P_indices->push_back(j);
    }
  }

  int ind_p = 0;
  for (int i = 0; i < N; ++i) {
    P_indptr->push_back(ind_p);
    int delta = std::min(i + 3, N - 1) - std::max(0, i - 3) + 1;
    ind_p += delta;
  }
  P_indptr->push_back(ind_p);

  CHECK_EQ(P_data->size(), P_indices->size());
}

void Fem1dLinearQpProblem::PreSetKernel() {
  CalculateKernel(&P_data, &P_indices, &P_indptr);
  is_const_kernel_ = true;
}

void Fem1dLinearQpProblem::PreSetAffineConstraintMatrix() {
  const int N = static_cast<int>(num_var_);
  for (int i = 0; i < N; ++i) {
    A_data.push_back(1.0);
    A_data.push_back(1.0);

    A_indices.push_back(i);
    A_indices.push_back(i + N);

    if (i + 1 < N) {
      A_data.push_back(-3.0);
      A_indices.push_back(i + N + 1);
    }
    if (i + 2 < N) {
      A_data.push_back(3.0);
      A_indices.push_back(i + N + 2);
    }
    if (i + 3 < N) {
      A_data.push_back(-1.0);
      A_indices.push_back(i + N + 3);
    }
  }
  CHECK_EQ(A_data.size(), A_indices.size());

  int ind_p = 0;
  for (int i = 0; i < N; ++i) {
    A_indptr.push_back(ind_p);
    if (i + 3 < N) {
      ind_p += 5;
    } else if (i + 2 < N) {
      ind_p += 4;
    } else if (i + 1 < N) {
      ind_p += 3;
    } else {  // i + 1 == N
      ind_p += 2;
    }
  }
  A_indptr.push_back(ind_p);
}

void Fem1dLinearQpProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  q->resize(num_var_);
  for (size_t i = 0; i < num_var_; ++i) {
    q->at(i) = -2.0 * weight_.x_mid_line_w *
               (std::get<0>(x_bounds_[i]) + std::get<1>(x_bounds_[i]));
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

void Fem1dLinearQpProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  const int kNumConstraint = static_cast<int>(num_var_ + num_var_);
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  int constraint_index = 0;
  for (size_t i = 0; i < num_var_; ++i) {
    // affine_constraint(i, i) = 1.0;
    lower_bounds->at(constraint_index) = std::get<0>(x_bounds_[i]);
    upper_bounds->at(constraint_index) = std::get<1>(x_bounds_[i]);
    ++constraint_index;
  }

  const double third_order_derivative_max_coff =
      max_x_third_order_derivative_ * delta_s_tri_;
  {
    // affine_constraint(constraint_index, i) = 1.0;
    const double t =
        x_init_[0] + x_init_[1] * delta_s_ + x_init_[2] * delta_s_sq_;
    lower_bounds->at(constraint_index) = -third_order_derivative_max_coff + t;
    upper_bounds->at(constraint_index) = third_order_derivative_max_coff + t;
    ++constraint_index;
  }
  {
    const double t = 2 * x_init_[0] + x_init_[1] * delta_s_;
    lower_bounds->at(constraint_index) = -third_order_derivative_max_coff - t;
    upper_bounds->at(constraint_index) = third_order_derivative_max_coff - t;
    ++constraint_index;
  }
  {
    lower_bounds->at(constraint_index) =
        -third_order_derivative_max_coff + x_init_[0];
    upper_bounds->at(constraint_index) =
        third_order_derivative_max_coff + x_init_[0];
    ++constraint_index;
  }

  std::fill(lower_bounds->begin() + constraint_index, lower_bounds->end(),
            -third_order_derivative_max_coff);
  std::fill(upper_bounds->begin() + constraint_index, upper_bounds->end(),
            third_order_derivative_max_coff);

  if (!is_const_affine_constraint_matrix_) {
    PreSetAffineConstraintMatrix();
    ADEBUG << "PreSetAffineConstraintMatrix() called.";
    is_const_affine_constraint_matrix_ = true;
  }
}

}  // namespace planning
}  // namespace apollo
