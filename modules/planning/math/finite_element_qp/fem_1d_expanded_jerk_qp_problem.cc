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

#include "cyber/common/log.h"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;
using apollo::common::math::DenseToCSCMatrix;

bool Fem1dExpandedJerkQpProblem::Optimize() {
  auto start_time = std::chrono::system_clock::now();
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);
  auto end_time1 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time1 - start_time;
  ADEBUG << "Set Kernel used time: " << diff.count() * 1000 << " ms.";

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
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

  // extract primal results
  x_.resize(num_var_);
  x_derivative_.resize(num_var_);
  x_second_order_derivative_.resize(num_var_);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPWorkspace* work = nullptr;

  const size_t kNumParam = 4 * num_var_;
  OptimizeWithOsqp(kNumParam, lower_bounds.size(), P_data, P_indices, P_indptr,
                   A_data, A_indices, A_indptr, lower_bounds, upper_bounds, q,
                   data, &work, settings);

  if (work == nullptr || work->solution == nullptr) {
    AERROR << "Failed to find QP solution.";
    return false;
  }

  // extract primal results
  x_.resize(num_var_);
  x_derivative_.resize(num_var_);
  x_second_order_derivative_.resize(num_var_);
  x_third_order_derivative_.resize(num_var_);
  for (size_t i = 0; i < num_var_; ++i) {
    x_.at(i) = work->solution->x[i];
    x_derivative_.at(i) = work->solution->x[i + num_var_];
    x_second_order_derivative_.at(i) = work->solution->x[i + 2 * num_var_];
    x_third_order_derivative_.at(i) = work->solution->x[i + 3 * num_var_];
  }
  x_derivative_.back() = 0.0;
  x_second_order_derivative_.back() = 0.0;
  x_third_order_derivative_.back() = 0.0;

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

void Fem1dExpandedJerkQpProblem::CalculateKernel(std::vector<c_float>* P_data,
                                                 std::vector<c_int>* P_indices,
                                                 std::vector<c_int>* P_indptr) {
  const int kNumParam = static_cast<int>(4 * num_var_);
  const int N = static_cast<int>(num_var_);
  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    if (i < N) {
      P_data->push_back(2.0 * weight_.x_w + 2.0 * weight_.x_mid_line_w);
    } else if (i < 2 * N) {
      P_data->push_back(2.0 * weight_.x_derivative_w);
    } else if (i < 3 * N) {
      P_data->push_back(2.0 * weight_.x_second_order_derivative_w);
    } else {  // i < 4 * N
      P_data->push_back(2.0 * weight_.x_third_order_derivative_w);
    }
    P_indices->push_back(i);
    P_indptr->push_back(ind_p);
    ++ind_p;
  }
  P_indptr->push_back(ind_p);
  DCHECK_EQ(P_data->size(), P_indices->size());
}

void Fem1dExpandedJerkQpProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const size_t kNumParam = 4 * x_bounds_.size();
  q->resize(kNumParam);
  for (size_t i = 0; i < kNumParam; ++i) {
    if (i < x_bounds_.size()) {
      q->at(i) = -2.0 * weight_.x_mid_line_w *
                 (std::get<0>(x_bounds_[i]) + std::get<1>(x_bounds_[i]));
    } else {
      q->at(i) = 0.0;
    }
  }
}

void Fem1dExpandedJerkQpProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 4N constraints on x, x', x'', x'''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const int kNumParam = static_cast<int>(4 * num_var_);
  const int N = static_cast<int>(num_var_);
  const int kNumConstraint =
      kNumParam + static_cast<int>(3 * (num_var_ - 1) + 3);
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  int constraint_index = 0;

  // set x, dx, ddx bounds
  for (int i = 0; i < kNumParam; ++i) {
    if (i < N) {
      lower_bounds->at(constraint_index) = std::get<0>(x_bounds_[i]);
      upper_bounds->at(constraint_index) = std::get<1>(x_bounds_[i]);
    } else if (i < 2 * N) {
      lower_bounds->at(constraint_index) = std::get<0>(dx_bounds_[i - N]);
      upper_bounds->at(constraint_index) = std::get<1>(dx_bounds_[i - N]);
    } else if (i + 1 < 3 * N) {
      lower_bounds->at(constraint_index) = std::get<0>(ddx_bounds_[i - 2 * N]);
      upper_bounds->at(constraint_index) = std::get<1>(ddx_bounds_[i - 2 * N]);
    } else if (i < 3 * N) {
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    } else if (i + 1 < 4 * N) {
      lower_bounds->at(constraint_index) = -max_x_third_order_derivative_;
      upper_bounds->at(constraint_index) = max_x_third_order_derivative_;
    } else {
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
    ++constraint_index;
  }

  // x(i) - x(i-1) - x(i-1)'delta_s - 0.5 * x(i-1)''delta_s^2 - 1/6.0 *
  // x(i-1)'''delta_s^3
  for (int i = 0; i < N; ++i) {
    if (i == 0) {
      lower_bounds->at(constraint_index) = x_init_[0];
      upper_bounds->at(constraint_index) = x_init_[0];
    } else {
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
    ++constraint_index;
  }

  // x(i)' - x(i-1)' - x(i-1)''delta_s - 0.5 * x(i-1)'''delta_s^2 = 0
  for (int i = 0; i < N; ++i) {
    if (i == 0) {
      lower_bounds->at(constraint_index) = x_init_[1];
      upper_bounds->at(constraint_index) = x_init_[1];
    } else {
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
    ++constraint_index;
  }

  // x(i)'' - x(i-1)'' - x(i-1)'''delta_s = 0
  for (int i = 0; i < N; ++i) {
    if (i == 0) {
      lower_bounds->at(constraint_index) = x_init_[2];
      upper_bounds->at(constraint_index) = x_init_[2];
    } else {
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    A_data->push_back(1.0);
    A_indices->push_back(i);
    A_indptr->push_back(ind_p);
    if (i < N) {
      A_data->push_back(1.0);
      A_indices->push_back(i + 4 * N);
      if (i + 1 < N) {
        A_data->push_back(-1.0);
        A_indices->push_back(i + 4 * N + 1);
        ind_p += 3;
      } else {
        ind_p += 2;
      }
    } else if (i < 2 * N) {
      const int j = i - N;  // extra work for clarity to the reader
      if (j + 1 < N) {
        A_data->push_back(-delta_s_);
        A_indices->push_back(j + 4 * N + 1);
      }
      A_data->push_back(1.0);
      A_indices->push_back(j + 5 * N);
      if (j + 1 < N) {
        A_data->push_back(-1.0);
        A_indices->push_back(j + 5 * N + 1);
      }
      if (j + 1 < N) {
        ind_p += 4;
      } else {
        ind_p += 2;
      }
    } else if (i < 3 * N) {
      const int j = i - 2 * N;
      if (j + 1 < N) {
        A_data->push_back(-0.5 * delta_s_sq_);
        A_indices->push_back(j + 4 * N + 1);
      }
      if (j + 1 < N) {
        A_data->push_back(-delta_s_);
        A_indices->push_back(j + 5 * N + 1);
      }
      A_data->push_back(1.0);
      A_indices->push_back(j + 6 * N);
      if (j + 1 < N) {
        A_data->push_back(-1.0);
        A_indices->push_back(j + 6 * N + 1);
      }
      if (j + 1 < N) {
        ind_p += 5;
      } else {
        ind_p += 2;
      }
    } else {  // i < 4 * N
      const int j = i - 3 * N;
      if (j + 1 < N) {
        A_data->push_back(-delta_s_tri_ / 6.0);
        A_indices->push_back(j + 4 * N + 1);
      }
      if (j + 1 < N) {
        A_data->push_back(-0.5 * delta_s_sq_);
        A_indices->push_back(j + 5 * N + 1);
      }
      if (j + 1 < N) {
        A_data->push_back(-delta_s_);
        A_indices->push_back(j + 6 * N + 1);
      }
      // A_data->push_back(1.0);
      // A_indices->push_back(j + 7 * N);
      if (j + 1 < N) {
        ind_p += 4;
      } else {
        ind_p += 1;
      }
    }
  }
  A_indptr->push_back(ind_p);
  DCHECK_EQ(A_data->size(), A_indices->size());
}

void Fem1dExpandedJerkQpProblem::CalculateAffineConstraintUsingDenseMatrix(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // N constraints on x
  // 3N constraints on x', x'', x''' (large)
  // N constraints on x'''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const size_t kNumParam = 4 * num_var_;
  const size_t kNumConstraint = kNumParam + 3 * (num_var_ - 1) + 3;
  MatrixXd affine_constraint = MatrixXd::Zero(kNumConstraint, kNumParam);
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);
  const int prime_offset = static_cast<int>(num_var_);
  const int pprime_offset = static_cast<int>(2 * num_var_);
  const int ppprime_offset = static_cast<int>(3 * num_var_);
  int constraint_index = 0;
  const double LARGE_VALUE = 2.0;
  for (size_t i = 0; i < kNumParam; ++i) {
    affine_constraint(constraint_index, i) = 1.0;
    if (i < num_var_) {
      // x_bounds_[i].first <= x[i] <= x_bounds_[i].second
      lower_bounds->at(constraint_index) = std::get<0>(x_bounds_[i]);
      upper_bounds->at(constraint_index) = std::get<1>(x_bounds_[i]);
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
      affine_constraint(constraint_index, i + pprime_offset - 1) =
          -0.5 * delta_s_sq_;
      affine_constraint(constraint_index, i + ppprime_offset - 1) =
          -delta_s_tri_ / 6.0;
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
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
      affine_constraint(constraint_index, i + pprime_offset - 1) = -delta_s_;
      affine_constraint(constraint_index, i + ppprime_offset - 1) =
          -0.5 * delta_s_sq_;
      lower_bounds->at(constraint_index) = 0.0;
      upper_bounds->at(constraint_index) = 0.0;
    }
    ++constraint_index;
  }
  // x(i)'' - x(i-1)'' - x(i-1)'''delta_s = 0
  for (size_t i = 0; i < num_var_; ++i) {
    if (i == 0) {
      affine_constraint(constraint_index, i + pprime_offset) = 1.0;
      lower_bounds->at(constraint_index) = x_init_[2];
      upper_bounds->at(constraint_index) = x_init_[2];
    } else {
      affine_constraint(constraint_index, i + pprime_offset) = 1.0;
      affine_constraint(constraint_index, i + pprime_offset - 1) = -1.0;
      affine_constraint(constraint_index, i + ppprime_offset - 1) = -delta_s_;
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
