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

#include "modules/planning/math/finite_element_qp/fem_1d_expanded_qp_problem.h"

#include <utility>

#include "cyber/common/log.h"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace planning {

bool Fem1dExpandedQpProblem::Optimize() {
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  std::vector<c_float> q;
  CalculateOffset(&q);

  // extract primal results
  x_.resize(num_var_);
  x_derivative_.resize(num_var_);
  x_second_order_derivative_.resize(num_var_);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPWorkspace* work = nullptr;

  const size_t kNumVariable = 3 * num_var_;
  OptimizeWithOsqp(kNumVariable, lower_bounds.size(), P_data, P_indices,
                   P_indptr, A_data, A_indices, A_indptr, lower_bounds,
                   upper_bounds, q, data, &work, settings);

  // extract primal results
  x_.resize(num_var_);
  x_derivative_.resize(num_var_);
  x_second_order_derivative_.resize(num_var_);
  for (size_t i = 0; i < x_bounds_.size(); ++i) {
    x_.at(i) = work->solution->x[i];
    x_derivative_.at(i) = work->solution->x[i + num_var_];
    x_second_order_derivative_.at(i) = work->solution->x[i + 2 * num_var_];
  }
  x_derivative_.back() = 0.0;
  x_second_order_derivative_.back() = 0.0;

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void Fem1dExpandedQpProblem::CalculateKernel(std::vector<c_float>* P_data,
                                             std::vector<c_int>* P_indices,
                                             std::vector<c_int>* P_indptr) {
  const size_t kNumParam = 3 * x_bounds_.size();
  P_data->resize(kNumParam);
  P_indices->resize(kNumParam);
  P_indptr->resize(kNumParam + 1);

  for (size_t i = 0; i < kNumParam; ++i) {
    if (i < x_bounds_.size()) {
      P_data->at(i) = 2.0 * weight_.x_w + 2.0 * weight_.x_mid_line_w;
    } else if (i < 2 * x_bounds_.size()) {
      P_data->at(i) = 2.0 * 2.0 * weight_.x_derivative_w;
    } else {
      P_data->at(i) = 2.0 * weight_.x_second_order_derivative_w;
    }
    P_indices->at(i) = i;
    P_indptr->at(i) = i;
  }
  P_indptr->at(kNumParam) = kNumParam;
  CHECK_EQ(P_data->size(), P_indices->size());
}

void Fem1dExpandedQpProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const size_t kNumParam = 3 * x_bounds_.size();
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

void Fem1dExpandedQpProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  const size_t kNumParam = 3 * x_bounds_.size();
  const size_t kNumConstraint = kNumParam + 3 * (num_var_ - 1) + 3;
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  const size_t prime_offset = num_var_;
  const size_t pprime_offset = 2 * num_var_;

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  size_t constraint_index = 0;

  // d_i+1'' - d_i''
  for (size_t i = 0; i + 1 < num_var_; ++i) {
    columns[pprime_offset + i].emplace_back(constraint_index, -1.0);
    columns[pprime_offset + i + 1].emplace_back(constraint_index, 1.0);

    lower_bounds->at(constraint_index) =
        -max_x_third_order_derivative_ * delta_s_;
    upper_bounds->at(constraint_index) =
        max_x_third_order_derivative_ * delta_s_;
    ++constraint_index;
  }

  // d_i+1' - d_i' - 0.5 * ds * (d_i'' + d_i+1'')
  for (size_t i = 0; i + 1 < num_var_; ++i) {
    columns[prime_offset + i].emplace_back(constraint_index, -1.0);
    columns[prime_offset + i + 1].emplace_back(constraint_index, 1.0);
    columns[pprime_offset + i].emplace_back(constraint_index, -0.5 * delta_s_);
    columns[pprime_offset + i + 1].emplace_back(constraint_index,
                                                -0.5 * delta_s_);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
  for (size_t i = 0; i + 1 < num_var_; ++i) {
    columns[i].emplace_back(constraint_index, -1.0);
    columns[i + 1].emplace_back(constraint_index, 1.0);
    columns[prime_offset + i].emplace_back(constraint_index, -delta_s_);
    columns[pprime_offset + i].emplace_back(constraint_index,
                                            -delta_s_ * delta_s_ / 3.0);
    columns[pprime_offset + i + 1].emplace_back(constraint_index,
                                                -delta_s_ * delta_s_ / 6.0);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0];
  upper_bounds->at(constraint_index) = x_init_[0];
  ++constraint_index;

  columns[prime_offset].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1];
  upper_bounds->at(constraint_index) = x_init_[1];
  ++constraint_index;

  columns[pprime_offset].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2];
  upper_bounds->at(constraint_index) = x_init_[2];
  ++constraint_index;

  const double LARGE_VALUE = 2.0;
  for (size_t i = 0; i < kNumParam; ++i) {
    columns[i].emplace_back(constraint_index, 1.0);
    if (i < num_var_) {
      lower_bounds->at(constraint_index) = x_bounds_[i].first;
      upper_bounds->at(constraint_index) = x_bounds_[i].second;
    } else {
      lower_bounds->at(constraint_index) = -LARGE_VALUE;
      upper_bounds->at(constraint_index) = LARGE_VALUE;
    }
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  size_t ind_p = 0;
  for (size_t j = 0; j < kNumParam; ++j) {
    A_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[j]) {
      A_data->push_back(row_data_pair.second);
      A_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  A_indptr->push_back(ind_p);
}

}  // namespace planning
}  // namespace apollo
