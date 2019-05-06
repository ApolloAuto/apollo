/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/math/piecewise_jerk/path_time_qp_problem.h"

#include <algorithm>

#include "cyber/common/log.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

void PathTimeQpProblem::CalculateKernel(std::vector<c_float>* P_data,
                                        std::vector<c_int>* P_indices,
                                        std::vector<c_int>* P_indptr) {
  const int N = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * N;
  const int kNumValue = 4 * N - 1;
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // x(i)^2 * w_ref
  for (int i = 0; i < N - 1; ++i) {
    columns[i].emplace_back(i, weight_.x_ref_w);
    ++value_index;
  }
  // x(n-1)^2 * (w_ref + w_x)
  columns[N - 1].emplace_back(N - 1, weight_.x_ref_w + weight_.x_w);
  ++value_index;

  // x(i)'^2 * w_dx
  for (int i = 0; i < N; ++i) {
    columns[N + i].emplace_back(
        N + i, weight_.x_derivative_w * (1.0 + penalty_dx_[i]));
    ++value_index;
  }

  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  columns[2 * N].emplace_back(
      2 * N, weight_.x_second_order_derivative_w +
                 weight_.x_third_order_derivative_w / delta_s_sq_);
  ++value_index;
  for (int i = 1; i < N - 1; ++i) {
    columns[2 * N + i].emplace_back(
        2 * N + i, weight_.x_second_order_derivative_w +
                       2.0 * weight_.x_third_order_derivative_w / delta_s_sq_);
    ++value_index;
  }
  columns[3 * N - 1].emplace_back(
      3 * N - 1, weight_.x_second_order_derivative_w +
                     weight_.x_third_order_derivative_w / delta_s_sq_);
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < N - 1; ++i) {
    columns[2 * N + i].emplace_back(
        2 * N + i + 1, -2.0 * weight_.x_third_order_derivative_w / delta_s_sq_);
    ++value_index;
  }

  CHECK_EQ(value_index, kNumValue);

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void PathTimeQpProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int N = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * N;
  q->resize(kNumParam);
  for (int i = 0; i < N; ++i) {
    q->at(i) += -2.0 * weight_.x_ref_w * x_ref_[i];
    q->at(N + i) += -2.0 * weight_.x_derivative_w * x_derivative_desire;
  }
  q->at(N - 1) += -2.0 * weight_.x_w * x_end_[0];
  q->at(2 * N - 1) += -2.0 * weight_.x_derivative_w * x_end_[1];
  q->at(3 * N - 1) += -2.0 * weight_.x_second_order_derivative_w * x_end_[2];
}

}  // namespace planning
}  // namespace apollo
