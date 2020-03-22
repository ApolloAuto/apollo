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

#include "modules/planning/lattice/trajectory_generation/lateral_osqp_optimizer.h"

#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

bool LateralOSQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(d_bounds, &P_data, &P_indices, &P_indptr);
  delta_s_ = delta_s;
  const int num_var = static_cast<int>(d_bounds.size());
  const int kNumParam = 3 * static_cast<int>(d_bounds.size());
  const int kNumConstraint = kNumParam + 3 * (num_var - 1) + 3;
  c_float lower_bounds[kNumConstraint];
  c_float upper_bounds[kNumConstraint];

  const int prime_offset = num_var;
  const int pprime_offset = 2 * num_var;

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);

  int constraint_index = 0;

  // d_i+1'' - d_i''
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[pprime_offset + i].emplace_back(constraint_index, -1.0);
    columns[pprime_offset + i + 1].emplace_back(constraint_index, 1.0);

    lower_bounds[constraint_index] =
        -FLAGS_lateral_third_order_derivative_max * delta_s_;
    upper_bounds[constraint_index] =
        FLAGS_lateral_third_order_derivative_max * delta_s_;
    ++constraint_index;
  }

  // d_i+1' - d_i' - 0.5 * ds * (d_i'' + d_i+1'')
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[prime_offset + i].emplace_back(constraint_index, -1.0);
    columns[prime_offset + i + 1].emplace_back(constraint_index, 1.0);
    columns[pprime_offset + i].emplace_back(constraint_index, -0.5 * delta_s_);
    columns[pprime_offset + i + 1].emplace_back(constraint_index,
                                                -0.5 * delta_s_);

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  // d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[i].emplace_back(constraint_index, -1.0);
    columns[i + 1].emplace_back(constraint_index, 1.0);
    columns[prime_offset + i].emplace_back(constraint_index, -delta_s_);
    columns[pprime_offset + i].emplace_back(constraint_index,
                                            -delta_s_ * delta_s_ / 3.0);
    columns[pprime_offset + i + 1].emplace_back(constraint_index,
                                                -delta_s_ * delta_s_ / 6.0);

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds[constraint_index] = d_state[0];
  upper_bounds[constraint_index] = d_state[0];
  ++constraint_index;

  columns[prime_offset].emplace_back(constraint_index, 1.0);
  lower_bounds[constraint_index] = d_state[1];
  upper_bounds[constraint_index] = d_state[1];
  ++constraint_index;

  columns[pprime_offset].emplace_back(constraint_index, 1.0);
  lower_bounds[constraint_index] = d_state[2];
  upper_bounds[constraint_index] = d_state[2];
  ++constraint_index;

  const double LARGE_VALUE = 2.0;
  for (int i = 0; i < kNumParam; ++i) {
    columns[i].emplace_back(constraint_index, 1.0);
    if (i < num_var) {
      lower_bounds[constraint_index] = d_bounds[i].first;
      upper_bounds[constraint_index] = d_bounds[i].second;
    } else {
      lower_bounds[constraint_index] = -LARGE_VALUE;
      upper_bounds[constraint_index] = LARGE_VALUE;
    }
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  // change affine_constraint to CSC format
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  int ind_p = 0;
  for (int j = 0; j < kNumParam; ++j) {
    A_indptr.push_back(ind_p);
    for (const auto& row_data_pair : columns[j]) {
      A_data.push_back(row_data_pair.second);
      A_indices.push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  A_indptr.push_back(ind_p);

  // offset
  double q[kNumParam];
  for (int i = 0; i < kNumParam; ++i) {
    if (i < num_var) {
      q[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance *
             (d_bounds[i].first + d_bounds[i].second);
    } else {
      q[i] = 0.0;
    }
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
  settings->verbose = FLAGS_enable_osqp_debug;

  // Populate data
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = kNumParam;
  data->m = kNumConstraint;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds;
  data->u = upper_bounds;

  // Workspace
  OSQPWorkspace* work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  // extract primal results
  for (int i = 0; i < num_var; ++i) {
    opt_d_.push_back(work->solution->x[i]);
    opt_d_prime_.push_back(work->solution->x[i + num_var]);
    opt_d_pprime_.push_back(work->solution->x[i + 2 * num_var]);
  }
  opt_d_prime_[num_var - 1] = 0.0;
  opt_d_pprime_[num_var - 1] = 0.0;

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void LateralOSQPOptimizer::CalculateKernel(
    const std::vector<std::pair<double, double>>& d_bounds,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  const int kNumParam = 3 * static_cast<int>(d_bounds.size());
  P_data->resize(kNumParam);
  P_indices->resize(kNumParam);
  P_indptr->resize(kNumParam + 1);

  for (int i = 0; i < kNumParam; ++i) {
    if (i < static_cast<int>(d_bounds.size())) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_offset +
                      2.0 * FLAGS_weight_lateral_obstacle_distance;
    } else if (i < 2 * static_cast<int>(d_bounds.size())) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_derivative;
    } else {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_second_order_derivative;
    }
    P_indices->at(i) = i;
    P_indptr->at(i) = i;
  }
  P_indptr->at(kNumParam) = kNumParam;
  CHECK_EQ(P_data->size(), P_indices->size());
}

}  // namespace planning
}  // namespace apollo
