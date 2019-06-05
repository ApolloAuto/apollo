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

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_problem.h"

#include <algorithm>

#include "cyber/common/log.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
constexpr double kMaxVariableRange = 1.0e10;
}  // namespace

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) {
  CHECK_GE(num_of_knots, 2);
  num_of_knots_ = num_of_knots;

  x_init_ = x_init;

  delta_s_ = delta_s;

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));
}

bool PiecewiseJerkProblem::OptimizeWithOsqp(
    const size_t kernel_dim, const size_t num_affine_constraint,
    std::vector<c_float>& P_data, std::vector<c_int>& P_indices,    // NOLINT
    std::vector<c_int>& P_indptr, std::vector<c_float>& A_data,     // NOLINT
    std::vector<c_int>& A_indices, std::vector<c_int>& A_indptr,    // NOLINT
    std::vector<c_float>& lower_bounds,                             // NOLINT
    std::vector<c_float>& upper_bounds,                             // NOLINT
    std::vector<c_float>& q, OSQPData* data, OSQPWorkspace** work,  // NOLINT
    OSQPSettings* settings) {
  CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  *work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(*work);

  auto status = (*work)->info->status_val;

  if (status < 0) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  if (status != 1 && status != 2) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  return true;
}

void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);
}

void PiecewiseJerkProblem::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}

void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);
}

void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,
                                        const double x_upper_bound) {
  for (auto& x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,
                                         const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,
                                          const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}

bool PiecewiseJerkProblem::Optimize(const int max_iter) {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  OSQPSettings* settings = SolverDefaultSettings();
  settings->max_iter = max_iter;

  OSQPWorkspace* work = nullptr;

  bool res =
      OptimizeWithOsqp(3 * num_of_knots_, lower_bounds.size(), P_data,
                       P_indices, P_indptr, A_data, A_indices, A_indptr,
                       lower_bounds, upper_bounds, q, data, &work, settings);
  if (res == false || work == nullptr || work->solution == nullptr) {
    AERROR << "Failed to find solution.";
    // Cleanup
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);

    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = work->solution->x[i];
    dx_.at(i) = work->solution->x[i + num_of_knots_];
    ddx_.at(i) = work->solution->x[i + 2 * num_of_knots_];
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const int N = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * N;
  const int kNumConstraint = kNumParam + 3 * (N - 1) + 3;
  lower_bounds->resize(kNumConstraint);
  upper_bounds->resize(kNumConstraint);

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int constraint_index = 0;

  // set x, x', x'' bounds
  for (int i = 0; i < kNumParam; ++i) {
    columns[i].emplace_back(constraint_index, 1.0);
    if (i < N) {
      lower_bounds->at(constraint_index) = std::get<0>(x_bounds_[i]);
      upper_bounds->at(constraint_index) = std::get<1>(x_bounds_[i]);
    } else if (i < 2 * N) {
      lower_bounds->at(constraint_index) = std::get<0>(dx_bounds_[i - N]);
      upper_bounds->at(constraint_index) = std::get<1>(dx_bounds_[i - N]);
    } else {
      lower_bounds->at(constraint_index) = std::get<0>(ddx_bounds_[i - 2 * N]);
      upper_bounds->at(constraint_index) = std::get<1>(ddx_bounds_[i - 2 * N]);
    }
    ++constraint_index;
  }
  CHECK_EQ(constraint_index, kNumParam);

  // x(i+1)'' - x(i)'' - x(i)''' * delta_s = 0
  for (int i = 0; i + 1 < N; ++i) {
    columns[2 * N + i].emplace_back(constraint_index, -1.0);
    columns[2 * N + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = dddx_bound_.first * delta_s_;
    upper_bounds->at(constraint_index) = dddx_bound_.second * delta_s_;
    ++constraint_index;
  }

  // x(i+1)' - x(i)' - 0.5 * delta_s * (x(i+1)'' + x(i)'') = 0
  for (int i = 0; i + 1 < N; ++i) {
    columns[N + i].emplace_back(constraint_index, -1.0);
    columns[N + i + 1].emplace_back(constraint_index, 1.0);
    columns[2 * N + i].emplace_back(constraint_index, -0.5 * delta_s_);
    columns[2 * N + i + 1].emplace_back(constraint_index, -0.5 * delta_s_);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - x(i)'*delta_s - 1/3*x(i)''*delta_s^2 - 1/6*x(i)''*delta_s^2
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < N; ++i) {
    columns[i].emplace_back(constraint_index, -1.0);
    columns[i + 1].emplace_back(constraint_index, 1.0);
    columns[N + i].emplace_back(constraint_index, -delta_s_);
    columns[2 * N + i].emplace_back(constraint_index, -delta_s_sq_ / 3.0);
    columns[2 * N + i + 1].emplace_back(constraint_index, -delta_s_sq_ / 6.0);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // constrain on x_init
  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0];
  upper_bounds->at(constraint_index) = x_init_[0];
  ++constraint_index;

  columns[N].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1];
  upper_bounds->at(constraint_index) = x_init_[1];
  ++constraint_index;

  columns[2 * N].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2];
  upper_bounds->at(constraint_index) = x_init_[2];
  ++constraint_index;

  CHECK_EQ(constraint_index, kNumConstraint);

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      A_data->push_back(row_data_pair.second);
      A_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  A_indptr->push_back(ind_p);
}

OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = FLAGS_enable_osqp_debug;
  settings->scaled_termination = true;
  return settings;
}

}  // namespace planning
}  // namespace apollo
