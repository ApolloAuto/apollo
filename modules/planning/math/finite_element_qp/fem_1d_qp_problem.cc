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

#include "modules/planning/math/finite_element_qp/fem_1d_qp_problem.h"

#include <chrono>

#include "cybertron/common/log.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

bool Fem1dQpProblem::Init(const size_t num_var,
                          const std::array<double, 3>& x_init,
                          const double delta_s, const std::array<double, 5>& w,
                          const double max_x_third_order_derivative) {
  num_var_ = num_var;
  if (num_var_ < 4) {
    return false;
  }

  x_init_ = x_init;

  weight_.x_w = w[0];
  weight_.x_derivative_w = w[1];
  weight_.x_second_order_derivative_w = w[2];
  weight_.x_third_order_derivative_w = w[3];
  weight_.x_mid_line_w = w[4];

  max_x_third_order_derivative_ = max_x_third_order_derivative;

  // pre-calcualte some consts
  delta_s_ = delta_s;
  delta_s_sq_ = delta_s * delta_s;
  delta_s_tri_ = delta_s_sq_ * delta_s;
  delta_s_tetra_ = delta_s_sq_ * delta_s_sq_;
  delta_s_penta_ = delta_s_sq_ * delta_s_tri_;
  delta_s_hex_ = delta_s_tri_ * delta_s_tri_;

  return true;
}

bool Fem1dQpProblem::OptimizeWithOsqp(
    const size_t kernel_dim, const size_t num_affine_constraint,
    std::vector<c_float>& P_data, std::vector<c_int>& P_indices,    // NOLINT
    std::vector<c_int>& P_indptr, std::vector<c_float>& A_data,     // NOLINT
    std::vector<c_int>& A_indices, std::vector<c_int>& A_indptr,    // NOLINT
    std::vector<c_float>& lower_bounds,                             // NOLINT
    std::vector<c_float>& upper_bounds,                             // NOLINT
    std::vector<c_float>& q, OSQPData* data, OSQPWorkspace** work,  // NOLINT
    OSQPSettings* settings) {
  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;  // Change alpha parameter
  settings->eps_abs = 1.0e-04;
  settings->eps_rel = 1.0e-04;
  settings->max_iter = 5000;
  settings->polish = true;
  settings->verbose = FLAGS_enable_osqp_debug;

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  CHECK_EQ(upper_bounds.size(), lower_bounds.size());

  *work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(*work);
  return true;
}
}  // namespace planning
}  // namespace apollo
