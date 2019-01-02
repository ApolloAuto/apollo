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

#include <algorithm>

#include "cyber/common/log.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
constexpr double kMaxVariableRange = 1e10;
}  // namespace

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

  x_bounds_.resize(num_var_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  dx_bounds_.resize(num_var_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  ddx_bounds_.resize(num_var_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  is_init_ = true;
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
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
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

void Fem1dQpProblem::ProcessBound(
    const std::vector<std::tuple<double, double, double>>& src,
    std::vector<std::pair<double, double>>* dst) {
  DCHECK_NOTNULL(dst);

  *dst = std::vector<std::pair<double, double>>(
      num_var_, std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  for (size_t i = 0; i < src.size(); ++i) {
    size_t index = static_cast<size_t>(std::get<0>(src[i]) / delta_s_ + 0.5);
    if (index < dst->size()) {
      dst->at(index).first =
          std::max(dst->at(index).first, std::get<1>(src[i]));
      dst->at(index).second =
          std::min(dst->at(index).second, std::get<2>(src[i]));
    }
  }
}

// x_bounds: tuple(s, lower_bounds, upper_bounds)
void Fem1dQpProblem::SetVariableBounds(
    const std::vector<std::tuple<double, double, double>>& x_bounds) {
  if (!is_init_) {
    AERROR << "Please Init() before setting bounds.";
    return;
  }
  ProcessBound(x_bounds, &x_bounds_);
}

// dx_bounds: tuple(s, lower_bounds, upper_bounds)
void Fem1dQpProblem::SetVariableDerivativeBounds(
    const std::vector<std::tuple<double, double, double>>& dx_bounds) {
  if (!is_init_) {
    AERROR << "Please Init() before setting bounds.";
    return;
  }
  ProcessBound(dx_bounds, &dx_bounds_);
}

// ddx_bounds: tuple(s, lower_bounds, upper_bounds)
void Fem1dQpProblem::SetVariableSecondOrderDerivativeBounds(
    const std::vector<std::tuple<double, double, double>>& ddx_bounds) {
  if (!is_init_) {
    AERROR << "Please Init() before setting bounds.";
    return;
  }
  ProcessBound(ddx_bounds, &ddx_bounds_);
}

void Fem1dQpProblem::SetOutputResolution(const double resolution) {
  // It is assumed that the third order derivative of x is const between each s
  // positions
  const double kEps = 1e-12;
  if (resolution < kEps || x_.empty()) {
    return;
  }
  std::vector<double> new_x;
  std::vector<double> new_dx;
  std::vector<double> new_ddx;
  std::vector<double> new_dddx;

  const double total_s = delta_s_ * (static_cast<double>(x_.size()) - 1.0);
  for (double s = resolution; s < total_s; s += resolution) {
    const size_t idx = static_cast<size_t>(std::floor(s / delta_s_));
    const double ds = s - delta_s_ * static_cast<double>(idx);

    double x = 0.0;
    double dx = 0.0;
    double d2x = 0.0;
    double d3x = 0.0;

    if (idx == 0) {
      d3x = x_third_order_derivative_.front();
      d2x = x_init_[2] + d3x * ds;
      dx = x_init_[1] + x_init_[2] * ds + 0.5 * d3x * ds * ds;
      x = x_init_[0] + x_init_[1] * ds + 0.5 * x_init_[2] * ds * ds +
          d3x * ds * ds * ds / 6.0;
    } else {
      d3x = x_third_order_derivative_[idx - 1];
      d2x = x_second_order_derivative_[idx - 1] + d3x * ds;
      dx = x_derivative_[idx - 1] + x_second_order_derivative_[idx - 1] * ds +
           0.5 * d3x * ds * ds;
      x = x_[idx - 1] + x_derivative_[idx - 1] * ds +
          0.5 * x_second_order_derivative_[idx - 1] * ds * ds +
          d3x * ds * ds * ds / 6.0;
    }

    new_x.push_back(x);
    new_dx.push_back(dx);
    new_ddx.push_back(d2x);
    new_ddx.push_back(d3x);
  }
  x_ = new_x;
  x_derivative_ = new_dx;
  x_second_order_derivative_ = new_ddx;
  x_third_order_derivative_ = new_dddx;
}

}  // namespace planning
}  // namespace apollo
