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

/**
 * @file piecewise_jerk_path_ipopt_solver.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_ipopt_solver.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

PiecewiseJerkPathIpoptSolver::PiecewiseJerkPathIpoptSolver(
    const double x_init, const double dx_init, const double ddx_init,
    const double delta_s, const double dddx_max,
    std::vector<std::pair<double, double>> d_bounds) {
  CHECK_GT(d_bounds.size(), 1U);

  x_init_ = x_init;

  dx_init_ = dx_init;

  ddx_init_ = ddx_init;

  CHECK_GT(delta_s, 0.0);
  delta_s_ = delta_s;

  CHECK_GT(dddx_max, 0.0);
  dddx_max_ = dddx_max;

  num_of_points_ = static_cast<int>(d_bounds.size());

  num_of_variables_ = 3 * num_of_points_;

  num_of_constraints_ = 3 * (num_of_points_ - 1);

  d_bounds_ = std::move(d_bounds);
}

void PiecewiseJerkPathIpoptSolver::set_objective_weights(const double w_x,
                                                         const double w_dx,
                                                         const double w_ddx,
                                                         const double w_dddx,
                                                         const double w_obs) {
  w_x_ = w_x;

  w_dx_ = w_dx;

  w_ddx_ = w_ddx;

  w_dddx_ = w_dddx;

  w_obs_ = w_obs;
}

bool PiecewiseJerkPathIpoptSolver::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                                int& nnz_h_lag,
                                                IndexStyleEnum& index_style) {
  // variables
  n = num_of_variables_;

  // constraints
  m = num_of_constraints_;

  nnz_jac_g = 11 * (num_of_points_ - 1);

  // none zero hessian and lagrange
  nnz_h_lag = num_of_variables_ + num_of_points_ - 1;

  index_style = IndexStyleEnum::C_STYLE;

  return true;
}

bool PiecewiseJerkPathIpoptSolver::get_bounds_info(int n, double* x_l,
                                                   double* x_u, int m,
                                                   double* g_l, double* g_u) {
  const double LARGE_VALUE = 1.0;
  // bounds for variables
  // x bounds;
  for (int i = 0; i < num_of_points_; ++i) {
    x_l[i] = d_bounds_[i].first;
    x_u[i] = d_bounds_[i].second;
  }
  x_l[0] = x_init_;
  x_u[0] = x_init_;

  // dx bounds
  int offset = num_of_points_;
  for (int i = 0; i < num_of_points_; ++i) {
    x_l[offset + i] = -LARGE_VALUE;
    x_u[offset + i] = LARGE_VALUE;
  }
  x_l[offset] = dx_init_;
  x_u[offset] = dx_init_;

  // ddx bounds;
  offset = 2 * num_of_points_;
  for (int i = 0; i < num_of_points_; ++i) {
    x_l[offset + i] = -LARGE_VALUE;
    x_u[offset + i] = LARGE_VALUE;
  }
  x_l[offset] = ddx_init_;
  x_u[offset] = ddx_init_;

  // bounds for constraints
  // jerk bounds
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[i] = -dddx_max_;
    g_u[i] = dddx_max_;
  }

  // speed increment
  offset = num_of_points_ - 1;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = g_u[offset + i] = 0.0;
  }

  // position increment
  offset = 2 * (num_of_points_ - 1);
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = g_u[offset + i] = 0.0;
  }
  return true;
}

bool PiecewiseJerkPathIpoptSolver::get_starting_point(int n, bool init_x,
                                                      double* x, bool init_z,
                                                      double* z_L, double* z_U,
                                                      int m, bool init_lambda,
                                                      double* lambda) {
  CHECK_EQ(num_of_variables_, n);

  auto offset_dx = num_of_points_;
  auto offset_ddx = num_of_points_ + num_of_points_;
  for (int i = 0; i < num_of_points_; ++i) {
    x[i] = 0.0;
    x[offset_dx + i] = 0.0;
    x[offset_ddx + i] = 0.0;
  }

  x[0] = x_init_;
  x[offset_dx] = dx_init_;
  x[offset_ddx] = ddx_init_;
  return true;
}

bool PiecewiseJerkPathIpoptSolver::eval_f(int n, const double* x, bool new_x,
                                          double& obj_value) {
  obj_value = 0.0;

  int offset_dx = num_of_points_;
  int offset_ddx = 2 * num_of_points_;
  for (int i = 0; i < num_of_points_; ++i) {
    // d
    obj_value += x[i] * x[i] * w_x_;

    // d_prime
    obj_value += x[offset_dx + i] * x[offset_dx + i] * w_dx_;

    // d_pprime
    obj_value += x[offset_ddx + i] * x[offset_ddx + i] * w_ddx_;

    // d_ppprime
    if (i > 0) {
      // d_pprime1 - d_pprime0
      auto dddx = (x[offset_ddx + i] - x[offset_ddx + i - 1]) / delta_s_;
      obj_value += dddx * dddx * w_dddx_;
    }

    // obstacle
    auto dist = x[i] - (d_bounds_[i].first + d_bounds_[i].second) * 0.5;
    obj_value += dist * dist * w_obs_;
  }
  return true;
}

bool PiecewiseJerkPathIpoptSolver::eval_grad_f(int n, const double* x,
                                               bool new_x, double* grad_f) {
  std::fill(grad_f, grad_f + n, 0.0);

  int offset_dx = num_of_points_;
  int offset_ddx = 2 * num_of_points_;
  for (int i = 0; i < num_of_points_; ++i) {
    grad_f[i] = 2.0 * x[i] * w_x_ +
                2.0 *
                    (x[i] - (d_bounds_[i].first + d_bounds_[i].second) * 0.5) *
                    w_obs_;

    grad_f[offset_dx + i] = 2.0 * x[offset_dx + i] * w_dx_;

    grad_f[offset_ddx + i] = 2.0 * x[offset_ddx + i] * w_ddx_;
  }

  for (int i = 1; i < num_of_points_; ++i) {
    auto delta_ddx = x[offset_ddx + i] - x[offset_ddx + i - 1];
    grad_f[offset_ddx + i - 1] +=
        -2.0 * delta_ddx / delta_s_ / delta_s_ * w_dddx_;
    grad_f[offset_ddx + i] += 2.0 * delta_ddx / delta_s_ / delta_s_ * w_dddx_;
  }
  return true;
}

bool PiecewiseJerkPathIpoptSolver::eval_g(int n, const double* x, bool new_x,
                                          int m, double* g) {
  std::fill(g, g + m, 0.0);
  int offset_v = num_of_points_;
  int offset_a = 2 * num_of_points_;

  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g[i] = (x[offset_a + i + 1] - x[offset_a + i]) / delta_s_;
  }

  for (int i = 0; i + 1 < num_of_points_; ++i) {
    double p0 = x[i];
    double v0 = x[offset_v + i];
    double a0 = x[offset_a + i];

    double p1 = x[i + 1];
    double v1 = x[offset_v + i + 1];
    double a1 = x[offset_a + i + 1];

    double j = (a1 - a0) / delta_s_;

    double end_v = v0 + a0 * delta_s_ + 0.5 * j * delta_s_ * delta_s_;
    double end_p = p0 + v0 * delta_s_ + 0.5 * a0 * delta_s_ * delta_s_ +
                   j * delta_s_ * delta_s_ * delta_s_ / 6.0;

    auto v_diff = v1 - end_v;
    g[num_of_points_ - 1 + i] = v_diff;

    auto p_diff = p1 - end_p;
    g[2 * (num_of_points_ - 1) + i] = p_diff;
  }
  return true;
}

bool PiecewiseJerkPathIpoptSolver::eval_jac_g(int n, const double* x,
                                              bool new_x, int m, int nele_jac,
                                              int* iRow, int* jCol,
                                              double* values) {
  CHECK_EQ(n, num_of_variables_);
  CHECK_EQ(m, num_of_constraints_);

  auto offset_v = num_of_points_;
  auto offset_a = 2 * num_of_points_;

  if (values == nullptr) {
    int nz_index = 0;
    int constraint_index = 0;

    // jerk constraint
    // d_i+1'' - d_i''
    for (int variable_index = 0; variable_index + 1 < num_of_points_;
         ++variable_index) {
      // d_i''
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_a + variable_index;
      ++nz_index;

      // d_i+1''
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_a + variable_index + 1;
      ++nz_index;

      ++constraint_index;
    }

    // velocity constraint
    // d_i+1' - d_i' - 0.5 * ds * (d_i'' + d_i+1'')
    for (int variable_index = 0; variable_index + 1 < num_of_points_;
         ++variable_index) {
      // d_i'
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_v + variable_index;
      ++nz_index;
      // d_i+1'
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_v + variable_index + 1;
      ++nz_index;
      // d_i''
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_a + variable_index;
      ++nz_index;
      // d_i+1''
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_a + variable_index + 1;
      ++nz_index;

      ++constraint_index;
    }

    // position constraint
    // d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
    for (int variable_index = 0; variable_index + 1 < num_of_points_;
         ++variable_index) {
      // d_i
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = variable_index;
      ++nz_index;
      // d_i+1
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = variable_index + 1;
      ++nz_index;
      // d_i'
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_v + variable_index;
      ++nz_index;
      // d_i''
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_a + variable_index;
      ++nz_index;
      // d_i+1''
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = offset_a + variable_index + 1;
      ++nz_index;

      ++constraint_index;
    }

    CHECK_EQ(nz_index, nele_jac);
    CHECK_EQ(constraint_index, m);
  } else {
    std::fill(values, values + nele_jac, 0.0);
    int nz_index = 0;

    // fill jerk constraint
    // d_i+1'' - d_i''
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      values[nz_index] = -1.0 / delta_s_;
      ++nz_index;

      values[nz_index] = 1.0 / delta_s_;
      ++nz_index;
    }

    // fill velocity constraint
    // d_i+1' - d_i - 0.5 * ds * (d_i'' + d_i+1'')
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // d_i'
      values[nz_index] = -1.0;
      ++nz_index;
      // d_i+1'
      values[nz_index] = 1.0;
      ++nz_index;
      // d_i''
      values[nz_index] = -0.5 * delta_s_;
      ++nz_index;
      // d_i+1''
      values[nz_index] = -0.5 * delta_s_;
      ++nz_index;
    }

    // position constraint
    // d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // d_i
      values[nz_index] = -1.0;
      ++nz_index;
      // d_i+1
      values[nz_index] = 1.0;
      ++nz_index;
      // d_i'
      values[nz_index] = -delta_s_;
      ++nz_index;
      // d_i''
      values[nz_index] = -delta_s_ * delta_s_ / 3.0;
      ++nz_index;
      // d_i+1''
      values[nz_index] = -delta_s_ * delta_s_ / 6.0;
      ++nz_index;
    }
    CHECK_EQ(nz_index, nele_jac);
  }
  return true;
}

bool PiecewiseJerkPathIpoptSolver::eval_h(int n, const double* x, bool new_x,
                                          double obj_factor, int m,
                                          const double* lambda, bool new_lambda,
                                          int nele_hess, int* iRow, int* jCol,
                                          double* values) {
  CHECK_EQ(num_of_variables_ + num_of_points_ - 1, nele_hess);
  if (values == nullptr) {
    for (int i = 0; i < num_of_variables_; ++i) {
      iRow[i] = i;
      jCol[i] = i;
    }

    auto offset_nnz_index = num_of_variables_;
    auto offset_ddx = num_of_points_ * 2;
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      iRow[offset_nnz_index + i] = offset_ddx + i;
      jCol[offset_nnz_index + i] = offset_ddx + i + 1;
    }

  } else {
    std::fill(values, values + nele_hess, 0.0);
    for (int i = 0; i < num_of_points_; ++i) {
      values[i] = 2.0 * w_x_ + 2.0 * w_obs_;
    }

    for (int i = num_of_points_; i < 2 * num_of_points_; ++i) {
      values[i] = 2.0 * w_dx_;
    }

    for (int i = 2 * num_of_points_; i < 3 * num_of_points_; ++i) {
      values[i] = 2.0 * w_ddx_;
    }

    auto delta_s_square = delta_s_ * delta_s_;
    auto t = 2.0 * w_dddx_ / delta_s_square;
    auto t2 = t * 2;

    for (int i = 2 * num_of_points_; i < 3 * num_of_points_; ++i) {
      if (i == 2 * num_of_points_) {
        values[i] += t;
      } else {
        values[i] += t2;
      }
    }

    auto offset = num_of_variables_;
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      values[i + offset] = -t;
    }
  }
  return true;
}

void PiecewiseJerkPathIpoptSolver::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  opt_x_.reserve(num_of_points_);
  opt_dx_.reserve(num_of_points_);
  opt_ddx_.reserve(num_of_points_);

  int offset_prime = num_of_points_;
  int offset_pprime = offset_prime + num_of_points_;

  for (int i = 0; i < num_of_points_; ++i) {
    opt_x_.push_back(x[i]);
    opt_dx_.push_back(x[offset_prime + i]);
    opt_ddx_.push_back(x[offset_pprime + i]);
  }
}

void PiecewiseJerkPathIpoptSolver::GetOptimizationResult(
    std::vector<double>* ptr_opt_d, std::vector<double>* ptr_opt_d_prime,
    std::vector<double>* ptr_opt_d_pprime) const {
  *ptr_opt_d = opt_x_;
  *ptr_opt_d_prime = opt_dx_;
  *ptr_opt_d_pprime = opt_ddx_;
}

}  // namespace planning
}  // namespace apollo
