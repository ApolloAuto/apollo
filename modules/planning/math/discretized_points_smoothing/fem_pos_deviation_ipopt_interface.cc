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
 * @file
 **/

#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_ipopt_interface.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

FemPosDeviationIpoptInterface::FemPosDeviationIpoptInterface(
    std::vector<std::pair<double, double>> points, std::vector<double> bounds) {
  CHECK_GT(points.size(), 1);
  CHECK_GT(bounds.size(), 1);
  bounds_around_refs_ = std::move(bounds);
  ref_points_ = std::move(points);
  num_of_points_ = ref_points_.size();
}

void FemPosDeviationIpoptInterface::get_optimization_results(
    std::vector<double>* ptr_x, std::vector<double>* ptr_y) const {
  *ptr_x = opt_x_;
  *ptr_y = opt_y_;
}

bool FemPosDeviationIpoptInterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                                 int& nnz_h_lag,
                                                 IndexStyleEnum& index_style) {
  CHECK_GT(num_of_points_, 3);
  // Number of variables
  // Variables include 2D points and curvature constraints slack variable
  num_of_slack_var_ = num_of_points_ - 2;
  n = static_cast<int>(num_of_points_ * 2 + num_of_slack_var_);
  num_of_variables_ = n;

  // Number of constraints
  // Constraints includes positional constraints, curvature constraints and
  // slack variable constraints
  num_of_curvature_constr_ = num_of_points_ - 2;
  num_of_slack_constr_ = num_of_points_ - 2;
  m = static_cast<int>(num_of_points_ * 2 + num_of_curvature_constr_ +
                       num_of_slack_constr_);
  num_of_constraints_ = m;

  // Indexes for variables and constraints,
  // Start index is actual first index and end index is one index after the
  // actual final index
  slack_var_start_index_ = num_of_points_ * 2;
  slack_var_end_index_ = slack_var_start_index_ + num_of_slack_var_;
  curvature_constr_start_index_ = num_of_points_ * 2;
  curvature_constr_end_index_ =
      curvature_constr_start_index_ + num_of_curvature_constr_;
  slack_constr_start_index_ = curvature_constr_end_index_;
  slack_constr_end_index_ = slack_constr_start_index_ + num_of_slack_constr_;

  generate_tapes(n, m, &nnz_jac_g, &nnz_h_lag);

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool FemPosDeviationIpoptInterface::get_bounds_info(int n, double* x_l,
                                                    double* x_u, int m,
                                                    double* g_l, double* g_u) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);
  // variables
  // a. for x, y
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    // x
    x_l[index] = -1e20;
    x_u[index] = 1e20;

    // y
    x_l[index + 1] = -1e20;
    x_u[index + 1] = 1e20;
  }
  // b. for slack var
  for (size_t i = slack_var_start_index_; i < slack_var_end_index_; ++i) {
    x_l[i] = -1e20;
    x_u[i] = 1e20;
  }

  // constraints
  // a. positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    // x
    g_l[index] = ref_points_[i].first - bounds_around_refs_[i];
    g_u[index] = ref_points_[i].first + bounds_around_refs_[i];

    // y
    g_l[index + 1] = ref_points_[i].second - bounds_around_refs_[i];
    g_u[index + 1] = ref_points_[i].second + bounds_around_refs_[i];
  }

  // b. curvature constraints
  double ref_total_length = 0.0;
  auto pre_point = ref_points_.front();
  for (size_t i = 1; i < num_of_points_; ++i) {
    auto cur_point = ref_points_[i];
    double x_diff = cur_point.first - pre_point.first;
    double y_diff = cur_point.second - pre_point.second;
    ref_total_length += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    pre_point = cur_point;
  }
  double average_delta_s =
      ref_total_length / static_cast<double>(num_of_points_ - 1);
  double curvature_constr_upper =
      average_delta_s * average_delta_s * curvature_constraint_;

  for (size_t i = curvature_constr_start_index_;
       i < curvature_constr_end_index_; ++i) {
    g_l[i] = -1e20;
    g_u[i] = curvature_constr_upper * curvature_constr_upper;
  }

  // c. slack var constraints
  for (size_t i = slack_constr_start_index_; i < slack_constr_end_index_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 1e20;
  }

  return true;
}

bool FemPosDeviationIpoptInterface::get_starting_point(int n, bool init_x,
                                                       double* x, bool init_z,
                                                       double* z_L, double* z_U,
                                                       int m, bool init_lambda,
                                                       double* lambda) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    x[index] = ref_points_[i].first;
    x[index + 1] = ref_points_[i].second;
  }

  for (size_t i = slack_var_start_index_; i < slack_var_end_index_; ++i) {
    x[i] = 0.0;
  }

  return true;
}

bool FemPosDeviationIpoptInterface::eval_f(int n, const double* x, bool new_x,
                                           double& obj_value) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);

  eval_obj(n, x, &obj_value);
  return true;
}

bool FemPosDeviationIpoptInterface::eval_grad_f(int n, const double* x,
                                                bool new_x, double* grad_f) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);

  gradient(tag_f, n, x, grad_f);
  return true;
}

bool FemPosDeviationIpoptInterface::eval_g(int n, const double* x, bool new_x,
                                           int m, double* g) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);

  eval_constraints(n, x, m, g);
  return true;
}

bool FemPosDeviationIpoptInterface::eval_jac_g(int n, const double* x,
                                               bool new_x, int m, int nele_jac,
                                               int* iRow, int* jCol,
                                               double* values) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);

  if (values == nullptr) {
    // return the structure of the jacobian
    for (int idx = 0; idx < nnz_jac_; idx++) {
      iRow[idx] = rind_g_[idx];
      jCol[idx] = cind_g_[idx];
    }
  } else {
    // return the values of the jacobian of the constraints
    sparse_jac(tag_g, m, n, 1, x, &nnz_jac_, &rind_g_, &cind_g_, &jacval_,
               options_g_);
    for (int idx = 0; idx < nnz_jac_; idx++) {
      values[idx] = jacval_[idx];
    }
  }
  return true;
}

bool FemPosDeviationIpoptInterface::eval_h(int n, const double* x, bool new_x,
                                           double obj_factor, int m,
                                           const double* lambda,
                                           bool new_lambda, int nele_hess,
                                           int* iRow, int* jCol,
                                           double* values) {
  if (values == nullptr) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.
    for (int idx = 0; idx < nnz_L_; idx++) {
      iRow[idx] = rind_L_[idx];
      jCol[idx] = cind_L_[idx];
    }
  } else {
    // return the values. This is a symmetric matrix, fill the lower left
    // triangle only
    obj_lam_[0] = obj_factor;
    for (int idx = 0; idx < m; idx++) {
      obj_lam_[1 + idx] = lambda[idx];
    }
    set_param_vec(tag_L, m + 1, &obj_lam_[0]);
    sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L_, &rind_L_,
                &cind_L_, &hessval_, options_L_);

    for (int idx = 0; idx < nnz_L_; idx++) {
      values[idx] = hessval_[idx];
    }
  }
  return true;
}

void FemPosDeviationIpoptInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  opt_x_.reserve(num_of_points_);
  opt_y_.reserve(num_of_points_);
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    opt_x_.emplace_back(x[index]);
    opt_y_.emplace_back(x[index + 1]);
  }
  free(rind_g_);
  free(cind_g_);
  free(rind_L_);
  free(cind_L_);
  free(jacval_);
  free(hessval_);
}

//***************    start ADOL-C part ***********************************

/** Template to return the objective value */
template <class T>
bool FemPosDeviationIpoptInterface::eval_obj(int n, const T* x, T* obj_value) {
  *obj_value = 0.0;

  // Distance to refs
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    *obj_value +=
        weight_ref_deviation_ *
        ((x[index] - ref_points_[i].first) * (x[index] - ref_points_[i].first) +
         (x[index + 1] - ref_points_[i].second) *
             (x[index + 1] - ref_points_[i].second));
  }

  // Fem_pos_deviation
  for (size_t i = 0; i + 2 < num_of_points_; ++i) {
    size_t findex = i * 2;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;

    *obj_value += weight_fem_pos_deviation_ *
                  (((x[findex] + x[lindex]) - 2.0 * x[mindex]) *
                       ((x[findex] + x[lindex]) - 2.0 * x[mindex]) +
                   ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]) *
                       ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]));
  }

  // Total length
  for (size_t i = 0; i + 1 < num_of_points_; ++i) {
    size_t findex = i * 2;
    size_t nindex = findex + 2;
    *obj_value +=
        weight_path_length_ *
        ((x[findex] - x[nindex]) * (x[findex] - x[nindex]) +
         (x[findex + 1] - x[nindex + 1]) * (x[findex + 1] - x[nindex + 1]));
  }

  // Slack variable minimization
  for (size_t i = slack_var_start_index_; i < slack_var_end_index_; ++i) {
    *obj_value += weight_curvature_constraint_slack_var_ * x[i];
  }

  return true;
}

/** Template to compute contraints */
template <class T>
bool FemPosDeviationIpoptInterface::eval_constraints(int n, const T* x, int m,
                                                     T* g) {
  // a. positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i * 2;
    g[index] = x[index];
    g[index + 1] = x[index + 1];
  }

  // b. curvature constraints
  for (size_t i = 0; i + 2 < num_of_points_; ++i) {
    size_t findex = i * 2;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    g[curvature_constr_start_index_ + i] =
        (((x[findex] + x[lindex]) - 2.0 * x[mindex]) *
             ((x[findex] + x[lindex]) - 2.0 * x[mindex]) +
         ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]) *
             ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1])) -
        x[slack_var_start_index_ + i];
  }

  // c. slack var constraints
  size_t slack_var_index = 0;
  for (size_t i = slack_constr_start_index_; i < slack_constr_end_index_; ++i) {
    g[i] = x[slack_var_start_index_ + slack_var_index];
    ++slack_var_index;
  }
  return true;
}

/** Method to generate the required tapes */
void FemPosDeviationIpoptInterface::generate_tapes(int n, int m, int* nnz_jac_g,
                                                   int* nnz_h_lag) {
  std::vector<double> xp(n, 0.0);
  std::vector<double> lamp(m, 0.0);
  std::vector<double> zl(m, 0.0);
  std::vector<double> zu(m, 0.0);
  std::vector<adouble> xa(n, 0.0);
  std::vector<adouble> g(m, 0.0);
  std::vector<double> lam(m, 0.0);

  double sig;
  adouble obj_value;

  double dummy = 0.0;
  obj_lam_.clear();
  obj_lam_.resize(m + 1, 0.0);
  get_starting_point(n, 1, &xp[0], 0, &zl[0], &zu[0], m, 0, &lamp[0]);

  // Trace on Objectives
  trace_on(tag_f);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_obj(n, &xa[0], &obj_value);
  obj_value >>= dummy;
  trace_off();

  // Trace on Jacobian
  trace_on(tag_g);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    g[idx] >>= dummy;
  }
  trace_off();

  // Trace on Hessian
  trace_on(tag_L);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  for (int idx = 0; idx < m; idx++) {
    lam[idx] = 1.0;
  }
  sig = 1.0;
  eval_obj(n, &xa[0], &obj_value);
  obj_value *= mkparam(sig);
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    obj_value += g[idx] * mkparam(lam[idx]);
  }
  obj_value >>= dummy;
  trace_off();

  rind_g_ = nullptr;
  cind_g_ = nullptr;
  rind_L_ = nullptr;
  cind_L_ = nullptr;

  options_g_[0] = 0; /* sparsity pattern by index domains (default) */
  options_g_[1] = 0; /*                         safe mode (default) */
  options_g_[2] = 0;
  options_g_[3] = 0; /*                column compression (default) */

  jacval_ = nullptr;
  hessval_ = nullptr;

  sparse_jac(tag_g, m, n, 0, &xp[0], &nnz_jac_, &rind_g_, &cind_g_, &jacval_,
             options_g_);
  *nnz_jac_g = nnz_jac_;
  options_L_[0] = 0;
  options_L_[1] = 1;
  sparse_hess(tag_L, n, 0, &xp[0], &nnz_L_, &rind_L_, &cind_L_, &hessval_,
              options_L_);
  *nnz_h_lag = nnz_L_;
}
}  // namespace planning
}  // namespace apollo
