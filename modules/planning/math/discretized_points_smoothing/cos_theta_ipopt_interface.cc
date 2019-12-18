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

#include "modules/planning/math/discretized_points_smoothing/cos_theta_ipopt_interface.h"

#include <algorithm>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {

CosThetaIpoptInterface::CosThetaIpoptInterface(
    std::vector<std::pair<double, double>> points, std::vector<double> bounds) {
  CHECK_GT(points.size(), 1);
  CHECK_GT(bounds.size(), 1);
  bounds_ = std::move(bounds);
  ref_points_ = std::move(points);
  num_of_points_ = ref_points_.size();
}

void CosThetaIpoptInterface::get_optimization_results(
    std::vector<double>* ptr_x, std::vector<double>* ptr_y) const {
  *ptr_x = opt_x_;
  *ptr_y = opt_y_;
}

bool CosThetaIpoptInterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                          int& nnz_h_lag,
                                          IndexStyleEnum& index_style) {
  // number of variables
  n = static_cast<int>(num_of_points_ << 1);
  num_of_variables_ = n;

  // number of constraints
  m = static_cast<int>(num_of_points_ << 1);
  num_of_constraints_ = m;

  if (use_automatic_differentiation_) {
    generate_tapes(n, m, &nnz_jac_g, &nnz_h_lag);
  } else {
    // number of nonzero constraint jacobian.
    nnz_jac_g = static_cast<int>(num_of_points_ << 1);
    nnz_jac_g_ = nnz_jac_g;

    // number of nonzero hessian and lagrangian.
    nnz_h_lag = static_cast<int>(num_of_points_ * 11 - 12);
    nnz_h_lag_ = nnz_h_lag;

    // load hessian structure
    hessian_strcuture();
  }

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool CosThetaIpoptInterface::get_bounds_info(int n, double* x_l, double* x_u,
                                             int m, double* g_l, double* g_u) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);
  // variables
  // a. for x, y
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    // x
    x_l[index] = -1e20;
    x_u[index] = 1e20;

    // y
    x_l[index + 1] = -1e20;
    x_u[index + 1] = 1e20;
  }

  // constraints
  // positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;

    x_lower = ref_points_[i].first - bounds_[i];
    x_upper = ref_points_[i].first + bounds_[i];
    y_lower = ref_points_[i].second - bounds_[i];
    y_upper = ref_points_[i].second + bounds_[i];

    // x
    g_l[index] = x_lower;
    g_u[index] = x_upper;

    // y
    g_l[index + 1] = y_lower;
    g_u[index + 1] = y_upper;
  }
  return true;
}

bool CosThetaIpoptInterface::get_starting_point(int n, bool init_x, double* x,
                                                bool init_z, double* z_L,
                                                double* z_U, int m,
                                                bool init_lambda,
                                                double* lambda) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  std::random_device rd;
  std::default_random_engine gen = std::default_random_engine(rd());
  std::normal_distribution<> dis{0, 0.05};
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    x[index] = ref_points_[i].first + dis(gen);
    x[index + 1] = ref_points_[i].second + dis(gen);
  }
  return true;
}

bool CosThetaIpoptInterface::eval_f(int n, const double* x, bool new_x,
                                    double& obj_value) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  if (use_automatic_differentiation_) {
    eval_obj(n, x, &obj_value);
    return true;
  }

  obj_value = 0.0;
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    obj_value +=
        (x[index] - ref_points_[i].first) * (x[index] - ref_points_[i].first) +
        (x[index + 1] - ref_points_[i].second) *
            (x[index + 1] - ref_points_[i].second);
  }
  for (size_t i = 0; i < num_of_points_ - 2; i++) {
    size_t findex = i << 1;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    obj_value -=
        weight_cos_included_angle_ *
        (((x[mindex] - x[findex]) * (x[lindex] - x[mindex])) +
         ((x[mindex + 1] - x[findex + 1]) * (x[lindex + 1] - x[mindex + 1]))) /
        std::sqrt((x[mindex] - x[findex]) * (x[mindex] - x[findex]) +
                  (x[mindex + 1] - x[findex + 1]) *
                      (x[mindex + 1] - x[findex + 1])) /
        std::sqrt((x[lindex] - x[mindex]) * (x[lindex] - x[mindex]) +
                  (x[lindex + 1] - x[mindex + 1]) *
                      (x[lindex + 1] - x[mindex + 1]));
  }
  return true;
}

bool CosThetaIpoptInterface::eval_grad_f(int n, const double* x, bool new_x,
                                         double* grad_f) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);

  if (use_automatic_differentiation_) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  std::fill(grad_f, grad_f + n, 0.0);
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    grad_f[index] = x[index] * 2 - ref_points_[i].first * 2;
    grad_f[index + 1] = x[index + 1] * 2 - ref_points_[i].second * 2;
  }

  for (size_t i = 0; i < num_of_points_ - 2; ++i) {
    size_t index = i << 1;
    double q1 = (x[index] - x[index + 2]) * (x[index] - x[index + 2]) +
                (x[index + 1] - x[index + 3]) * (x[index + 1] - x[index + 3]);
    double q2 = (x[index + 2] - x[index + 4]) * (x[index + 2] - x[index + 4]) +
                (x[index + 3] - x[index + 5]) * (x[index + 3] - x[index + 5]);
    double q3 =
        ((2 * x[index] - 2 * x[index + 2]) *
         ((x[index] - x[index + 2]) * (x[index + 2] - x[index + 4]) +
          (x[index + 1] - x[index + 3]) * (x[index + 3] - x[index + 5])));
    double q4 =
        ((2 * x[index + 1] - 2 * x[index + 3]) *
         ((x[index] - x[index + 2]) * (x[index + 2] - x[index + 4]) +
          (x[index + 1] - x[index + 3]) * (x[index + 3] - x[index + 5])));
    double q5 =
        ((2 * x[index + 2] - 2 * x[index + 4]) *
         ((x[index] - x[index + 2]) * (x[index + 2] - x[index + 4]) +
          (x[index + 1] - x[index + 3]) * (x[index + 3] - x[index + 5])));
    double q6 =
        ((2 * x[index + 3] - 2 * x[index + 5]) *
         ((x[index] - x[index + 2]) * (x[index + 2] - x[index + 4]) +
          (x[index + 1] - x[index + 3]) * (x[index + 3] - x[index + 5])));
    double sqrt_q1 = std::sqrt(q1);
    double sqrt_q2 = std::sqrt(q2);
    grad_f[index] += -weight_cos_included_angle_ *
                     ((x[index + 2] - x[index + 4]) / (sqrt_q1 * sqrt_q2) -
                      q3 / (2 * q1 * sqrt_q1 * sqrt_q2));
    grad_f[index + 1] += -weight_cos_included_angle_ *
                         ((x[index + 3] - x[index + 5]) / (sqrt_q1 * sqrt_q2) -
                          q4 / (2 * q1 * sqrt_q1 * sqrt_q2));
    grad_f[index + 2] +=
        -weight_cos_included_angle_ *
        ((x[index] - 2 * x[index + 2] + x[index + 4]) / (sqrt_q1 * sqrt_q2) +
         q3 / (2 * q1 * sqrt_q1 * sqrt_q2) - q5 / (2 * sqrt_q1 * q2 * sqrt_q2));
    grad_f[index + 3] +=
        -weight_cos_included_angle_ *
        ((x[index + 1] - 2 * x[index + 3] + x[index + 5]) /
             (sqrt_q1 * sqrt_q2) +
         q4 / (2 * q1 * sqrt_q1 * sqrt_q2) - q6 / (2 * sqrt_q1 * q2 * sqrt_q2));
    grad_f[index + 4] += -weight_cos_included_angle_ *
                         (q5 / (2 * sqrt_q1 * q2 * sqrt_q2) -
                          (x[index] - x[index + 2]) / (sqrt_q1 * sqrt_q2));
    grad_f[index + 5] += -weight_cos_included_angle_ *
                         (q6 / (2 * sqrt_q1 * q2 * sqrt_q2) -
                          (x[index + 1] - x[index + 3]) / (sqrt_q1 * sqrt_q2));
  }
  return true;
}

bool CosThetaIpoptInterface::eval_g(int n, const double* x, bool new_x, int m,
                                    double* g) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);

  if (use_automatic_differentiation_) {
    eval_constraints(n, x, m, g);
    return true;
  }
  // fill in the positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    g[index] = x[index];
    g[index + 1] = x[index + 1];
  }
  return true;
}

bool CosThetaIpoptInterface::eval_jac_g(int n, const double* x, bool new_x,
                                        int m, int nele_jac, int* iRow,
                                        int* jCol, double* values) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);

  if (use_automatic_differentiation_) {
    if (values == nullptr) {
      // return the structure of the jacobian
      for (int idx = 0; idx < nnz_jac_; idx++) {
        iRow[idx] = rind_g_[idx];
        jCol[idx] = cind_g_[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      // auto* rind_g = &rind_g_[0];
      // auto* cind_g = &cind_g_[0];
      // auto* jacval = &jacval_[0];
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac_, &rind_g_, &cind_g_, &jacval_,
                 options_g_);
      for (int idx = 0; idx < nnz_jac_; idx++) {
        values[idx] = jacval_[idx];
      }
    }
    return true;
  }

  if (values == nullptr) {
    // positional deviation constraints
    for (int i = 0; i < static_cast<int>(num_of_variables_); ++i) {
      iRow[i] = i;
      jCol[i] = i;
    }
  } else {
    std::fill(values, values + nnz_jac_g_, 0.0);
    // positional deviation constraints
    for (size_t i = 0; i < num_of_variables_; ++i) {
      values[i] = 1;
    }
  }
  return true;
}

bool CosThetaIpoptInterface::eval_h(int n, const double* x, bool new_x,
                                    double obj_factor, int m,
                                    const double* lambda, bool new_lambda,
                                    int nele_hess, int* iRow, int* jCol,
                                    double* values) {
  if (use_automatic_differentiation_) {
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
      // auto* rind_L = &rind_L_[0];
      // auto* cind_L = &cind_L_[0];
      // auto* hessval = &hessval_[0];
      set_param_vec(tag_L, m + 1, &obj_lam_[0]);
      sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L_, &rind_L_,
                  &cind_L_, &hessval_, options_L_);

      for (int idx = 0; idx < nnz_L_; idx++) {
        values[idx] = hessval_[idx];
      }
    }
    return true;
  }

  if (values == nullptr) {
    int index = 0;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j <= i; ++j) {
        iRow[index] = i;
        jCol[index] = j;
        index++;
      }
    }

    int shift = 0;
    for (int i = 6; i < static_cast<int>(num_of_variables_); ++i) {
      if (i % 2 == 0) {
        for (int j = 2 + shift; j <= 6 + shift; ++j) {
          iRow[index] = i;
          jCol[index] = j;
          index++;
        }

      } else {
        for (int j = 2 + shift; j <= 7 + shift; ++j) {
          iRow[index] = i;
          jCol[index] = j;
          index++;
        }
        shift += 2;
      }
    }
    CHECK_EQ(index, static_cast<size_t>(nele_hess));
  } else {
    std::fill(values, values + nele_hess, 0.0);
    // fill the included angle part of obj
    for (size_t i = 0; i < num_of_points_ - 2; ++i) {
      size_t topleft = i * 2;
      const double q5 = (2 * x[topleft] - 2 * x[topleft + 2]);
      const double q6 = (2 * x[topleft + 1] - 2 * x[topleft + 3]);
      const double q7 = (x[topleft] - 2 * x[topleft + 2] + x[topleft + 4]);
      const double q8 = (2 * x[topleft + 2] - 2 * x[topleft + 4]);
      const double q9 = (2 * x[topleft + 3] - 2 * x[topleft + 5]);
      const double q10 = (x[topleft + 1] - 2 * x[topleft + 3] + x[topleft + 5]);
      const double q11 = (x[topleft + 3] - x[topleft + 5]);
      const double q12 = (x[topleft] - x[topleft + 2]);
      const double q13 = (x[topleft + 2] - x[topleft + 4]);
      const double q14 = (x[topleft + 1] - x[topleft + 3]);
      const double q1 = q12 * q12 + q14 * q14;
      const double q2 = q13 * q13 + q11 * q11;
      const double q3 = (3 * q5 * q5 * (q12 * q13 + q14 * q11));
      const double q4 = (q12 * q13 + q14 * q11);
      const double sqrt_q1 = std::sqrt(q1);
      const double q1_sqrt_q1 = q1 * sqrt_q1;
      const double sqrt_q2 = std::sqrt(q2);
      const double q2_sqrt_q2 = q2 * sqrt_q2;
      const double square_q1 = q1 * q1;
      const double square_q2 = q2 * q2;
      const double sqrt_q1q2 = sqrt_q1 * sqrt_q2;
      const double q1_sqrt_q1q2 = q1 * sqrt_q1q2;
      const double sqrt_q1_q2_sqrt_q2 = sqrt_q1 * q2_sqrt_q2;
      const double q1_sqrt_q1_q2_sqrt_q2 = q1_sqrt_q1 * q2_sqrt_q2;
      values[idx_map_[std::make_pair(topleft, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (q3 / (4 * square_q1 * sqrt_q1q2) - q4 / q1_sqrt_q1q2 -
           (q5 * q13) / q1_sqrt_q1q2);
      values[idx_map_[std::make_pair(topleft + 1, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q5 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) -
           (q6 * q13) / (2 * q1_sqrt_q1q2) - (q5 * q11) / (2 * q1_sqrt_q1q2));
      values[idx_map_[std::make_pair(topleft + 2, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / sqrt_q1q2 + q4 / q1_sqrt_q1q2 - (q5 * q7) / (2 * q1_sqrt_q1q2) -
           q3 / (4 * square_q1 * sqrt_q1q2) + (q5 * q13) / (2 * q1_sqrt_q1q2) -
           (q8 * q13) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q5 * q8 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 3, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q13) / (2 * q1_sqrt_q1q2) - (q5 * q10) / (2 * q1_sqrt_q1q2) -
           (q9 * q13) / (2 * sqrt_q1_q2_sqrt_q2) -
           (3 * q5 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) +
           (q5 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q12) / (2 * q1_sqrt_q1q2) - 1 / sqrt_q1q2 +
           (q8 * q13) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q5 * q8 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q14) / (2 * q1_sqrt_q1q2) +
           (q9 * q13) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q5 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 1, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q6 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) -
           q4 / q1_sqrt_q1q2 - (q6 * q11) / q1_sqrt_q1q2);
      values[idx_map_[std::make_pair(topleft + 2, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q11) / (2 * q1_sqrt_q1q2) - (q6 * q7) / (2 * q1_sqrt_q1q2) -
           (q8 * q11) / (2 * sqrt_q1_q2_sqrt_q2) -
           (3 * q5 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) +
           (q8 * q6 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 3, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / sqrt_q1q2 + q4 / q1_sqrt_q1q2 - (q6 * q10) / (2 * q1_sqrt_q1q2) -
           (3 * q6 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) +
           (q6 * q11) / (2 * q1_sqrt_q1q2) -
           (q9 * q11) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q6 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q12) / (2 * q1_sqrt_q1q2) +
           (q8 * q11) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q8 * q6 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q14) / (2 * q1_sqrt_q1q2) - 1 / sqrt_q1q2 +
           (q9 * q11) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q6 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 2, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q7) / q1_sqrt_q1q2 -
           q4 / (sqrt_q1_q2_sqrt_q2)-q4 / q1_sqrt_q1q2 - 2 / sqrt_q1q2 -
           (q8 * q7) / (sqrt_q1_q2_sqrt_q2) + q3 / (4 * square_q1 * sqrt_q1q2) +
           (3 * q8 * q8 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           (q5 * q8 * q4) / (2 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 3, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q7) / (2 * q1_sqrt_q1q2) -
           (q9 * q7) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q5 * q10) / (2 * q1_sqrt_q1q2) -
           (q8 * q10) / (2 * sqrt_q1_q2_sqrt_q2) +
           (3 * q5 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) -
           (q5 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2) -
           (q8 * q6 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2) +
           (3 * q8 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / sqrt_q1q2 + q4 / (sqrt_q1_q2_sqrt_q2) +
           (q8 * q7) / (2 * sqrt_q1_q2_sqrt_q2) -
           (3 * q8 * q8 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           (q5 * q12) / (2 * q1_sqrt_q1q2) +
           (q8 * q12) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q5 * q8 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q9 * q7) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q5 * q14) / (2 * q1_sqrt_q1q2) +
           (q8 * q14) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q5 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2) -
           (3 * q8 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 3, topleft + 3)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q10) / q1_sqrt_q1q2 -
           q4 / (sqrt_q1_q2_sqrt_q2)-q4 / q1_sqrt_q1q2 - 2 / sqrt_q1q2 -
           (q9 * q10) / (sqrt_q1_q2_sqrt_q2) +
           (3 * q6 * q6 * q4) / (4 * square_q1 * sqrt_q1q2) +
           (3 * q9 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           (q6 * q9 * q4) / (2 * q1_sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft + 3)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q8 * q10) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q6 * q12) / (2 * q1_sqrt_q1q2) +
           (q9 * q12) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q8 * q6 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2) -
           (3 * q8 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 3)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / sqrt_q1q2 + q4 / (sqrt_q1_q2_sqrt_q2) +
           (q9 * q10) / (2 * sqrt_q1_q2_sqrt_q2) -
           (3 * q9 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           (q6 * q14) / (2 * q1_sqrt_q1q2) +
           (q9 * q14) / (2 * sqrt_q1_q2_sqrt_q2) +
           (q6 * q9 * q4) / (4 * q1_sqrt_q1_q2_sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 4, topleft + 4)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q8 * q8 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           q4 / (sqrt_q1_q2_sqrt_q2) - (q8 * q12) / (sqrt_q1_q2_sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 4)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q8 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           (q9 * q12) / (2 * sqrt_q1_q2_sqrt_q2) -
           (q8 * q14) / (2 * sqrt_q1_q2_sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 5, topleft + 5)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q9 * q9 * q4) / (4 * sqrt_q1 * square_q2 * sqrt_q2) -
           q4 / (sqrt_q1_q2_sqrt_q2) - (q9 * q14) / (sqrt_q1_q2_sqrt_q2));
    }

    // fill the deviation part of obj
    for (size_t i = 0; i < num_of_variables_; ++i) {
      values[idx_map_[std::make_pair(i, i)]] += obj_factor * 2;
    }
  }
  return true;
}

void CosThetaIpoptInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  opt_x_.reserve(num_of_points_);
  opt_y_.reserve(num_of_points_);
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    opt_x_.emplace_back(x[index]);
    opt_y_.emplace_back(x[index + 1]);
  }
  if (use_automatic_differentiation_) {
    free(rind_g_);
    free(cind_g_);
    free(rind_L_);
    free(cind_L_);
    free(jacval_);
    free(hessval_);
  }
}

void CosThetaIpoptInterface::set_automatic_differentiation_flag(
    const bool use_ad) {
  use_automatic_differentiation_ = use_ad;
}

void CosThetaIpoptInterface::set_weight_cos_included_angle(
    const double weight_cos_included_angle) {
  weight_cos_included_angle_ = weight_cos_included_angle;
}

void CosThetaIpoptInterface::set_weight_anchor_points(
    const double weight_anchor_points) {
  weight_anchor_points_ = weight_anchor_points;
}

void CosThetaIpoptInterface::set_weight_length(const double weight_length) {
  weight_length_ = weight_length;
}

void CosThetaIpoptInterface::hessian_strcuture() {
  size_t index = 0;
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j <= i; ++j) {
      idx_map_.insert(std::pair<std::pair<size_t, size_t>, size_t>(
          std::pair<size_t, size_t>(i, j), index));
      index++;
    }
  }

  size_t shift = 0;
  for (size_t i = 6; i < num_of_variables_; ++i) {
    if (i % 2 == 0) {
      for (size_t j = 2 + shift; j <= 6 + shift; ++j) {
        idx_map_.insert(std::pair<std::pair<size_t, size_t>, size_t>(
            std::pair<size_t, size_t>(i, j), index));
        index++;
      }

    } else {
      for (size_t j = 2 + shift; j <= 7 + shift; ++j) {
        idx_map_.insert(std::pair<std::pair<size_t, size_t>, size_t>(
            std::pair<size_t, size_t>(i, j), index));
        index++;
      }
      shift += 2;
    }
  }
}

//***************    start ADOL-C part ***********************************

/** Template to return the objective value */
template <class T>
bool CosThetaIpoptInterface::eval_obj(int n, const T* x, T* obj_value) {
  *obj_value = 0.0;
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    *obj_value +=
        weight_anchor_points_ *
        ((x[index] - ref_points_[i].first) * (x[index] - ref_points_[i].first) +
         (x[index + 1] - ref_points_[i].second) *
             (x[index + 1] - ref_points_[i].second));
  }
  for (size_t i = 0; i < num_of_points_ - 2; ++i) {
    size_t findex = i << 1;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    *obj_value -=
        weight_cos_included_angle_ *
        (((x[mindex] - x[findex]) * (x[lindex] - x[mindex])) +
         ((x[mindex + 1] - x[findex + 1]) * (x[lindex + 1] - x[mindex + 1]))) /
        (sqrt((x[mindex] - x[findex]) * (x[mindex] - x[findex]) +
              (x[mindex + 1] - x[findex + 1]) *
                  (x[mindex + 1] - x[findex + 1])) *
         sqrt((x[lindex] - x[mindex]) * (x[lindex] - x[mindex]) +
              (x[lindex + 1] - x[mindex + 1]) *
                  (x[lindex + 1] - x[mindex + 1])));
  }

  // Total length
  for (size_t i = 0; i < num_of_points_ - 1; ++i) {
    size_t findex = i << 1;
    size_t nindex = findex + 2;
    *obj_value +=
        weight_length_ *
        ((x[findex] - x[nindex]) * (x[findex] - x[nindex]) +
         (x[findex + 1] - x[nindex + 1]) * (x[findex + 1] - x[nindex + 1]));
  }
  return true;
}

/** Template to compute contraints */
template <class T>
bool CosThetaIpoptInterface::eval_constraints(int n, const T* x, int m, T* g) {
  // fill in the positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    g[index] = x[index];
    g[index + 1] = x[index + 1];
  }
  return true;
}

/** Method to generate the required tapes */
void CosThetaIpoptInterface::generate_tapes(int n, int m, int* nnz_jac_g,
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

  // rind_g_.clear();
  // cind_g_.clear();
  // rind_L_.clear();
  // cind_L_.clear();
  rind_g_ = nullptr;
  cind_g_ = nullptr;
  rind_L_ = nullptr;
  cind_L_ = nullptr;

  options_g_[0] = 0; /* sparsity pattern by index domains (default) */
  options_g_[1] = 0; /*                         safe mode (default) */
  options_g_[2] = 0;
  options_g_[3] = 0; /*                column compression (default) */

  // jacval_.clear();
  // hessval_.clear();
  jacval_ = nullptr;
  hessval_ = nullptr;

  // auto* rind_g = &rind_g_[0];
  // auto* cind_g = &cind_g_[0];
  // auto* jacval = &jacval_[0];
  // auto* rind_L = &rind_L_[0];
  // auto* cind_L = &cind_L_[0];
  // auto* hessval = &hessval_[0];

  sparse_jac(tag_g, m, n, 0, &xp[0], &nnz_jac_, &rind_g_, &cind_g_, &jacval_,
             options_g_);
  *nnz_jac_g = nnz_jac_;
  options_L_[0] = 0;
  options_L_[1] = 1;
  sparse_hess(tag_L, n, 0, &xp[0], &nnz_L_, &rind_L_, &cind_L_, &hessval_,
              options_L_);
  *nnz_h_lag = nnz_L_;
}

//***************    end   ADOL-C part ***********************************
}  // namespace planning
}  // namespace apollo
