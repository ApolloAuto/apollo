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

#include "modules/planning/reference_line/cos_theta_problem_interface.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

CosThetaProbleminterface::CosThetaProbleminterface(
    std::vector<Eigen::Vector2d> points, std::vector<double> lateral_bounds) {
  init_points_ = std::move(points);
  num_of_points_ = init_points_.size();
  lateral_bounds_ = std::move(lateral_bounds);
  CHECK_GT(num_of_points_, 1);
}

void CosThetaProbleminterface::get_optimization_results(
    std::vector<double>* ptr_x, std::vector<double>* ptr_y) const {
  *ptr_x = opt_x_;
  *ptr_y = opt_y_;
}

bool CosThetaProbleminterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                            int& nnz_h_lag,
                                            IndexStyleEnum& index_style) {
  // number of variables
  n = static_cast<int>(num_of_points_ << 1);
  num_of_variables_ = n;

  // number of constraints
  m = static_cast<int>(num_of_points_ << 1);
  num_of_constraints_ = m;

  if (use_automatic_differentiation_) {
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);
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

bool CosThetaProbleminterface::get_bounds_info(int n, double* x_l, double* x_u,
                                               int m, double* g_l,
                                               double* g_u) {
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
    double bound = std::min(lateral_bounds_[i], default_max_point_deviation_);
    double radius_ratio = std::sqrt(2);
    double relax_square = relax_ / radius_ratio;
    double bound_square = bound / radius_ratio;

    if (i == 0 && has_fixed_start_point_) {
      x_lower = start_x_ - relax_square;
      x_upper = start_x_ + relax_square;
      y_lower = start_y_ - relax_square;
      y_upper = start_y_ + relax_square;
    } else if (i + 1 == num_of_points_ && has_fixed_end_point_) {
      x_lower = end_x_ - relax_square;
      x_upper = end_x_ + relax_square;
      y_lower = end_y_ - relax_square;
      y_upper = end_y_ + relax_square;
    } else {
      x_lower = init_points_[i].x() - bound_square;
      x_upper = init_points_[i].x() + bound_square;
      y_lower = init_points_[i].y() - bound_square;
      y_upper = init_points_[i].y() + bound_square;
    }
    // x
    g_l[index] = x_lower;
    g_u[index] = x_upper;

    // y
    g_l[index + 1] = y_lower;
    g_u[index + 1] = y_upper;
  }
  return true;
}

bool CosThetaProbleminterface::get_starting_point(int n, bool init_x, double* x,
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
    x[index] = init_points_[i].x() + dis(gen);
    x[index + 1] = init_points_[i].y() + dis(gen);
  }
  return true;
}

bool CosThetaProbleminterface::eval_f(int n, const double* x, bool new_x,
                                      double& obj_value) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  if (use_automatic_differentiation_) {
    eval_obj(n, x, obj_value);
    return true;
  }

  obj_value = 0.0;
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    obj_value +=
        (x[index] - init_points_[i].x()) * (x[index] - init_points_[i].x()) +
        (x[index + 1] - init_points_[i].y()) *
            (x[index + 1] - init_points_[i].y());
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

bool CosThetaProbleminterface::eval_grad_f(int n, const double* x, bool new_x,
                                           double* grad_f) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);

  if (use_automatic_differentiation_) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  std::fill(grad_f, grad_f + n, 0.0);
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    grad_f[index] = x[index] * 2 - init_points_[i].x() * 2;
    grad_f[index + 1] = x[index + 1] * 2 - init_points_[i].y() * 2;
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

bool CosThetaProbleminterface::eval_g(int n, const double* x, bool new_x, int m,
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

bool CosThetaProbleminterface::eval_jac_g(int n, const double* x, bool new_x,
                                          int m, int nele_jac, int* iRow,
                                          int* jCol, double* values) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);

  if (use_automatic_differentiation_) {
    if (values == nullptr) {
      // return the structure of the jacobian
      for (int idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval,
                 options_g);
      for (int idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
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

bool CosThetaProbleminterface::eval_h(int n, const double* x, bool new_x,
                                      double obj_factor, int m,
                                      const double* lambda, bool new_lambda,
                                      int nele_hess, int* iRow, int* jCol,
                                      double* values) {
  if (use_automatic_differentiation_) {
    if (values == nullptr) {
      // return the structure. This is a symmetric matrix, fill the lower left
      // triangle only.
      for (int idx = 0; idx < nnz_L; idx++) {
        iRow[idx] = rind_L[idx];
        jCol[idx] = cind_L[idx];
      }
    } else {
      // return the values. This is a symmetric matrix, fill the lower left
      // triangle only
      obj_lam[0] = obj_factor;
      for (int idx = 0; idx < m; idx++) obj_lam[1 + idx] = lambda[idx];
      set_param_vec(tag_L, m + 1, obj_lam);
      sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L, &rind_L, &cind_L,
                  &hessval, options_L);

      for (int idx = 0; idx < nnz_L; idx++) {
        values[idx] = hessval[idx];
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

void CosThetaProbleminterface::finalize_solution(
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
    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
  }
}

void CosThetaProbleminterface::set_default_max_point_deviation(
    const double max_point_deviation) {
  default_max_point_deviation_ = max_point_deviation;
}

void CosThetaProbleminterface::set_relax_end_constraint(const double relax) {
  relax_ = relax;
}

void CosThetaProbleminterface::set_automatic_differentiation_flag(
    const bool use_ad) {
  use_automatic_differentiation_ = use_ad;
}

void CosThetaProbleminterface::set_start_point(const double x, const double y) {
  has_fixed_start_point_ = true;
  start_x_ = x;
  start_y_ = y;
}

void CosThetaProbleminterface::set_end_point(const double x, const double y) {
  has_fixed_end_point_ = true;
  end_x_ = x;
  end_y_ = y;
}

void CosThetaProbleminterface::set_weight_cos_included_angle(
    const double weight_cos_included_angle) {
  weight_cos_included_angle_ = weight_cos_included_angle;
}

void CosThetaProbleminterface::hessian_strcuture() {
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
bool CosThetaProbleminterface::eval_obj(int n, const T* x, T& obj_value) {
  obj_value = 0.0;
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    obj_value +=
        (x[index] - init_points_[i].x()) * (x[index] - init_points_[i].x()) +
        (x[index + 1] - init_points_[i].y()) *
            (x[index + 1] - init_points_[i].y());
  }
  for (size_t i = 0; i < num_of_points_ - 2; ++i) {
    size_t findex = i << 1;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    obj_value -=
        weight_cos_included_angle_ *
        (((x[mindex] - x[findex]) * (x[lindex] - x[mindex])) +
         ((x[mindex + 1] - x[findex + 1]) * (x[lindex + 1] - x[mindex + 1]))) /
        (sqrt((x[mindex] - x[findex]) * (x[mindex] - x[findex]) +
              (x[mindex + 1] - x[findex + 1]) *
                  (x[mindex + 1] - x[findex + 1])) *
         sqrt((x[lindex] - x[mindex]) * (x[lindex] - x[mindex]) +
              (x[lindex + 1] - x[mindex + 1]) *
                  (x[lindex + 1] - x[mindex + 1])));

    // obj_value +=
    //     weight_cos_included_angle_ *
    //     acos(
    //         (((x[mindex] - x[findex]) * (x[lindex] - x[mindex])) +
    //          ((x[mindex + 1] - x[findex + 1]) *
    //           (x[lindex + 1] - x[mindex + 1]))) /
    //         (1e-8 + sqrt((x[mindex] - x[findex]) * (x[mindex] - x[findex]) +
    //                      (x[mindex + 1] - x[findex + 1]) *
    //                          (x[mindex + 1] - x[findex + 1])) *
    //                     sqrt((x[lindex] - x[mindex]) * (x[lindex] -
    //                     x[mindex]) +
    //                          (x[lindex + 1] - x[mindex + 1]) *
    //                              (x[lindex + 1] - x[mindex + 1]))));

    // obj_value -=
    //     weight_cos_included_angle_ *
    //     (((x[mindex] - x[findex]) * (x[lindex] - x[mindex])) +
    //      ((x[mindex + 1] - x[findex + 1]) * (x[lindex + 1] - x[mindex +
    //      1])));
  }

  // for (size_t i = 0; i < num_of_points_ - 1; ++i) {
  //   size_t findex = i << 1;
  //   size_t nindex = findex + 2;
  //   obj_value +=
  //       (x[findex] - x[nindex]) * (x[findex] - x[nindex]) +
  //       (x[findex + 1] - x[nindex + 1]) * (x[findex + 1] - x[nindex + 1]);
  // }
  return true;
}

/** Template to compute contraints */
template <class T>
bool CosThetaProbleminterface::eval_constraints(int n, const T* x, int m,
                                                T* g) {
  // fill in the positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    g[index] = x[index];
    g[index + 1] = x[index + 1];
  }
  return true;
}

/** Method to generate the required tapes */
void CosThetaProbleminterface::generate_tapes(int n, int m, int& nnz_jac_g,
                                              int& nnz_h_lag) {
  Ipopt::Number* xp = new double[n];
  Ipopt::Number* lamp = new double[m];
  Ipopt::Number* zl = new double[m];
  Ipopt::Number* zu = new double[m];
  adouble* xa = new adouble[n];
  adouble* g = new adouble[m];
  double* lam = new double[m];
  double sig;
  adouble obj_value;

  double dummy;
  obj_lam = new double[m + 1];
  get_starting_point(n, 1, xp, 0, zl, zu, m, 0, lamp);
  trace_on(tag_f);

  for (int idx = 0; idx < n; idx++) xa[idx] <<= xp[idx];
  eval_obj(n, xa, obj_value);
  obj_value >>= dummy;
  trace_off();

  trace_on(tag_g);

  for (int idx = 0; idx < n; idx++) xa[idx] <<= xp[idx];
  eval_constraints(n, xa, m, g);
  for (int idx = 0; idx < m; idx++) g[idx] >>= dummy;
  trace_off();
  trace_on(tag_L);

  for (int idx = 0; idx < n; idx++) xa[idx] <<= xp[idx];
  for (int idx = 0; idx < m; idx++) lam[idx] = 1.0;
  sig = 1.0;
  eval_obj(n, xa, obj_value);
  obj_value *= mkparam(sig);
  eval_constraints(n, xa, m, g);

  for (int idx = 0; idx < m; idx++) obj_value += g[idx] * mkparam(lam[idx]);
  obj_value >>= dummy;
  trace_off();
  rind_g = NULL;
  cind_g = NULL;
  rind_L = NULL;
  cind_L = NULL;
  options_g[0] = 0; /* sparsity pattern by index domains (default) */
  options_g[1] = 0; /*                         safe mode (default) */
  options_g[2] = 0;
  options_g[3] = 0; /*                column compression (default) */

  jacval = NULL;
  hessval = NULL;
  sparse_jac(tag_g, m, n, 0, xp, &nnz_jac, &rind_g, &cind_g, &jacval,
             options_g);
  nnz_jac_g = nnz_jac;
  options_L[0] = 0;
  options_L[1] = 1;
  sparse_hess(tag_L, n, 0, xp, &nnz_L, &rind_L, &cind_L, &hessval, options_L);
  nnz_h_lag = nnz_L;
  delete[] lam;
  delete[] g;
  delete[] xa;
  delete[] zu;
  delete[] zl;
  delete[] lamp;
  delete[] xp;
}

//***************    end   ADOL-C part ***********************************
}  // namespace planning
}  // namespace apollo
