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

#include "modules/planning/reference_line/cosTheta_problem_interface.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <random>
#include "modules/common/log.h"

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
  n = num_of_points_ * 2;
  num_of_variables_ = n;

  // number of constraints
  m = num_of_points_;
  // m = 0;
  num_of_constraints_ = m;

  // number of nonzero constraint jacobian.
  nnz_jac_g = num_of_points_ * 2;
  // nnz_jac_g = 0;
  nnz_jac_g_ = nnz_jac_g;

  // number of nonzero hessian and lagrangian.
  nnz_h_lag = num_of_points_ * 11 - 12;
  nnz_h_lag_ = nnz_h_lag;

  // load hessian structure
  hessian_strcuture();

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool CosThetaProbleminterface::get_bounds_info(int n, double* x_l, double* x_u,
                                               int m, double* g_l,
                                               double* g_u) {
  CHECK_EQ(static_cast<std::size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<std::size_t>(m), num_of_constraints_);
  // variables
  // a. for x, y
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 2;
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;
    double bound = std::min(lateral_bounds_[i], default_max_point_deviation_);
    // / std::pow(2,0.5)

    if (i == 0 && has_fixed_start_point_) {
      x_lower = start_x_ - relax_;
      x_upper = start_x_ + relax_;
      y_lower = start_y_ - relax_;
      y_upper = start_y_ + relax_;
    } else if (i + 1 == num_of_points_ && has_fixed_end_point_) {
      x_lower = end_x_ - relax_;
      x_upper = end_x_ + relax_;
      y_lower = end_y_ - relax_;
      y_upper = end_y_ + relax_;
    } else {
      x_lower = init_points_[i].x() - bound;
      x_upper = init_points_[i].x() + bound;
      y_lower = init_points_[i].y() - bound;
      y_upper = init_points_[i].y() + bound;
    }
    // x
    x_l[index] = x_lower;
    x_u[index] = x_upper;

    // y
    x_l[index + 1] = y_lower;
    x_u[index + 1] = y_upper;
  }

  // constraints
  // positional deviation constraints
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    double bound = std::min(lateral_bounds_[i], default_max_point_deviation_);
    g_l[i] = 0.0;
    g_u[i] = bound * bound;
  }

  // seems like the given constraint on end point is too tight, not good for
  // problem scaling

  g_u[0] = relax_ * relax_;
  g_u[num_of_points_ - 1] = relax_ * relax_;

  return true;
}

bool CosThetaProbleminterface::get_starting_point(int n, bool init_x, double* x,
                                                  bool init_z, double* z_L,
                                                  double* z_U, int m,
                                                  bool init_lambda,
                                                  double* lambda) {
  CHECK_EQ(static_cast<std::size_t>(n), num_of_variables_);
  std::random_device rd;
  std::default_random_engine gen = std::default_random_engine(rd());
  std::normal_distribution<> dis{0, 0.05};

  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 2;
    x[index] = init_points_[i].x() + dis(gen);
    x[index + 1] = init_points_[i].y() + dis(gen);
    // + dis(gen)
  }
  // for (std::size_t i = 0; i < num_of_points_; i++) {
  //   std::size_t index = i * 2;
  //   z_L[index] = 4.0e-8;
  //   z_L[index + 1] = 4.0e-8;
  // }
  // for (std::size_t i = 0; i < num_of_points_; i++) {
  //   std::size_t index = i * 2;
  //   z_U[index] = 4.0e-8;
  //   z_U[index + 1] = 4.0e-8;
  // }
  // for (std::size_t i = 0; i < num_of_points_; i++) {
  //   lambda[i] = 0.0;
  // }

  return true;
}

bool CosThetaProbleminterface::eval_f(int n, const double* x, bool new_x,
                                      double& obj_value) {
  CHECK_EQ(static_cast<std::size_t>(n), num_of_variables_);
  obj_value = 0.0;
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 2;
    obj_value +=
        (x[index] - init_points_[i].x()) * (x[index] - init_points_[i].x()) +
        (x[index + 1] - init_points_[i].y()) *
            (x[index + 1] - init_points_[i].y());
  }
  for (std::size_t i = 0; i < num_of_points_ - 2; i++) {
    std::size_t findex = i * 2;
    std::size_t mindex = findex + 2;
    std::size_t lindex = mindex + 2;
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
  CHECK_EQ(static_cast<std::size_t>(n), num_of_variables_);
  std::fill(grad_f, grad_f + n, 0.0);

  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 2;
    grad_f[index] = x[index] * 2 - init_points_[i].x() * 2;
    grad_f[index + 1] = x[index + 1] * 2 - init_points_[i].y() * 2;
  }

  for (std::size_t i = 0; i < num_of_points_ - 2; ++i) {
    std::size_t index = i * 2;
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
    grad_f[index] +=
        -weight_cos_included_angle_ *
        ((x[index + 2] - x[index + 4]) / (sqrt_q1 * sqrt_q2) -
         q3 / (2 * q1 * sqrt_q1 * sqrt_q2));
    grad_f[index + 1] +=
        -weight_cos_included_angle_ *
        ((x[index + 3] - x[index + 5]) / (sqrt_q1 * sqrt_q2) -
         q4 / (2 * q1 * sqrt_q1 * sqrt_q2));
    grad_f[index + 2] += -weight_cos_included_angle_ *
                         ((x[index] - 2 * x[index + 2] + x[index + 4]) /
                              (sqrt_q1 * sqrt_q2) +
                          q3 / (2 * q1 * sqrt_q1 * sqrt_q2) -
                          q5 / (2 * sqrt_q1 * q2 * sqrt_q2));
    grad_f[index + 3] += -weight_cos_included_angle_ *
                         ((x[index + 1] - 2 * x[index + 3] + x[index + 5]) /
                              (sqrt_q1 * sqrt_q2) +
                          q4 / (2 * q1 * sqrt_q1 * sqrt_q2) -
                          q6 / (2 * sqrt_q1 * q2 * sqrt_q2));
    grad_f[index + 4] +=
        -weight_cos_included_angle_ *
        (q5 / (2 * sqrt_q1 * q2 * sqrt_q2) -
         (x[index] - x[index + 2]) / (sqrt_q1 * sqrt_q2));
    grad_f[index + 5] +=
        -weight_cos_included_angle_ *
        (q6 / (2 * sqrt_q1 * q2 * sqrt_q2) -
         (x[index + 1] - x[index + 3]) / (sqrt_q1 * sqrt_q2));
  }
  return true;
}

bool CosThetaProbleminterface::eval_g(int n, const double* x, bool new_x, int m,
                                      double* g) {
  CHECK_EQ(static_cast<std::size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<std::size_t>(m), num_of_constraints_);
  // fill in the positional deviation constraints
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 2;
    g[i] = (x[index] - init_points_[i].x()) * (x[index] - init_points_[i].x()) +
           (x[index + 1] - init_points_[i].y()) *
               (x[index + 1] - init_points_[i].y());
  }

  return true;
}

bool CosThetaProbleminterface::eval_jac_g(int n, const double* x, bool new_x,
                                          int m, int nele_jac, int* iRow,
                                          int* jCol, double* values) {
  CHECK_EQ(static_cast<std::size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<std::size_t>(m), num_of_constraints_);
  if (values == NULL) {
    for (std::size_t i = 0; i < num_of_points_; ++i) {
      std::size_t index = i * 2;
      iRow[index] = i;
      jCol[index] = index;
      iRow[index + 1] = i;
      jCol[index + 1] = index + 1;
    }
  } else {
    std::fill(values, values + nnz_jac_g_, 0.0);
    for (std::size_t i = 0; i < num_of_points_; ++i) {
      std::size_t index = i * 2;
      values[index] = x[index] * 2 - init_points_[i].x() * 2;
      values[index + 1] = x[index + 1] * 2 - init_points_[i].y() * 2;
    }
  }

  return true;
}

bool CosThetaProbleminterface::eval_h(int n, const double* x, bool new_x,
                                      double obj_factor, int m,
                                      const double* lambda, bool new_lambda,
                                      int nele_hess, int* iRow, int* jCol,
                                      double* values) {
  if (values == NULL) {
    std::size_t index = 0;
    for (std::size_t i = 0; i < 6; ++i) {
      for (std::size_t j = 0; j <= i; ++j) {
        iRow[index] = i;
        jCol[index] = j;
        index++;
      }
    }

    std::size_t shift = 0;
    for (std::size_t i = 6; i < 2 * num_of_points_; ++i) {
      if (i % 2 == 0) {
        for (std::size_t j = 2 + shift; j <= 6 + shift; ++j) {
          iRow[index] = i;
          jCol[index] = j;
          index++;
        }

      } else {
        for (std::size_t j = 2 + shift; j <= 7 + shift; ++j) {
          iRow[index] = i;
          jCol[index] = j;
          index++;
        }
        shift += 2;
      }
    }
    CHECK_EQ(index, static_cast<std::size_t>(nele_hess));
  } else {
    std::fill(values, values + nele_hess, 0.0);
    // fill the included angle part of obj
    for (std::size_t i = 0; i < num_of_points_ - 2; ++i) {
      std::size_t topleft = i * 2;
      double q5 = (2 * x[topleft] - 2 * x[topleft + 2]);
      double q6 = (2 * x[topleft + 1] - 2 * x[topleft + 3]);
      double q7 = (x[topleft] - 2 * x[topleft + 2] + x[topleft + 4]);
      double q8 = (2 * x[topleft + 2] - 2 * x[topleft + 4]);
      double q9 = (2 * x[topleft + 3] - 2 * x[topleft + 5]);
      double q10 = (x[topleft + 1] - 2 * x[topleft + 3] + x[topleft + 5]);
      double q11 = (x[topleft + 3] - x[topleft + 5]);
      double q12 = (x[topleft] - x[topleft + 2]);
      double q13 = (x[topleft + 2] - x[topleft + 4]);
      double q14 = (x[topleft + 1] - x[topleft + 3]);
      double q1 = q12 * q12 + q14 * q14;
      double q2 = q13 * q13 + q11 * q11;
      double q3 = (3 * q5 * q5 * (q12 * q13 + q14 * q11));
      double q4 = (q12 * q13 + q14 * q11);
      double sqrt_q1 = std::sqrt(q1);
      double sqrt_q2 = std::sqrt(q2);
      values[idx_map_[std::make_pair(topleft, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (q3 / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) -
           q4 / (q1 * sqrt_q1 * sqrt_q2) -
           (q5 * q13) / (q1 * sqrt_q1 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 1, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q5 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) -
           (q6 * q13) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q5 * q11) / (2 * q1 * sqrt_q1 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 2, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / (sqrt_q1 * sqrt_q2) +
           q4 / (q1 * sqrt_q1 * sqrt_q2) -
           (q5 * q7) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           q3 / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) +
           (q5 * q13) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q8 * q13) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q5 * q8 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 3, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q13) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q5 * q10) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q9 * q13) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (3 * q5 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) +
           (q5 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q12) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           1 / (sqrt_q1 * sqrt_q2) +
           (q8 * q13) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q5 * q8 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q14) / (2 * q1 * sqrt_q1 * sqrt_q2) +
           (q9 * q13) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q5 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 1, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q6 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) -
           q4 / (q1 * sqrt_q1 * sqrt_q2) -
           (q6 * q11) / (q1 * sqrt_q1 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 2, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q11) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q6 * q7) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q8 * q11) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (3 * q5 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) +
           (q8 * q6 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 3, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / (sqrt_q1 * sqrt_q2) +
           q4 / (q1 * sqrt_q1 * sqrt_q2) -
           (q6 * q10) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (3 * q6 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) +
           (q6 * q11) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q9 * q11) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q6 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q12) / (2 * q1 * sqrt_q1 * sqrt_q2) +
           (q8 * q11) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q8 * q6 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 1)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q14) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           1 / (sqrt_q1 * sqrt_q2) +
           (q9 * q11) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q6 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 2, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q5 * q7) / (q1 * sqrt_q1 * sqrt_q2) -
           q4 / (sqrt_q1 * q2 * sqrt_q2) -
           q4 / (q1 * sqrt_q1 * sqrt_q2) -
           2 / (sqrt_q1 * sqrt_q2) -
           (q8 * q7) / (sqrt_q1 * q2 * sqrt_q2) +
           q3 / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) +
           (3 * q8 * q8 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           (q5 * q8 * q4) / (2 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 3, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q7) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q9 * q7) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q5 * q10) / (2 * q1 * sqrt_q1 * sqrt_q2) -
           (q8 * q10) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (3 * q5 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) -
           (q5 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2) -
           (q8 * q6 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2) +
           (3 * q8 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / (sqrt_q1 * sqrt_q2) +
           q4 / (sqrt_q1 * q2 * sqrt_q2) +
           (q8 * q7) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (3 * q8 * q8 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           (q5 * q12) / (2 * q1 * sqrt_q1 * sqrt_q2) +
           (q8 * q12) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q5 * q8 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 2)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q9 * q7) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q5 * q14) / (2 * q1 * sqrt_q1 * sqrt_q2) +
           (q8 * q14) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q5 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2) -
           (3 * q8 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 3, topleft + 3)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q6 * q10) / (q1 * sqrt_q1 * sqrt_q2) -
           q4 / (sqrt_q1 * q2 * sqrt_q2) -
           q4 / (q1 * sqrt_q1 * sqrt_q2) -
           2 / (sqrt_q1 * sqrt_q2) -
           (q9 * q10) / (sqrt_q1 * q2 * sqrt_q2) +
           (3 * q6 * q6 * q4) / (4 * q1 * q1 * sqrt_q1 * sqrt_q2) +
           (3 * q9 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           (q6 * q9 * q4) / (2 * q1 * sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 4, topleft + 3)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((q8 * q10) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q6 * q12) / (2 * q1 * sqrt_q1 * sqrt_q2) +
           (q9 * q12) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q8 * q6 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2) -
           (3 * q8 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 3)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          (1 / (sqrt_q1 * sqrt_q2) +
           q4 / (sqrt_q1 * q2 * sqrt_q2) +
           (q9 * q10) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (3 * q9 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           (q6 * q14) / (2 * q1 * sqrt_q1 * sqrt_q2) +
           (q9 * q14) / (2 * sqrt_q1 * q2 * sqrt_q2) +
           (q6 * q9 * q4) / (4 * q1 * sqrt_q1 * q2 * sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 4, topleft + 4)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q8 * q8 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           q4 / (sqrt_q1 * q2 * sqrt_q2) -
           (q8 * q12) / (sqrt_q1 * q2 * sqrt_q2));
      values[idx_map_[std::make_pair(topleft + 5, topleft + 4)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q8 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           (q9 * q12) / (2 * sqrt_q1 * q2 * sqrt_q2) -
           (q8 * q14) / (2 * sqrt_q1 * q2 * sqrt_q2));

      values[idx_map_[std::make_pair(topleft + 5, topleft + 5)]] +=
          obj_factor * (-weight_cos_included_angle_) *
          ((3 * q9 * q9 * q4) / (4 * sqrt_q1 * q2 * q2 * sqrt_q2) -
           q4 / (sqrt_q1 * q2 * sqrt_q2) -
           (q9 * q14) / (sqrt_q1 * q2 * sqrt_q2));
    }

    // fill the deviation part of obj
    for (std::size_t i = 0; i < num_of_points_ * 2; ++i) {
      values[idx_map_[std::make_pair(i, i)]] += obj_factor * 2;
    }
    // fill the constraints
    std::size_t lambda_idx = 0;
    for (std::size_t i = 0; i < num_of_points_; ++i) {
      std::size_t idx_constr = i * 2;
      std::pair<size_t, size_t> coors = std::make_pair(idx_constr, idx_constr);
      std::pair<size_t, size_t> coors1 =
          std::make_pair(idx_constr + 1, idx_constr + 1);
      values[idx_map_[coors]] += lambda[lambda_idx] * 2;
      values[idx_map_[coors1]] += lambda[lambda_idx] * 2;
      lambda_idx++;
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
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 2;
    opt_x_.emplace_back(x[index]);
    opt_y_.emplace_back(x[index + 1]);
  }
}

void CosThetaProbleminterface::set_default_max_point_deviation(
    const double max_point_deviation) {
  default_max_point_deviation_ = max_point_deviation;
}

void CosThetaProbleminterface::set_relax_end_constraint(const double relax) {
  relax_ = relax;
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
  std::size_t index = 0;
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      idx_map_.insert(
          std::pair<std::pair<std::size_t, std::size_t>, std::size_t>(
              std::pair<std::size_t, std::size_t>(i, j), index));
      index++;
    }
  }

  std::size_t shift = 0;
  for (std::size_t i = 6; i < 2 * num_of_points_; ++i) {
    if (i % 2 == 0) {
      for (std::size_t j = 2 + shift; j <= 6 + shift; ++j) {
        idx_map_.insert(
            std::pair<std::pair<std::size_t, std::size_t>, std::size_t>(
                std::pair<std::size_t, std::size_t>(i, j), index));
        index++;
      }

    } else {
      for (std::size_t j = 2 + shift; j <= 7 + shift; ++j) {
        idx_map_.insert(
            std::pair<std::pair<std::size_t, std::size_t>, std::size_t>(
                std::pair<std::size_t, std::size_t>(i, j), index));
        index++;
      }
      shift += 2;
    }
  }
}
}  // namespace planning
}  // namespace apollo
