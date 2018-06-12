/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/*
 * spiral_problem_interface.cc
 */

#include "modules/planning/reference_line/spiral_problem_interface.h"

#include <math.h>
#include <utility>

#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

constexpr std::size_t N = 10;

SpiralProblemInterface::SpiralProblemInterface(
    std::vector<Eigen::Vector2d> points)
    : init_points_(std::move(points)) {
  num_of_points_ = init_points_.size();
  CHECK_GT(num_of_points_, 1);

  point_distances_.reserve(num_of_points_ - 1);
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    point_distances_.push_back((init_points_[i + 1] - init_points_[i]).norm());
  }

  std::vector<double> normalized_theta;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    Eigen::Vector2d v = init_points_[i + 1] - init_points_[i];
    double theta = std::atan2(v.y(), v.x());
    normalized_theta.push_back(theta);
  }
  normalized_theta.push_back(normalized_theta.back());

  relative_theta_.push_back(normalized_theta.front());
  for (std::size_t i = 1; i < num_of_points_; ++i) {
    double theta_diff =
        common::math::AngleDiff(relative_theta_.back(), normalized_theta[i]);
    relative_theta_.push_back(relative_theta_.back() + theta_diff);
  }

  piecewise_paths_.resize(num_of_points_ - 1);
}

void SpiralProblemInterface::get_optimization_results(
    std::vector<double>* ptr_theta, std::vector<double>* ptr_kappa,
    std::vector<double>* ptr_dkappa, std::vector<double>* ptr_s,
    std::vector<double>* ptr_x, std::vector<double>* ptr_y) const {
  *ptr_theta = opt_theta_;
  *ptr_kappa = opt_kappa_;
  *ptr_dkappa = opt_dkappa_;
  *ptr_s = opt_s_;
  *ptr_x = opt_x_;
  *ptr_y = opt_y_;
}

bool SpiralProblemInterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                          int& nnz_h_lag,
                                          IndexStyleEnum& index_style) {
  // number of variables
  n = num_of_points_ * 5 + num_of_points_ - 1;
  num_of_variables_ = n;

  // number of constraints
  // b. positional equality constraints;
  // totally 2 * (num_of_points - 1) considering x and y separately
  m = (num_of_points_ - 1) * 2;
  // a. positional movements; totally num_of_points
  m += num_of_points_;
  num_of_constraints_ = m;

  // number of nonzero constraint jacobian.
  nnz_jac_g = (num_of_points_ - 1) * 2 * 9 + num_of_points_ * 2;
  nnz_jac_g_ = nnz_jac_g;

  // number of nonzero hessian and lagrangian.
  nnz_h_lag = 22 * (num_of_points_ - 1);
  nnz_h_lag_ = nnz_h_lag;

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool SpiralProblemInterface::get_bounds_info(int n, double* x_l, double* x_u,
                                             int m, double* g_l, double* g_u) {
  CHECK_EQ(std::size_t(n), num_of_variables_);
  CHECK_EQ(std::size_t(m), num_of_constraints_);

  // variables
  // a. for theta, kappa, dkappa, x, y
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 5;

    double theta_lower = 0.0;
    double theta_upper = 0.0;
    double kappa_lower = 0.0;
    double kappa_upper = 0.0;
    double dkappa_lower = 0.0;
    double dkappa_upper = 0.0;
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;
    if (i == 0 && has_fixed_start_point_) {
      theta_lower = start_theta_;
      theta_upper = start_theta_;
      kappa_lower = start_kappa_;
      kappa_upper = start_kappa_;
      dkappa_lower = start_dkappa_;
      dkappa_upper = start_dkappa_;
      x_lower = start_x_;
      x_upper = start_x_;
      y_lower = start_y_;
      y_upper = start_y_;

    } else if (i + 1 == num_of_points_ && has_fixed_end_point_) {
      theta_lower = end_theta_;
      theta_upper = end_theta_;
      kappa_lower = end_kappa_;
      kappa_upper = end_kappa_;
      dkappa_lower = end_dkappa_;
      dkappa_upper = end_dkappa_;
      x_lower = end_x_;
      x_upper = end_x_;
      y_lower = end_y_;
      y_upper = end_y_;
    } else if (i + 1 == num_of_points_ && has_fixed_end_point_position_) {
      theta_lower = relative_theta_[i] - M_PI * 0.2;
      theta_upper = relative_theta_[i] + M_PI * 0.2;
      kappa_lower = -0.25;
      kappa_upper = 0.25;
      dkappa_lower = -0.02;
      dkappa_upper = 0.02;
      x_lower = end_x_;
      x_upper = end_x_;
      y_lower = end_y_;
      y_upper = end_y_;
    } else {
      theta_lower = relative_theta_[i] - M_PI * 0.2;
      theta_upper = relative_theta_[i] + M_PI * 0.2;
      kappa_lower = -0.25;
      kappa_upper = 0.25;
      dkappa_lower = -0.02;
      dkappa_upper = 0.02;
      x_lower = init_points_[i].x() - default_max_point_deviation_;
      x_upper = init_points_[i].x() + default_max_point_deviation_;
      y_lower = init_points_[i].y() - default_max_point_deviation_;
      y_upper = init_points_[i].y() + default_max_point_deviation_;
    }

    // theta
    x_l[index] = theta_lower;
    x_u[index] = theta_upper;

    // kappa
    x_l[index + 1] = kappa_lower;
    x_u[index + 1] = kappa_upper;

    // dkappa
    x_l[index + 2] = dkappa_lower;
    x_u[index + 2] = dkappa_upper;

    // x
    x_l[index + 3] = x_lower;
    x_u[index + 3] = x_upper;

    // y
    x_l[index + 4] = y_lower;
    x_u[index + 4] = y_upper;
  }

  // b. for delta_s
  std::size_t variable_offset = num_of_points_ * 5;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    x_l[variable_offset + i] =
        point_distances_[i] - 2.0 * default_max_point_deviation_;
    x_u[variable_offset + i] = point_distances_[i] * M_PI * 0.5;
  }

  // constraints
  // a. positional equality constraints
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    // for x
    g_l[i * 2] = 0.0;
    g_u[i * 2] = 0.0;

    // for y
    g_l[i * 2 + 1] = 0.0;
    g_u[i * 2 + 1] = 0.0;
  }
  // b. positional deviation constraints
  std::size_t constraint_offset = 2 * (num_of_points_ - 1);
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    g_l[constraint_offset + i] = 0.0;
    g_u[constraint_offset + i] =
        default_max_point_deviation_ * default_max_point_deviation_;
  }
  return true;
}

bool SpiralProblemInterface::get_starting_point(int n, bool init_x, double* x,
                                                bool init_z, double* z_L,
                                                double* z_U, int m,
                                                bool init_lambda,
                                                double* lambda) {
  CHECK_EQ(std::size_t(n), num_of_variables_);
  CHECK(init_x == true);
  CHECK(init_z == false);
  CHECK(init_lambda == false);

  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 5;
    x[index] = relative_theta_[i];
    x[index + 1] = 0.0;
    x[index + 2] = 0.0;
    x[index + 3] = init_points_[i].x();
    x[index + 4] = init_points_[i].y();
  }

  std::size_t variable_offset = num_of_points_ * 5;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    double delta_theta = relative_theta_[i + 1] - relative_theta_[i];
    x[variable_offset + i] = point_distances_[i] / std::cos(0.5 * delta_theta);
  }

  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    double delta_theta = relative_theta_[i + 1] - relative_theta_[i];
    x[(i + 1) * 5 + 1] = delta_theta / x[variable_offset + i];
  }
  x[1] = x[6];

  if (has_fixed_start_point_) {
    x[0] = start_theta_;
    x[1] = start_kappa_;
    x[2] = start_dkappa_;
  }
  return true;
}

bool SpiralProblemInterface::eval_f(int n, const double* x, bool new_x,
                                    double& obj_value) {
  CHECK_EQ(std::size_t(n), num_of_variables_);
  if (new_x) {
    update_piecewise_spiral_paths(x, n);
  }

  obj_value = 0.0;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    const QuinticSpiralPath& spiral_curve = piecewise_paths_[i];
    double delta_s = spiral_curve.ParamLength();

    for (std::size_t j = 0; j < num_of_internal_points_; ++j) {
      double ratio = static_cast<double>(j) / num_of_internal_points_;
      double s = ratio * delta_s;
      double kappa = spiral_curve.Evaluate(1, s);
      obj_value += kappa * kappa * weight_kappa_;

      double dkappa = spiral_curve.Evaluate(2, s);
      obj_value += dkappa * dkappa * weight_dkappa_;

      double d2kappa = spiral_curve.Evaluate(3, s);
      obj_value += d2kappa * d2kappa * weight_d2kappa_;
    }
  }
  return true;
}

bool SpiralProblemInterface::eval_grad_f(int n, const double* x, bool new_x,
                                         double* grad_f) {
  CHECK_EQ(std::size_t(n), num_of_variables_);
  std::fill(grad_f, grad_f + n, 0.0);

  if (new_x) {
    update_piecewise_spiral_paths(x, n);
  }

  std::size_t variable_offset = num_of_points_ * 5;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    std::size_t index0 = i * 5;
    std::size_t index1 = (i + 1) * 5;

    const QuinticSpiralPath& spiral_curve = piecewise_paths_[i];
    double delta_s = spiral_curve.ParamLength();

    for (std::size_t j = 0; j < num_of_internal_points_; ++j) {
      double ratio = static_cast<double>(j) / num_of_internal_points_;

      double kappa = spiral_curve.Evaluate(1, ratio * delta_s);
      grad_f[index0] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::THETA0, ratio);
      grad_f[index0 + 1] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::KAPPA0, ratio);
      grad_f[index0 + 2] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::DKAPPA0, ratio);

      grad_f[index1] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::THETA1, ratio);
      grad_f[index1 + 1] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::KAPPA1, ratio);
      grad_f[index1 + 2] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::DKAPPA1, ratio);

      grad_f[variable_offset + i] +=
          weight_kappa_ * 2.0 * kappa *
          spiral_curve.DeriveKappaDerivative(QuinticSpiralPath::DELTA_S, ratio);

      double dkappa = spiral_curve.Evaluate(2, ratio * delta_s);
      grad_f[index0] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(QuinticSpiralPath::THETA0, ratio);
      grad_f[index0 + 1] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(QuinticSpiralPath::KAPPA0, ratio);
      grad_f[index0 + 2] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(
              QuinticSpiralPath::DKAPPA0, ratio);

      grad_f[index1] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(QuinticSpiralPath::THETA1, ratio);
      grad_f[index1 + 1] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(QuinticSpiralPath::KAPPA1, ratio);
      grad_f[index1 + 2] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(
              QuinticSpiralPath::DKAPPA1, ratio);

      grad_f[variable_offset + i] +=
          weight_dkappa_ * 2.0 * dkappa *
          spiral_curve.DeriveDKappaDerivative(
              QuinticSpiralPath::DELTA_S, ratio);

      double d2kappa = spiral_curve.Evaluate(3, ratio * delta_s);
      grad_f[index0] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::THETA0, ratio);
      grad_f[index0 + 1] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::KAPPA0, ratio);
      grad_f[index0 + 2] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::DKAPPA0, ratio);

      grad_f[index1] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::THETA1, ratio);
      grad_f[index1 + 1] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::KAPPA1, ratio);
      grad_f[index1 + 2] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::DKAPPA1, ratio);

      grad_f[variable_offset + i] +=
          weight_d2kappa_ * 2.0 * d2kappa *
          spiral_curve.DeriveD2KappaDerivative(
              QuinticSpiralPath::DELTA_S, ratio);
    }
  }
  return true;
}

bool SpiralProblemInterface::eval_g(int n, const double* x, bool new_x, int m,
                                    double* g) {
  CHECK_EQ(std::size_t(n), num_of_variables_);
  CHECK_EQ(std::size_t(m), num_of_constraints_);

  if (new_x) {
    update_piecewise_spiral_paths(x, n);
  }

  // first, fill in the positional equality constraints
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    std::size_t index0 = i * 5;
    std::size_t index1 = (i + 1) * 5;

    const QuinticSpiralPath& spiral_curve = piecewise_paths_[i];
    double delta_s = spiral_curve.ParamLength();

    double x_diff = x[index1 + 3] - x[index0 + 3] -
                    spiral_curve.ComputeCartesianDeviationX<N>(delta_s);
    g[i * 2] = x_diff * x_diff;

    double y_diff = x[index1 + 4] - x[index0 + 4] -
                    spiral_curve.ComputeCartesianDeviationY<N>(delta_s);
    g[i * 2 + 1] = y_diff * y_diff;
  }

  // second, fill in the positional deviation constraints
  std::size_t constraint_offset = 2 * (num_of_points_ - 1);
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t variable_index = i * 5;
    double x_cor = x[variable_index + 3];
    double y_cor = x[variable_index + 4];

    double x_diff = x_cor - init_points_[i].x();
    double y_diff = y_cor - init_points_[i].y();

    g[constraint_offset + i] = x_diff * x_diff + y_diff * y_diff;
  }
  return true;
}

bool SpiralProblemInterface::eval_jac_g(int n, const double* x, bool new_x,
                                        int m, int nele_jac, int* iRow,
                                        int* jCol, double* values) {
  CHECK_EQ(std::size_t(n), num_of_variables_);
  CHECK_EQ(std::size_t(m), num_of_constraints_);

  if (values == NULL) {
    std::size_t nz_index = 0;

    std::size_t variable_offset = num_of_points_ * 5;
    for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
      std::size_t variable_index = i * 5;

      // theta0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 0;
      ++nz_index;

      // kappa0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 1;
      ++nz_index;

      // dkappa0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 2;
      ++nz_index;

      // x0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 3;
      ++nz_index;

      // theta1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 5;
      ++nz_index;

      // kappa1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 6;
      ++nz_index;

      // dkappa1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 7;
      ++nz_index;

      // x1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 8;
      ++nz_index;

      // s
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_offset + i;
      ++nz_index;

      // theta0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 0;
      ++nz_index;

      // kappa0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 1;
      ++nz_index;

      // dkappa0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 2;
      ++nz_index;

      // y0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 4;
      ++nz_index;

      // theta1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 5;
      ++nz_index;

      // kappa1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 6;
      ++nz_index;

      // dkappa1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 7;
      ++nz_index;

      // y1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 9;
      ++nz_index;

      // s
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_offset + i;
      ++nz_index;
    }

    std::size_t constraint_offset = 2 * (num_of_points_ - 1);
    for (std::size_t i = 0; i < num_of_points_; ++i) {
      iRow[nz_index] = constraint_offset + i;
      jCol[nz_index] = i * 5 + 3;
      ++nz_index;

      iRow[nz_index] = constraint_offset + i;
      jCol[nz_index] = i * 5 + 4;
      ++nz_index;
    }

    CHECK_EQ(nz_index, nnz_jac_g_);
  } else {
    if (new_x) {
      update_piecewise_spiral_paths(x, n);
    }

    std::fill(values, values + nnz_jac_g_, 0.0);
    // first, positional equality constraints
    std::size_t nz_index = 0;

    for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
      std::size_t index0 = i * 5;
      std::size_t index1 = (i + 1) * 5;

      const QuinticSpiralPath& spiral_curve = piecewise_paths_[i];
      double delta_s = spiral_curve.ParamLength();

      double x_diff = x[index1 + 3] - x[index0 + 3] -
                      spiral_curve.ComputeCartesianDeviationX<N>(delta_s);
      double y_diff = x[index1 + 4] - x[index0 + 4] -
                      spiral_curve.ComputeCartesianDeviationY<N>(delta_s);

      auto pos_theta0 =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::THETA0);
      auto pos_kappa0 =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::KAPPA0);
      auto pos_dkappa0 =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::DKAPPA0);

      auto pos_theta1 =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::THETA1);
      auto pos_kappa1 =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::KAPPA1);
      auto pos_dkappa1 =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::DKAPPA1);

      auto pos_delta_s =
          spiral_curve.DeriveCartesianDeviation<N>(QuinticSpiralPath::DELTA_S);

      // for x coordinate
      // theta0
      values[nz_index] += 2.0 * x_diff * (-pos_theta0.first);
      ++nz_index;

      // kappa0
      values[nz_index] += 2.0 * x_diff * (-pos_kappa0.first);
      ++nz_index;

      // dkappa0
      values[nz_index] += 2.0 * x_diff * (-pos_dkappa0.first);
      ++nz_index;

      // x0
      values[nz_index] += 2.0 * x_diff * (-1.0);
      ++nz_index;

      // theta1
      values[nz_index] += 2.0 * x_diff * (-pos_theta1.first);
      ++nz_index;

      // kappa1
      values[nz_index] += 2.0 * x_diff * (-pos_kappa1.first);
      ++nz_index;

      // dkappa1
      values[nz_index] += 2.0 * x_diff * (-pos_dkappa1.first);
      ++nz_index;

      // x1
      values[nz_index] += 2.0 * x_diff;
      ++nz_index;

      // delta_s
      values[nz_index] += 2.0 * x_diff * (-pos_delta_s.first);
      ++nz_index;

      // for y coordinate
      // theta0
      values[nz_index] += 2.0 * y_diff * (-pos_theta0.second);
      ++nz_index;

      // kappa0
      values[nz_index] += 2.0 * y_diff * (-pos_kappa0.second);
      ++nz_index;

      // dkappa0
      values[nz_index] += 2.0 * y_diff * (-pos_dkappa0.second);
      ++nz_index;

      // y0
      values[nz_index] += 2.0 * y_diff * (-1.0);
      ++nz_index;

      // theta1
      values[nz_index] += 2.0 * y_diff * (-pos_theta1.second);
      ++nz_index;

      // kappa1
      values[nz_index] += 2.0 * y_diff * (-pos_kappa1.second);
      ++nz_index;

      // dkappa1
      values[nz_index] += 2.0 * y_diff * (-pos_dkappa1.second);
      ++nz_index;

      // y1
      values[nz_index] += 2.0 * y_diff;
      ++nz_index;

      // delta_s
      values[nz_index] += 2.0 * y_diff * (-pos_delta_s.second);
      ++nz_index;
    }

    for (std::size_t i = 0; i < num_of_points_; ++i) {
      values[nz_index] = 2.0 * (x[i * 5 + 3] - init_points_[i].x());
      ++nz_index;

      values[nz_index] = 2.0 * (x[i * 5 + 4] - init_points_[i].y());
      ++nz_index;
    }

    CHECK_EQ(nz_index, nnz_jac_g_);
  }
  return true;
}

bool SpiralProblemInterface::eval_h(int n, const double* x, bool new_x,
                                    double obj_factor, int m,
                                    const double* lambda, bool new_lambda,
                                    int nele_hess, int* iRow, int* jCol,
                                    double* values) {
  if (values == NULL) {
    std::size_t index = 0;
    for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
      std::size_t variable_index = i * 5;

      iRow[index] = variable_index;
      jCol[index] = variable_index;
      ++index;

      iRow[index] = variable_index;
      jCol[index] = variable_index + 1;
      ++index;

      iRow[index] = variable_index;
      jCol[index] = variable_index + 2;
      ++index;

      iRow[index] = variable_index;
      jCol[index] = variable_index + 3;
      ++index;

      iRow[index] = variable_index;
      jCol[index] = variable_index + 5;
      ++index;

      iRow[index] = variable_index;
      jCol[index] = variable_index + 6;
      ++index;

      iRow[index] = variable_index;
      jCol[index] = variable_index + 7;
      ++index;

      iRow[index] = variable_index + 1;
      jCol[index] = variable_index + 1;
      ++index;

      iRow[index] = variable_index + 1;
      jCol[index] = variable_index + 2;
      ++index;

      iRow[index] = variable_index + 1;
      jCol[index] = variable_index + 5;
      ++index;

      iRow[index] = variable_index + 1;
      jCol[index] = variable_index + 6;
      ++index;

      iRow[index] = variable_index + 1;
      jCol[index] = variable_index + 7;
      ++index;

      iRow[index] = variable_index + 2;
      jCol[index] = variable_index + 2;
      ++index;

      iRow[index] = variable_index + 2;
      jCol[index] = variable_index + 5;
      ++index;

      iRow[index] = variable_index + 2;
      jCol[index] = variable_index + 6;
      ++index;

      iRow[index] = variable_index + 2;
      jCol[index] = variable_index + 7;
      ++index;
    }

    std::size_t variable_offset = 5 * num_of_points_;
    for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
      iRow[index] = i * 5;
      jCol[index] = i + variable_offset;
      ++index;

      iRow[index] = i * 5 + 1;
      jCol[index] = i + variable_offset;
      ++index;

      iRow[index] = i * 5 + 2;
      jCol[index] = i + variable_offset;
      ++index;

      iRow[index] = i * 5 + 5;
      jCol[index] = i + variable_offset;
      ++index;

      iRow[index] = i * 5 + 6;
      jCol[index] = i + variable_offset;
      ++index;

      iRow[index] = i * 5 + 7;
      jCol[index] = i + variable_offset;
      ++index;

      iRow[index] = i + variable_offset;
      jCol[index] = i + variable_offset;
      ++index;
    }

    CHECK_EQ(index, nnz_h_lag_);
  } else {
    CHECK(false);
  }
  return true;
}

void SpiralProblemInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  opt_theta_.reserve(num_of_points_);
  opt_kappa_.reserve(num_of_points_);
  opt_dkappa_.reserve(num_of_points_);
  opt_x_.reserve(num_of_points_);
  opt_y_.reserve(num_of_points_);
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    std::size_t index = i * 5;
    opt_theta_.push_back(x[index]);
    opt_kappa_.push_back(x[index + 1]);
    opt_dkappa_.push_back(x[index + 2]);
    opt_x_.push_back(x[index + 3]);
    opt_y_.push_back(x[index + 4]);
  }

  opt_s_.reserve(num_of_points_ - 1);
  std::size_t variable_offset = num_of_points_ * 5;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    opt_s_.push_back(x[variable_offset + i]);
  }
}

void SpiralProblemInterface::update_piecewise_spiral_paths(const double* x,
                                                           const int n) {
  std::size_t variable_offset = num_of_points_ * 5;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    std::size_t index0 = i * 5;
    std::size_t index1 = (i + 1) * 5;

    std::array<double, 3> x0 = {x[index0], x[index0 + 1], x[index0 + 2]};
    std::array<double, 3> x1 = {x[index1], x[index1 + 1], x[index1 + 2]};
    double delta_s = x[variable_offset + i];
    piecewise_paths_[i] = std::move(QuinticSpiralPath(x0, x1, delta_s));
  }
}

void SpiralProblemInterface::set_default_max_point_deviation(
    const double max_point_deviation) {
  default_max_point_deviation_ = max_point_deviation;
}

void SpiralProblemInterface::set_start_point(const double x, const double y,
                                             const double theta,
                                             const double kappa,
                                             const double dkappa) {
  has_fixed_start_point_ = true;
  start_x_ = x;
  start_y_ = y;
  start_theta_ = theta;
  start_kappa_ = kappa;
  start_dkappa_ = dkappa;
}

void SpiralProblemInterface::set_end_point(const double x, const double y,
                                           const double theta,
                                           const double kappa,
                                           const double dkappa) {
  has_fixed_end_point_ = true;
  end_x_ = x;
  end_y_ = y;
  end_theta_ = theta;
  end_kappa_ = kappa;
  end_dkappa_ = dkappa;
}

void SpiralProblemInterface::set_end_point_position(const double x,
                                                    const double y) {
  has_fixed_end_point_position_ = true;
  end_x_ = x;
  end_y_ = y;
}

void SpiralProblemInterface::set_element_weight_curve_length(
    const double weight_curve_length) {
  weight_curve_length_ = weight_curve_length;
}

void SpiralProblemInterface::set_element_weight_kappa(
    const double weight_kappa) {
  weight_kappa_ = weight_kappa;
}

void SpiralProblemInterface::set_element_weight_dkappa(
    const double weight_dkappa) {
  weight_dkappa_ = weight_dkappa;
}

void SpiralProblemInterface::set_element_weight_d2kappa(
    const double weight_d2kappa) {
  weight_d2kappa_ = weight_d2kappa;
}

}  // namespace planning
}  // namespace apollo
