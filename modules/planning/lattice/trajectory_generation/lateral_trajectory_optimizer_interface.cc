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

/**
 * @file
 **/

#include "modules/planning/lattice/trajectory_generation/lateral_trajectory_optimizer_interface.h"

#include "glog/logging.h"
#include "modules/planning/lattice/trajectory1d/constant_jerk_trajectory1d.h"

namespace apollo {
namespace planning {

LateralTrajectoryOptimizerInterface::LateralTrajectoryOptimizerInterface(
    const std::size_t num_of_points) {
  // TODO Auto-generated constructor stub

}

LateralTrajectoryOptimizerInterface::~LateralTrajectoryOptimizerInterface() {
  // TODO Auto-generated destructor stub
}

bool LateralTrajectoryOptimizerInterface::get_nlp_info(int& n, int& m,
    int& nnz_jac_g, int& nnz_h_lag, IndexStyleEnum& index_style) {
  // variables
  n = num_of_points_ * 3;

  // constraints
  m = num_of_points_ * 3;

  // TODO:
  nnz_h_lag = n;


  index_style = IndexStyleEnum::C_STYLE;

  return true;
}

bool LateralTrajectoryOptimizerInterface::get_bounds_info(int n, double* x_l,
    double* x_u, int m, double* g_l, double* g_u) {

  const double LARGE_VALUE = 10.0;

  // bounds for variables
  // d bounds;
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    x_l[i] = d_bounds_[i].first;
    x_u[i] = d_bounds_[i].second;
  }

  // d_prime bounds
  std::size_t offset = num_of_points_;
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    x_l[offset + i] = -LARGE_VALUE;
    x_u[offset + i] = LARGE_VALUE;
  }

  // d_pprime bounds;
  offset = 2 * num_of_points_;
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    x_l[offset + i] = -LARGE_VALUE;
    x_u[offset + i] = LARGE_VALUE;
  }

  // bounds for constraints

  // jerk bounds
//  auto jerk_max_sqr = d_ppprime_max_ * delta_s_;
//  jerk_max_sqr = jerk_max_sqr * jerk_max_sqr;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    g_l[i] = -d_ppprime_max_ * delta_s_;
    g_u[i] = d_ppprime_max_ * delta_s_;
  }

  // velocity increment
  offset = num_of_points_ - 1;
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = g_u[offset + i] = 0.0;
  }

  // position increment
  offset = 2 * (num_of_points_ - 1);
  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = g_u[offset + i] = 0.0;
  }

  offset = 3 * (num_of_points_ - 1);
  // d_init
  g_l[offset] = g_u[offset] = 0.0;

  // d_prime_init
  g_l[offset + 1] = g_u[offset + 1] = 0.0;

  // d_pprime_init
  g_l[offset + 2] = g_u[offset + 2] = 0.0;

  return true;
}

bool LateralTrajectoryOptimizerInterface::get_starting_point(int n, bool init_x,
    double* x, bool init_z, double* z_L, double* z_U, int m, bool init_lambda,
    double* lambda) {

  CHECK_EQ(std::size_t(n), num_of_points_ * 3);
  CHECK(init_x == true);
  CHECK(init_z == false);
  CHECK(init_lambda == false);

  for (std::size_t i = 0; i < num_of_points_; ++i) {
    x[i] = 0.0;
    x[num_of_points_ + i] = 0.0;
    x[num_of_points_ + num_of_points_ + i] = 0.0;
  }

  x[0] = d_init_;
  x[num_of_points_] = d_prime_init_;
  x[num_of_points_ + num_of_points_] = d_pprime_init_;
  return false;
}

bool LateralTrajectoryOptimizerInterface::eval_f(int n, const double* x,
    bool new_x, double& obj_value) {
  obj_value = 0.0;

  std::size_t offset_prime = num_of_points_;
  std::size_t offset_pprime = 2 * num_of_points_;
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    obj_value += x[i] * x[i] * w_d_;
    obj_value += x[offset_prime + i] * x[offset_prime + i] * w_d_prime_;
    obj_value += x[offset_pprime + i] * x[offset_pprime + i] * w_d_pprime_;

    auto dist = x[i] - (d_bounds_[i].first + d_bounds_[i].second) * 0.5;
    obj_value += dist * dist * w_d_obs_;
  }

  return true;
}

bool LateralTrajectoryOptimizerInterface::eval_grad_f(int n, const double* x,
    bool new_x, double* grad_f) {

  std::fill(grad_f, grad_f + n, 0.0);

  std::size_t offset_prime = num_of_points_;
  std::size_t offset_pprime = 2 * num_of_points_;
  for (std::size_t i = 0; i < num_of_points_; ++i) {
    auto obs_center = (d_bounds_[i].first + d_bounds_[i].second) * 0.5;
    grad_f[i] = 2.0 * x[i] * w_d_ + 2.0 * (x[i] - obs_center) * w_d_obs_;

    grad_f[offset_prime + i] = 2.0 * x[offset_prime + i] * w_d_prime_;

    grad_f[offset_pprime + i] = 2.0 * x[offset_pprime + i] * w_d_pprime_;
  }

  return true;
}

bool LateralTrajectoryOptimizerInterface::eval_g(int n, const double* x,
    bool new_x, int m, double* g) {

  std::size_t offset_prime = num_of_points_;
  std::size_t offset_pprime = 2 * num_of_points_;

  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    g[i] = x[offset_pprime + i + 1] - x[offset_pprime + i];
  }

  for (std::size_t i = 0; i + 1 < num_of_points_; ++i) {
    double p0 = x[i];
    double v0 = x[offset_prime + i];
    double a0 = x[offset_pprime + i];

    double p1 = x[i + 1];
    double v1 = x[offset_prime + i + 1];
    double a1 = x[offset_pprime + i + 1];

    double j = (a1 - a0) / delta_s_;
    ConstantJerkTrajectory1d t(p0, v0, a0, j, delta_s_);

    g[num_of_points_ - 1 + i] = t.GetEndVelocity() - v1;
    g[2 * (num_of_points_ - 1) + i] = t.GetEndState() - p1;
  }

  std::size_t offset = 3 * (num_of_points_ - 1);
  g[offset] = x[0] - d_init_;
  g[offset + 1] = x[offset_prime] - d_prime_init_;
  g[offset + 2] = x[offset_pprime] - d_pprime_init_;
  return true;
}

bool LateralTrajectoryOptimizerInterface::eval_jac_g(int n, const double* x,
    bool new_x, int m, int nele_jac, int* iRow, int* jCol, double* values) {

  CHECK_EQ(std::size_t(n), num_of_points_ * 3);
  CHECK_EQ(std::size_t(m), num_of_points_ * 3);

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

bool LateralTrajectoryOptimizerInterface::eval_h(int n, const double* x,
    bool new_x, double obj_factor, int m, const double* lambda, bool new_lambda,
    int nele_hess, int* iRow, int* jCol, double* values) {
  if (values == nullptr) {
    for (std::size_t i = 0; i < 3 * num_of_points_; ++i) {
      iRow[i] = i;
      jCol[i] = i;
    }
  } else {
    for (std::size_t i = 0; i < num_of_points_; ++i) {
      values[i] = 4.0;
    }

    for (std::size_t i = num_of_points_; i < 3 * num_of_points_; ++i) {
      values[i] = 2.0;
    }
  }
  return true;
}

void LateralTrajectoryOptimizerInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
}

} // namespace planning
} // namespace apollo
