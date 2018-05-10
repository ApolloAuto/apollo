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

/**
 * @file  : quintic_spiral_curve.cc
 **/
#include "modules/planning/math/spiral_curve/quintic_spiral_curve.h"

#include <algorithm>
#include "Eigen/Core"
#include "Eigen/LU"

#include "modules/common/log.h"
#include "modules/common/math/integral.h"
#include "modules/planning/math/spiral_curve/spiral_formula.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;

QuinticSpiralCurve::QuinticSpiralCurve(const common::PathPoint& s,
                                       const common::PathPoint& e)
    : SpiralCurve(s, e, 5) {
  // generate an order 5 spiral path with four parameters
}

bool QuinticSpiralCurve::CalculatePath() {
  // extract infos
  double x_s = start_point().x();
  double y_s = start_point().y();
  double theta_s = std::fmod(start_point().theta(), s_two_pi_);

  if (theta_s < 0) {
    theta_s += s_two_pi_;
  }

  double x_t = end_point().x() - x_s;  // with position
  double y_t = end_point().y() - y_s;  // with position

  double x_g = std::cos(theta_s) * x_t + std::sin(theta_s) * y_t;
  double y_g = -std::sin(theta_s) * x_t + std::cos(theta_s) * y_t;
  double theta_g = std::fmod(end_point().theta(), s_two_pi_);
  theta_g -= theta_s;

  while (theta_g < -M_PI) {
    theta_g += s_two_pi_;
  }

  while (theta_g > +M_PI) {
    theta_g -= s_two_pi_;
  }

  double sg = (theta_g * theta_g / 5 + 1.0) * std::sqrt(x_g * x_g + y_g * y_g);

  std::array<double, 6> p_shoot;
  p_shoot[0] = start_point().kappa();
  p_shoot[1] = start_point().dkappa();
  p_shoot[2] = start_point().ddkappa();
  p_shoot[3] = 0.0;
  p_shoot[4] = 0.0;
  p_shoot[5] = end_point().kappa();

  // intermediate params
  Eigen::Matrix<double, 3, 1> q_g;
  q_g << x_g, y_g, theta_g;            // goal, x(p, sg), y(p, sg), theta(p, sg)
  Eigen::Matrix<double, 3, 3> jacobi;  // Jacobian matrix for newton method

  // simpson integrations func values in Jacobian
  // integration point initialization:
  double ds =
      sg / (spiral_config().simpson_size() - 1);  // bandwith for integration
  // basic theta value vectors:
  std::vector<double> theta(spiral_config().simpson_size(), 0.0);
  std::vector<double> cos_theta(spiral_config().simpson_size(), 0.0);
  std::vector<double> sin_theta(spiral_config().simpson_size(), 0.0);
  // partial derivatvies vectors for Jacobian
  std::vector<double> ptp_p3(spiral_config().simpson_size(), 0.0);
  std::vector<double> ptp_p4(spiral_config().simpson_size(), 0.0);
  std::vector<double> ptp_sg(spiral_config().simpson_size(), 0.0);
  std::vector<double> sin_ptp_p3(spiral_config().simpson_size(), 0.0);
  std::vector<double> sin_ptp_p4(spiral_config().simpson_size(), 0.0);
  std::vector<double> sin_ptp_sg(spiral_config().simpson_size(), 0.0);
  std::vector<double> cos_ptp_p3(spiral_config().simpson_size(), 0.0);
  std::vector<double> cos_ptp_p4(spiral_config().simpson_size(), 0.0);
  std::vector<double> cos_ptp_sg(spiral_config().simpson_size(), 0.0);
  // newton iteration difference (col) vectors
  Eigen::Matrix<double, 3, 1> delta_q;  // goal difference
  Eigen::Matrix<double, 3, 1> delta_p;  // parameter difference
  Eigen::Matrix<double, 3, 1>
      q_guess;      // q with current paramter, delta_q = q_g - q_guess
  double diff = 0;  // absolute error for q iteration stop

  for (int32_t nt = 0; nt < spiral_config().newton_raphson_max_iter(); ++nt) {
    // calculate parameters for simpson integration
    double s = 0;

    for (int32_t i = 0; i < spiral_config().simpson_size(); ++i) {
      theta[i] = SpiralFormula::theta_func_k5(s, sg, p_shoot);

      cos_theta[i] = std::cos(theta[i]);
      sin_theta[i] = std::sin(theta[i]);

      ptp_p3[i] = SpiralFormula::partial_theta_p3_k5(s, sg);
      ptp_p4[i] = SpiralFormula::partial_theta_p4_k5(s, sg);
      ptp_sg[i] = SpiralFormula::partial_theta_sg_k5(s, sg, p_shoot);

      sin_ptp_p3[i] = sin_theta[i] * ptp_p3[i];
      sin_ptp_p4[i] = sin_theta[i] * ptp_p4[i];
      sin_ptp_sg[i] = sin_theta[i] * ptp_sg[i];

      cos_ptp_p3[i] = cos_theta[i] * ptp_p3[i];
      cos_ptp_p4[i] = cos_theta[i] * ptp_p4[i];
      cos_ptp_sg[i] = cos_theta[i] * ptp_sg[i];
      s += ds;
    }

    // update Jacobian and delta q
    jacobi(0, 0) = -apollo::common::math::IntegrateBySimpson(
        sin_ptp_p3, ds, spiral_config().simpson_size());

    jacobi(0, 1) = -apollo::common::math::IntegrateBySimpson(
        sin_ptp_p4, ds, spiral_config().simpson_size());

    jacobi(0, 2) = cos_theta[spiral_config().simpson_size() - 1] -
                   apollo::common::math::IntegrateBySimpson(
                       sin_ptp_sg, ds, spiral_config().simpson_size());

    jacobi(1, 0) = apollo::common::math::IntegrateBySimpson(
        cos_ptp_p3, ds, spiral_config().simpson_size());

    jacobi(1, 1) = apollo::common::math::IntegrateBySimpson(
        cos_ptp_p4, ds, spiral_config().simpson_size());

    jacobi(1, 2) = sin_theta[spiral_config().simpson_size() - 1] +
                   apollo::common::math::IntegrateBySimpson(
                       cos_ptp_sg, ds, spiral_config().simpson_size());

    jacobi(2, 0) = ptp_p3[spiral_config().simpson_size() - 1];

    jacobi(2, 1) = ptp_p4[spiral_config().simpson_size() - 1];

    jacobi(2, 2) = ptp_sg[spiral_config().simpson_size() - 1];

    q_guess(0) = apollo::common::math::IntegrateBySimpson(
        cos_theta, ds, spiral_config().simpson_size());

    q_guess(1) = apollo::common::math::IntegrateBySimpson(
        sin_theta, ds, spiral_config().simpson_size());

    q_guess(2) = theta[spiral_config().simpson_size() - 1];

    delta_q = q_g - q_guess;

    // early stop
    diff =
        std::fabs(delta_q(0)) + std::fabs(delta_q(1)) + std::fabs(delta_q(2));

    if (diff < spiral_config().newton_raphson_tol()) {
      break;
    }

    delta_p = jacobi.lu().solve(delta_q);
    // update p, sg, ds
    p_shoot[3] += delta_p(0);
    p_shoot[4] += delta_p(1);
    sg += delta_p(2);
    ds = sg / (spiral_config().simpson_size() - 1);
  }

  // update params
  PrependToPParams(p_shoot.begin(), p_shoot.end());
  set_sg(sg);
  set_error(diff);

  return diff < spiral_config().newton_raphson_tol() && ResultSanityCheck();
}

Status QuinticSpiralCurve::GetPathVec(
    const std::uint32_t n, std::vector<common::PathPoint>* path_points) const {
  CHECK_NOTNULL(path_points);

  if (n <= 1 || error() > spiral_config().newton_raphson_tol()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "QuinticSpiralCurve::get_path_vec");
  }

  path_points->resize(n);
  std::vector<common::PathPoint>& result = *path_points;

  result[0].set_x(start_point().x());
  result[0].set_y(start_point().y());
  result[0].set_theta(start_point().theta());
  result[0].set_kappa(start_point().kappa());
  result[0].set_dkappa(start_point().dkappa());
  result[0].set_ddkappa(start_point().ddkappa());

  const double ds = sg() / (n - 1);
  double s = ds;

  // calculate theta kappa along the path
  std::array<double, 6> p_value;
  std::copy(p_params().begin(), p_params().end(), p_value.begin());
  std::array<double, 6> a_params = SpiralFormula::p_to_a_k5(sg(), p_value);

  for (std::uint32_t i = 1; i < n; ++i) {
    result[i].set_s(s);
    result[i].set_theta(SpiralFormula::theta_func_k5_a(s, a_params) +
                        result[0].theta());
    result[i].set_kappa(SpiralFormula::kappa_func_k5_a(s, a_params));
    result[i].set_dkappa(SpiralFormula::dkappa_func_k5_a(s, a_params));
    s += ds;
  }

  // integration x, y along the path
  double dx = 0.0;
  double dy = 0.0;

  for (std::uint32_t k = 1; k < n; ++k) {
    dx = (dx / k) * (k - 1) +
         (std::cos(std::fmod(result[k].theta(), s_two_pi_)) +
          std::cos(std::fmod(result[k - 1].theta(), s_two_pi_))) /
             (2 * k);
    dy = (dy / k) * (k - 1) +
         (std::sin(std::fmod(result[k].theta(), s_two_pi_)) +
          std::sin(std::fmod(result[k - 1].theta(), s_two_pi_))) /
             (2 * k);
    result[k].set_x(result[k].s() * dx + result[0].x());
    result[k].set_y(result[k].s() * dy + result[0].y());
  }

  return Status::OK();
}

Status QuinticSpiralCurve::GetPathVecWithS(
    const std::vector<double>& vec_s,
    std::vector<common::PathPoint>* path_points) const {
  CHECK_NOTNULL(path_points);

  if (error() > spiral_config().newton_raphson_tol()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "QuinticSpiralCurve::get_path_vec_with_s");
  }

  if (vec_s.size() == 0) {
    return Status::OK();
  }

  const std::uint32_t n = vec_s.size();

  common::PathPoint ref_point = start_point();

  path_points->resize(n);
  std::vector<common::PathPoint>& result = *path_points;
  std::array<double, 6> p_value;
  std::copy(p_params().begin(), p_params().end(), p_value.begin());
  std::array<double, 6> a_params = SpiralFormula::p_to_a_k5(sg(), p_value);

  for (std::uint32_t i = 0; i < n; ++i) {
    result[i].set_s(vec_s[i]);
    result[i].set_theta(SpiralFormula::theta_func_k5_a(vec_s[i], a_params) +
                        ref_point.theta());
    result[i].set_kappa(SpiralFormula::kappa_func_k5_a(vec_s[i], a_params));
    result[i].set_dkappa(SpiralFormula::dkappa_func_k5_a(vec_s[i], a_params));
  }

  double dx = 0.0;
  double dy = 0.0;

  // start from here, add k = 0, revise the result[k] = ...
  dx += (vec_s[0] - ref_point.s()) *
        (std::cos(std::fmod(ref_point.theta(), s_two_pi_)) +
         std::cos(std::fmod(result[0].theta(), s_two_pi_))) /
        2.0;
  dy += (vec_s[0] - ref_point.s()) *
        (std::cos(std::fmod(ref_point.theta(), s_two_pi_)) +
         std::cos(std::fmod(result[0].theta(), s_two_pi_))) /
        2.0;
  result[0].set_x(dx + ref_point.x());
  result[0].set_y(dy + ref_point.y());
  for (std::uint32_t k = 1; k < n; ++k) {
    dx += (vec_s[k] - vec_s[k - 1]) *
          (std::cos(std::fmod(result[k - 1].theta(), s_two_pi_)) +
           std::cos(std::fmod(result[k].theta(), s_two_pi_))) /
          2.0;
    dy += (vec_s[k] - vec_s[k - 1]) *
          (std::sin(std::fmod(result[k - 1].theta(), s_two_pi_)) +
           std::sin(std::fmod(result[k].theta(), s_two_pi_))) /
          2.0;
    result[k].set_x(dx + ref_point.x());
    result[k].set_y(dy + ref_point.y());
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
