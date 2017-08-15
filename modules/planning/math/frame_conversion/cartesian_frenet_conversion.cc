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
 * @file cartesian_frenet_conversion.cc
 **/

#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"

#include <cmath>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

void CartesianFrenetConverter::cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa, const double x, const double y,
    const double v, const double a, const double theta, const double kappa,
    std::array<double, 3>* const ptr_s_condition,
    std::array<double, 3>* const ptr_d_condition) {
  double dx = x - rx;
  double dy = y - ry;

  double cos_theta_r = std::cos(rtheta);
  double sin_theta_r = std::sin(rtheta);

  double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  (*ptr_d_condition)[0] =
      std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

  double delta_theta = theta - rtheta;
  double tan_delta_theta = std::tan(delta_theta);
  double cos_delta_theta = std::cos(delta_theta);

  double one_minus_kappa_r_d = 1 - rkappa * (*ptr_d_condition)[0];
  (*ptr_d_condition)[1] = one_minus_kappa_r_d * tan_delta_theta;

  double kappa_r_d_prime =
      rdkappa * (*ptr_d_condition)[0] + rkappa * (*ptr_d_condition)[1];

  (*ptr_d_condition)[2] =
      -kappa_r_d_prime * tan_delta_theta +
      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
          (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

  (*ptr_s_condition)[0] = rs;

  (*ptr_s_condition)[1] = v * cos_delta_theta / one_minus_kappa_r_d;

  double delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
  (*ptr_s_condition)[2] =
      (a * cos_delta_theta -
       (*ptr_s_condition)[1] * (*ptr_s_condition)[1] *
           ((*ptr_d_condition)[1] * delta_theta_prime - kappa_r_d_prime)) /
      one_minus_kappa_r_d;
  return;
}

void CartesianFrenetConverter::frenet_to_cartesian(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa,
    const std::array<double, 3>& s_condition,
    const std::array<double, 3>& d_condition, double* const ptr_x,
    double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
    double* const ptr_v, double* const ptr_a) {

  CHECK(std::abs(rs - s_condition[0]) < 1.0e-6
      && "The reference point s and s_condition[0] don't match");

  double cos_theta_r = std::cos(rtheta);
  double sin_theta_r = std::sin(rtheta);

  *ptr_x = rx - sin_theta_r * d_condition[0];
  *ptr_y = ry + cos_theta_r * d_condition[0];

  double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

  double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
  double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
  double cos_delta_theta = std::cos(delta_theta);

  *ptr_theta = common::math::NormalizeAngle(delta_theta + rtheta);

  double kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1];
  *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                 cos_delta_theta * cos_delta_theta) /
                    (one_minus_kappa_r_d) +
                rkappa) *
               cos_delta_theta / (one_minus_kappa_r_d);

  double d_dot = d_condition[1] * s_condition[1];
  *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                         s_condition[1] * s_condition[1] +
                     d_dot * d_dot);

  double delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

  *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
           s_condition[1] * s_condition[1] / cos_delta_theta *
               (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
  return;
}

}  // namespace planning
}  // namespace apollo
