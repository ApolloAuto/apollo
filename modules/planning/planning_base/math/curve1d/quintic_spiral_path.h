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
 * @file quintic_spiral_path.h
 **/

#pragma once

#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/integral.h"
#include "modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

/**
 * Describe a quintic spiral path
 * Map (theta0, kappa0, dkappa0) ----- delta_s -----> (theta1, kappa1, dkappa1)
 */
class QuinticSpiralPath : public QuinticPolynomialCurve1d {
 public:
  QuinticSpiralPath() = default;

  QuinticSpiralPath(const std::array<double, 3>& start,
                    const std::array<double, 3>& end, const double delta_s);

  QuinticSpiralPath(const double theta0, const double kappa0,
                    const double dkappa0, const double theta1,
                    const double kappa1, const double dkappa1,
                    const double delta_s);

  template <size_t N>
  double ComputeCartesianDeviationX(const double s) const {
    auto cos_theta = [this](const double s) {
      const auto a = Evaluate(0, s);
      return std::cos(a);
    };
    return common::math::IntegrateByGaussLegendre<N>(cos_theta, 0.0, s);
  }

  template <size_t N>
  double ComputeCartesianDeviationY(const double s) const {
    auto sin_theta = [this](const double s) {
      const auto a = Evaluate(0, s);
      return std::sin(a);
    };
    return common::math::IntegrateByGaussLegendre<N>(sin_theta, 0.0, s);
  }

  template <size_t N>
  std::pair<double, double> DeriveCartesianDeviation(
      const size_t param_index) const {
    auto gauss_points = common::math::GetGaussLegendrePoints<N>();
    std::array<double, N> x = gauss_points.first;
    std::array<double, N> w = gauss_points.second;

    std::pair<double, double> cartesian_deviation = {0.0, 0.0};
    for (size_t i = 0; i < N; ++i) {
      double r = 0.5 * x[i] + 0.5;
      auto curr_theta = Evaluate(0, r * param_);
      double derived_theta = DeriveTheta(param_index, r);

      cartesian_deviation.first +=
          w[i] * (-std::sin(curr_theta)) * derived_theta;
      cartesian_deviation.second += w[i] * std::cos(curr_theta) * derived_theta;
    }

    cartesian_deviation.first *= param_ * 0.5;
    cartesian_deviation.second *= param_ * 0.5;

    if (param_index == DELTA_S) {
      for (size_t i = 0; i < N; ++i) {
        double r = 0.5 * x[i] + 0.5;
        auto theta_angle = Evaluate(0, r * param_);

        cartesian_deviation.first += 0.5 * w[i] * std::cos(theta_angle);
        cartesian_deviation.second += 0.5 * w[i] * std::sin(theta_angle);
      }
    }
    return cartesian_deviation;
  }

  double DeriveKappaDerivative(const size_t param_index,
                               const double ratio) const;

  double DeriveDKappaDerivative(const size_t param_index,
                                const double ratio) const;

  double DeriveD2KappaDerivative(const size_t param_index,
                                 const double r) const;

  static const size_t THETA0 = 0;
  static const size_t KAPPA0 = 1;
  static const size_t DKAPPA0 = 2;
  static const size_t THETA1 = 3;
  static const size_t KAPPA1 = 4;
  static const size_t DKAPPA1 = 5;
  static const size_t DELTA_S = 6;

 private:
  double DeriveTheta(const size_t param_index,
                     const double delta_s_ratio) const;

  std::array<std::array<double, 7>, 6> coef_deriv_;
};

}  // namespace planning
}  // namespace apollo
