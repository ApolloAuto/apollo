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

#pragma once

#include <unordered_map>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/integral.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

template <size_t N>
class QuinticSpiralPathWithDerivation : public QuinticPolynomialCurve1d {
 public:
  QuinticSpiralPathWithDerivation() = default;

  QuinticSpiralPathWithDerivation(const std::array<double, 3>& start,
                                  const std::array<double, 3>& end,
                                  const double delta_s);

  QuinticSpiralPathWithDerivation(const double theta0, const double kappa0,
                                  const double dkappa0, const double theta1,
                                  const double kappa1, const double dkappa1,
                                  const double delta_s);

  virtual ~QuinticSpiralPathWithDerivation() = default;

  double evaluate(const size_t order, const size_t i, const size_t n);

  double ComputeCartesianDeviationX() const { return dx_; }

  double ComputeCartesianDeviationY() const { return dy_; }

  std::pair<double, double> DeriveCartesianDeviation(const size_t param_index);

  double DeriveKappaDerivative(const size_t param_index, const int i,
                               const int n);

  double DeriveDKappaDerivative(const size_t param_index, const int i,
                                const int n);

  static const size_t THETA0 = 0;
  static const size_t KAPPA0 = 1;
  static const size_t DKAPPA0 = 2;
  static const size_t THETA1 = 3;
  static const size_t KAPPA1 = 4;
  static const size_t DKAPPA1 = 5;
  static const size_t DELTA_S = 6;

  static const size_t INDEX_MAX = 7;

  double ComputeCartesianDeviationX(const double s) const {
    auto cos_theta = [this](const double s) {
      const auto a = Evaluate(0, s);
      return std::cos(a);
    };
    return common::math::IntegrateByGaussLegendre<N>(cos_theta, 0.0, s);
  }

  double ComputeCartesianDeviationY(const double s) const {
    auto sin_theta = [this](const double s) {
      const auto a = Evaluate(0, s);
      return std::sin(a);
    };
    return common::math::IntegrateByGaussLegendre<N>(sin_theta, 0.0, s);
  }

 private:
  double DeriveCosTheta(const size_t param_index, const double r) const;

  double DeriveSinTheta(const size_t param_index, const double r) const;

  double DeriveThetaDerivative(const size_t param_index, const double r) const;

  std::array<std::array<double, 7>, 6> coef_deriv_;

  std::unordered_map<size_t, double> cache_evaluate_;

  std::unordered_map<size_t, std::pair<double, double>> cache_cartesian_deriv_;

  std::unordered_map<size_t, double> cache_kappa_deriv_;

  std::unordered_map<size_t, double> cache_dkappa_deriv_;

  double dx_ = 0.0;

  double dy_ = 0.0;

  std::array<double, N> gauss_points_;

  std::array<double, N> gauss_point_weights_;
};

template <size_t N>
QuinticSpiralPathWithDerivation<N>::QuinticSpiralPathWithDerivation(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double ddx1, const double s)
    : QuinticPolynomialCurve1d(x0, dx0, ddx0, x1, dx1, ddx1, s) {
  ACHECK(s > 0.0);

  auto gauss_points = common::math::GetGaussLegendrePoints<N>();
  gauss_points_ = gauss_points.first;
  gauss_point_weights_ = gauss_points.second;

  dx_ = ComputeCartesianDeviationX(s);

  dy_ = ComputeCartesianDeviationY(s);

  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s3 * s;
  double s5 = s2 * s3;
  double s6 = s3 * s3;

  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      coef_deriv_[i][j] = 0.0;
    }
  }
  // derive a
  // double a = -6.0 * x0 / p5 - 3.0 * dx0 / p4 - 0.5 * ddx0 / p3 + 6.0 * x1 /
  // p5 - 3.0 * dx1 / p4 + 0.5 * ddx1 / p3;
  coef_deriv_[5][0] = -6.0 / s5;
  coef_deriv_[5][1] = -3.0 / s4;
  coef_deriv_[5][2] = -0.5 / s3;
  coef_deriv_[5][3] = 6.0 / s5;
  coef_deriv_[5][4] = -3.0 / s4;
  coef_deriv_[5][5] = 0.5 / s3;
  coef_deriv_[5][6] = 30.0 * x0 / s6 + 12.0 * dx0 / s5 + 1.5 * ddx0 / s4 -
                      30.0 * x1 / s6 + 12.0 * dx1 / s5 - 1.5 * ddx1 / s4;

  // derive b
  // double b = 15.0 * x0 / p4 + 8.0 * dx0 / p3 + 1.5 * ddx0 / p2 - 15.0 * x1 /
  // p4 + 7.0 * dx1 / p3 - ddx1 / p2;
  coef_deriv_[4][0] = 15.0 / s4;
  coef_deriv_[4][1] = 8.0 / s3;
  coef_deriv_[4][2] = 1.5 / s2;
  coef_deriv_[4][3] = -15.0 / s4;
  coef_deriv_[4][4] = 7.0 / s3;
  coef_deriv_[4][5] = -1.0 / s2;
  coef_deriv_[4][6] = -60.0 * x0 / s5 - 24.0 * dx0 / s4 - 3.0 * ddx0 / s3 +
                      60.0 * x1 / s5 - 21.0 * dx1 / s4 + 2.0 * ddx1 / s3;

  // derive c
  // double c = -10.0 * x0 / p3 - 6.0 * dx0 / p2 - 1.5 * ddx0 / p + 10.0 * x1 /
  // p3 - 4.0 * dx1 / p2 + 0.5 * ddx1 / p;
  coef_deriv_[3][0] = -10.0 / s3;
  coef_deriv_[3][1] = -6.0 / s2;
  coef_deriv_[3][2] = -1.5 / s;
  coef_deriv_[3][3] = 10.0 / s3;
  coef_deriv_[3][4] = -4.0 / s2;
  coef_deriv_[3][5] = 0.5 / s;
  coef_deriv_[3][6] = 30.0 * x0 / s4 + 12.0 * dx0 / s3 + 1.5 * ddx0 / s2 -
                      30.0 * x1 / s4 + 8.0 * dx1 / s3 - 0.5 * ddx1 / s2;

  // derive d
  // double d = 0.5 * ddx0;
  coef_deriv_[2][2] = 0.5;

  // derive e
  // double e = dx0;
  coef_deriv_[1][1] = 1.0;

  // derive f
  // double f = x0;
  coef_deriv_[0][0] = 1.0;
}

template <size_t N>
QuinticSpiralPathWithDerivation<N>::QuinticSpiralPathWithDerivation(
    const std::array<double, 3>& start, const std::array<double, 3>& end,
    const double delta_s)
    : QuinticSpiralPathWithDerivation<N>(start[0], start[1], start[2], end[0],
                                         end[1], end[2], delta_s) {}

template <size_t N>
double QuinticSpiralPathWithDerivation<N>::evaluate(const size_t order,
                                                    const size_t i,
                                                    const size_t n) {
  auto key = order * 100 + n * 10 + i;
  if (cache_evaluate_.find(key) != cache_evaluate_.end()) {
    return cache_evaluate_[key];
  }
  auto res =
      Evaluate(order, static_cast<double>(i) / static_cast<double>(n) * param_);
  cache_evaluate_[key] = res;
  return res;
}

template <size_t N>
std::pair<double, double>
QuinticSpiralPathWithDerivation<N>::DeriveCartesianDeviation(
    const size_t param_index) {
  if (cache_cartesian_deriv_.find(param_index) !=
      cache_cartesian_deriv_.end()) {
    return cache_cartesian_deriv_[param_index];
  }

  const auto& g = gauss_points_;
  const auto& w = gauss_point_weights_;

  std::pair<double, double> cartesian_deviation = {0.0, 0.0};
  if (param_index != DELTA_S) {
    for (size_t i = 0; i < N; ++i) {
      double r = 0.5 * g[i] + 0.5;
      cartesian_deviation.first += w[i] * DeriveCosTheta(param_index, r);
      cartesian_deviation.second += w[i] * DeriveSinTheta(param_index, r);
    }

    cartesian_deviation.first *= param_ * 0.5;
    cartesian_deviation.second *= param_ * 0.5;
  } else {
    for (size_t i = 0; i < N; ++i) {
      double r = 0.5 * g[i] + 0.5;
      cartesian_deviation.first += w[i] * DeriveCosTheta(param_index, r);
      cartesian_deviation.second += w[i] * DeriveSinTheta(param_index, r);
    }

    cartesian_deviation.first *= param_ * 0.5;
    cartesian_deviation.second *= param_ * 0.5;

    for (size_t i = 0; i < N; ++i) {
      double r = 0.5 * g[i] + 0.5;
      auto theta = Evaluate(0, r * param_);

      cartesian_deviation.first += 0.5 * w[i] * std::cos(theta);
      cartesian_deviation.second += 0.5 * w[i] * std::sin(theta);
    }
  }
  cache_cartesian_deriv_[param_index] = cartesian_deviation;
  return cartesian_deviation;
}

template <size_t N>
double QuinticSpiralPathWithDerivation<N>::DeriveKappaDerivative(
    const size_t param_index, const int i, const int n) {
  auto key = param_index * INDEX_MAX + i;
  if (cache_kappa_deriv_.find(key) != cache_kappa_deriv_.end()) {
    return cache_kappa_deriv_[key];
  }

  auto r = static_cast<double>(i) / static_cast<double>(n);
  double s = param_ * r;
  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s2 * s2;

  double derivative = 5.0 * coef_deriv_[5][param_index] * s4 +
                      4.0 * coef_deriv_[4][param_index] * s3 +
                      3.0 * coef_deriv_[3][param_index] * s2 +
                      2.0 * coef_deriv_[2][param_index] * s +
                      coef_deriv_[1][param_index];

  if (param_index == DELTA_S) {
    derivative += 20.0 * coef_[5] * s3 * r + 12.0 * coef_[4] * s2 * r +
                  6.0 * coef_[3] * s * r + 2.0 * coef_[2] * r;
  }

  cache_kappa_deriv_[key] = derivative;
  return derivative;
}

template <size_t N>
double QuinticSpiralPathWithDerivation<N>::DeriveDKappaDerivative(
    const size_t param_index, const int i, const int n) {
  auto key = param_index * INDEX_MAX + i;
  if (cache_dkappa_deriv_.find(key) != cache_dkappa_deriv_.end()) {
    return cache_dkappa_deriv_[key];
  }

  auto r = static_cast<double>(i) / static_cast<double>(n);
  double s = param_ * r;
  double s2 = s * s;
  double s3 = s2 * s;

  double derivative = 20.0 * coef_deriv_[5][param_index] * s3 +
                      12.0 * coef_deriv_[4][param_index] * s2 +
                      6.0 * coef_deriv_[3][param_index] * s +
                      2.0 * coef_deriv_[2][param_index];

  if (param_index == DELTA_S) {
    derivative +=
        60.0 * coef_[5] * s2 * r + 24.0 * coef_[4] * s * r + 6.0 * coef_[3] * r;
  }

  cache_dkappa_deriv_[key] = derivative;
  return derivative;
}

template <size_t N>
double QuinticSpiralPathWithDerivation<N>::DeriveThetaDerivative(
    const size_t param_index, const double r) const {
  double s = param_ * r;
  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s2 * s2;
  double s5 = s3 * s2;

  double derivative =
      coef_deriv_[5][param_index] * s5 + coef_deriv_[4][param_index] * s4 +
      coef_deriv_[3][param_index] * s3 + coef_deriv_[2][param_index] * s2 +
      coef_deriv_[1][param_index] * s + coef_deriv_[0][param_index];

  if (param_index == DELTA_S) {
    derivative += coef_[5] * 5.0 * s4 * r + coef_[4] * 4.0 * s3 * r +
                  coef_[3] * 3.0 * s2 * r + coef_[2] * 2.0 * s * r +
                  coef_[1] * r;
  }
  return derivative;
}

template <size_t N>
double QuinticSpiralPathWithDerivation<N>::DeriveCosTheta(
    const size_t param_index, const double r) const {
  double g = param_ * r;
  double theta = Evaluate(0, g);

  double derivative = -std::sin(theta) * DeriveThetaDerivative(param_index, r);
  return derivative;
}

template <size_t N>
double QuinticSpiralPathWithDerivation<N>::DeriveSinTheta(
    const size_t param_index, const double r) const {
  double g = param_ * r;
  double theta = Evaluate(0, g);

  double derivative = std::cos(theta) * DeriveThetaDerivative(param_index, r);
  return derivative;
}

}  // namespace planning
}  // namespace apollo
