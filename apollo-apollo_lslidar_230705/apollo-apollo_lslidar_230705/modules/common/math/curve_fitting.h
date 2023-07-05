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
 */

#pragma once

#include <array>

#include "Eigen/Dense"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace common {
namespace math {

// The coef is in ascending order,
// i.e., f(x) = coef[0] + coef[1] * x + coef[2] * x^2 ...
template <std::size_t N>
double EvaluatePolynomial(const std::array<double, N + 1>& coef,
                          const double p) {
  double r = 0.0;
  for (int i = N; i >= 0; --i) {
    r = r * p + coef[i];
  }
  return r;
}

// Fit a Nth order polynomial using M 2d points.
template <std::size_t N, std::size_t M>
std::array<double, N + 1> FitPolynomial(
    const std::array<Eigen::Vector2d, M>& points,
    double* ptr_error_square = nullptr) {
  Eigen::Matrix<double, M, N + 1> X;
  Eigen::Matrix<double, M, 1> Y;
  for (std::size_t i = 0; i < M; ++i) {
    double x = points[i].x();
    double y = points[i].y();

    X(i, 0) = 1.0;
    for (std::size_t j = 1; j < N + 1; ++j) {
      X(i, j) = X(i, j - 1) * x;
    }

    Y(i, 0) = y;
  }

  Eigen::Matrix<double, N + 1, 1> t =
      PseudoInverse<double, N + 1, N + 1>(X.transpose() * X) * X.transpose() *
      Y;

  std::array<double, N + 1> coefs;
  for (std::size_t i = 0; i < N + 1; ++i) {
    coefs[i] = t(i, 0);
  }

  if (ptr_error_square != nullptr) {
    *ptr_error_square = 0.0;
    for (const Eigen::Vector2d& point : points) {
      double error = EvaluatePolynomial(coefs, point.x()) - point.y();
      *ptr_error_square += error * error;
    }
  }
  return coefs;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
