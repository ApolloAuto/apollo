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
 * @file: interpolation.h
 **/

#ifndef MODULES_PLANNING_MATH_INTERPOLATION_H_
#define MODULES_PLANNING_MATH_INTERPOLATION_H_

#include <array>
#include <cmath>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/math/hermite_spline.h"

namespace apollo {
namespace planning {

class Interpolation {
 public:
  Interpolation() = delete;

  static double slerp(const double a0, const double p0, const double a1,
                      const double p1, const double p);

  template <typename T>
  static T lerp(const T& x0, const double p0, const T& x1, const double p1,
                const double p);

  template <typename T, std::size_t N>
  static std::array<T, N> hermite(const std::array<T, N>& x0, const double p0,
                                  const std::array<T, N>& x1, const double p1,
                                  const double p);
};

template <typename T>
inline T Interpolation::lerp(const T& x0, const double p0, const T& x1,
                             const double p1, const double p) {
  CHECK(p0 < p1);
  double r = (p - p0) / (p1 - p0);
  return x0 + (x1 - x0) * r;
}

inline double Interpolation::slerp(const double a0, const double p0,
                                   const double a1, const double p1,
                                   const double p) {
  double a0_n = common::math::NormalizeAngle(a0);
  double a1_n = common::math::NormalizeAngle(a1);

  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2.0 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2.0 * M_PI;
  }

  double r = (p - p0) / (p1 - p0);
  double a = a0_n + d * r;
  return a;
}

template <typename T, std::size_t N>
inline std::array<T, N> Interpolation::hermite(const std::array<T, N>& x0,
                                               const double p0,
                                               const std::array<T, N>& x1,
                                               const double p1,
                                               const double p) {
  CHECK(N == 2 || N == 3)
      << "Error: currently hermite interpolation only supports cubic and "
         "quintic!";

  HermiteSpline<T, 2 * N - 1> hermite_spline(x0, x1, p0, p1);
  std::array<T, N> x;
  for (std::size_t i = 0; i < N; ++i) {
    x[i] = hermite_spline.evaluate(i, p);
  }
  return x;
}

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_MATH_INTERPOLATION_H_ */
