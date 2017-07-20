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
 * @file: integration.cc
 **/

#include "modules/planning/math/integration.h"

#include <array>

#include "glog/logging.h"

namespace apollo {
namespace planning {

double Integration::trapezoidal(const std::vector<double>& v, const double dx,
                                const std::size_t nsteps) {
  double sum = 0;
  for (std::size_t i = 1; i + 1 < nsteps; ++i) {
    sum += v[i];
  }
  return dx * sum + 0.5 * dx * (v[0] + v[nsteps - 1]);
}

double Integration::simpson(const std::vector<double>& v, const double dx,
                            const std::size_t nsteps) {
  CHECK_EQ(1, nsteps & 1);
  double sum1 = 0.0;
  double sum2 = 0.0;
  for (std::size_t i = 1; i + 1 < nsteps; ++i) {
    if ((i & 1) != 0) {
      sum1 += v[i];
    } else {
      sum2 += v[i];
    }
  }
  return dx / 3.0 * (4.0 * sum1 + 2.0 * sum2 + v[0] + v[nsteps - 1]);
}

double Integration::gauss_legendre(const std::function<double(double)>& func,
                                   const double a, const double b) {
  double t = (b - a) * 0.5;
  double m = (b + a) * 0.5;

  std::array<double, 5> w;
  w[0] = 0.5688888889;
  w[1] = 0.4786286705;
  w[2] = 0.4786286705;
  w[3] = 0.2369268851;
  w[4] = 0.2369268851;

  std::array<double, 5> x;
  x[0] = 0.0;
  x[1] = 0.5384693101;
  x[2] = -0.5384693101;
  x[3] = 0.9061798459;
  x[4] = -0.9061798459;

  double integral = 0.0;
  for (size_t i = 0; i < 5; ++i) {
    integral += w[i] * func(t * x[i] + m);
  }

  return integral * t;
}

}  // namespace planning
}  // namespace apollo
