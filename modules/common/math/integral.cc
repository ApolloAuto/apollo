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

#include "modules/common/math/integral.h"

namespace apollo {
namespace common {
namespace math {

double GaussLegendre(const std::function<double(double)>& func,
                     const double lower_bound,
                     const double upper_bound) {
  double t = (upper_bound - lower_bound) * 0.5;
  double m = (upper_bound + lower_bound) * 0.5;

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

}  // namespace math
}  // namespace common
}  // namespace apollo
