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

#include "modules/common/math/linear_interpolation.h"

#include <cmath>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace common {
namespace math {

double lerp(const double x0, const double t0, const double x1, const double t1,
            const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    AERROR << "input time difference is too small";
    return x0;
  }
  double r = (t - t0) / (t1 - t0);
  double x = x0 + r * (x1 - x0);
  return x;
}

void lerp(const double x0, const double y0, const double t0, const double x1,
          const double y1, const double t1, const double t, double* x,
          double* y) {
  *x = lerp(x0, t0, x1, t1, t);
  *y = lerp(y0, t0, y1, t1, t);
}

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    AERROR << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  double a0_n = NormalizeAngle(a0);
  double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  double r = (t - t0) / (t1 - t0);
  double a = a0_n + d * r;
  return NormalizeAngle(a);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
