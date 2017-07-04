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

#include "modules/common/math/math_utils.h"

#include <utility>

#include "glog/logging.h"

namespace apollo {
namespace common {
namespace math {

double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  const double new_angle = std::fmod(angle + M_PI, M_PI * 2.0);
  return (new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle) - M_PI;
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

int double_compare(const double d1, const double d2, const double epsilon) {
  DCHECK(!std::isnan(d1));
  DCHECK(!std::isnan(d2));

  if ((d1 - d2) > std::fmax(std::fabs(d1), std::fabs(d2)) * epsilon) {
    return 1;
  } else if ((d2 - d1) > std::fmax(std::fabs(d1), std::fabs(d2)) * epsilon) {
    return -1;
  } else {
    return 0;
  }
}

}  // namespace math
}  // namespace common
}  // namespace apollo
