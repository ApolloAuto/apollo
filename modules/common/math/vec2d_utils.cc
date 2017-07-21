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

#include "modules/common/math/vec2d_utils.h"

#include <cmath>

#include "modules/common/log.h"

namespace apollo {

using common::Vec2D;

constexpr double kMathEpsilon = 1e-10;

//! Sums two Vec2D
Vec2D operator+(const Vec2D &v0, const Vec2D &v1) {
  return common::math::Vec2DCtor(v0.x() + v1.x(), v0.y() + v1.y());
}

//! Subtracts two Vec2D
Vec2D operator-(const Vec2D &v0, const Vec2D &v1) {
  return common::math::Vec2DCtor(v0.x() - v1.x(), v0.y() - v1.y());
}

//! Multiplies Vec2D by a scalar
Vec2D operator*(const Vec2D &v, const double ratio) {
  return common::math::Vec2DCtor(v.x() * ratio, v.y() * ratio);
}

Vec2D operator*(const double ratio, const Vec2D &v) {
  return v * ratio;
}

//! Divides Vec2D by a scalar
Vec2D operator/(const Vec2D &v, const double ratio) {
  CHECK_GT(std::abs(ratio), kMathEpsilon) << "Devides by 0!";
  return common::math::Vec2DCtor(v.x() / ratio, v.y() / ratio);
}

//! Sums another Vec2D to the current one
Vec2D &operator+=(Vec2D &v0, const Vec2D &v1) {
  v0.set_x(v0.x() + v1.x());
  v0.set_y(v0.y() + v1.y());
  return v0;
}

//! Subtracts another Vec2D to the current one
Vec2D &operator-=(Vec2D &v0, const Vec2D &v1) {
  v0.set_x(v0.x() - v1.x());
  v0.set_y(v0.y() - v1.y());
  return v0;
}

//! Multiplies this Vec2D by a scalar
Vec2D &operator*=(Vec2D &v, const double ratio) {
  v.set_x(v.x() * ratio);
  v.set_y(v.y() * ratio);
  return v;
}

//! Divides this Vec2D by a scalar
Vec2D &operator/=(Vec2D &v, const double ratio) {
  CHECK_GT(std::abs(ratio), kMathEpsilon) << "Devides by 0!";
  v.set_x(v.x() / ratio);
  v.set_y(v.y() / ratio);
  return v;
}

//! Compares two Vec2D
bool operator==(const Vec2D &v0, const Vec2D &v1) {
  return (std::abs(v0.x() - v1.x()) < kMathEpsilon &&
          std::abs(v0.y() - v1.y()) < kMathEpsilon);
}

namespace common {
namespace math {

Vec2D Vec2DCtor(const double x, const double y) {
  Vec2D ret;
  ret.set_x(x);
  ret.set_y(y);
  return ret;
}

//! Creates a unit-vector with a given angle to the positive x semi-axis
Vec2D Vec2DUnit(const double angle) {
  return Vec2DCtor(cos(angle), sin(angle));
}

//! Gets the length of the vector
double VecLength(const Vec2D &v) {
  return std::hypot(v.x(), v.y());
}

//! Gets the squared length of the vector
double VecLengthSquare(const Vec2D &v) {
  return v.x() * v.x() + v.y() * v.y();
}

//! Gets the angle between the vector and the positive x semi-axis
double VecAngle(const Vec2D &v) {
  return std::atan2(v.y(), v.x());
}

//! Returns the unit vector that is co-linear with this vector
void VecNormalize(Vec2D *v) {
  const double len = VecLength(*v);
  if (len > kMathEpsilon) {
    *v /= len;
  } else {
    AERROR << "Vector length is too small: " << v->DebugString();
  }
}

Vec2D VecNormalized(const Vec2D &v) {
  const double len = VecLength(v);
  if (len > kMathEpsilon) {
    return v / len;
  }
  AERROR << "Vector length is too small: " << v.DebugString();
  return v;
}

//! Returns the distance to the given vector
double VecDistance(const Vec2D &v0, const Vec2D &v1) {
  return std::hypot(v0.x() - v1.x(), v0.y() - v1.y());
}

//! Returns the squared distance to the given vector
double VecDistanceSquare(const Vec2D &v0, const Vec2D &v1) {
  const double dx = v0.x() - v1.x();
  const double dy = v0.y() - v1.y();
  return dx * dx + dy * dy;
}

//! Returns the "cross" product between these two Vec2D (non-standard).
double VecCrossProd(const Vec2D &v0, const Vec2D &v1) {
  return v0.x() * v1.y() - v0.y() * v1.x();
}

//! Returns the inner product between these two Vec2D.
double VecInnerProd(const Vec2D &v0, const Vec2D &v1) {
  return v0.x() * v1.x() + v0.y() * v1.y();
}

}  // namespace math
}  // namespace common
}  // namespace apollo
