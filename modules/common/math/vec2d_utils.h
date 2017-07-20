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

#ifndef MODULES_COMMON_MATH_VEC2D_UTILS_H_
#define MODULES_COMMON_MATH_VEC2D_UTILS_H_

#include <cmath>
#include <string>

#include "modules/common/proto/vector.pb.h"

namespace apollo {
// Put operators in ::apollo namespace to make it project wide accessible.

//! Sums two Vec2D
common::Vec2D operator+(const common::Vec2D &v0, const common::Vec2D &v1);

//! Subtracts two Vec2D
common::Vec2D operator-(const common::Vec2D &v0, const common::Vec2D &v1);

//! Multiplies Vec2D by a scalar
common::Vec2D operator*(const common::Vec2D &v, const double ratio);
common::Vec2D operator*(const double ratio, const common::Vec2D &v);

//! Divides Vec2D by a scalar
common::Vec2D operator/(const common::Vec2D &v, const double ratio);

//! Sums another Vec2D to the current one
common::Vec2D &operator+=(common::Vec2D &v0, const common::Vec2D &v1);

//! Subtracts another Vec2D to the current one
common::Vec2D &operator-=(common::Vec2D &v0, const common::Vec2D &v1);

//! Multiplies this Vec2D by a scalar
common::Vec2D &operator*=(common::Vec2D &v, const double ratio);

//! Divides this Vec2D by a scalar
common::Vec2D &operator/=(common::Vec2D &v, const double ratio);

//! Compares two Vec2D
bool operator==(const common::Vec2D &v0, const common::Vec2D &v1);

namespace common {
namespace math {

constexpr double kMathEpsilon = 1e-10;

//! Creates a unit-vector with a given angle to the positive x semi-axis
Vec2D Vec2DUnit(const double angle);

//! Gets the length of the vector
double VecLength(const Vec2D &v);

//! Gets the squared length of the vector
double VecLengthSquare(const Vec2D &v);

//! Gets the angle between the vector and the positive x semi-axis
double VecAngle(const Vec2D &v);

//! Returns the unit vector that is co-linear with this vector
void VecNormalize(Vec2D *v);
Vec2D VecNormalized(const Vec2D &v);

//! Returns the distance to the given vector
double VecDistance(const Vec2D &v0, const Vec2D &v1);

//! Returns the squared distance to the given vector
double VecDistanceSquare(const Vec2D &v0, const Vec2D &v1);

//! Returns the "cross" product between these two Vec2D (non-standard).
double VecCrossProd(const Vec2D &v0, const Vec2D &v1);

//! Returns the inner product between these two Vec2D.
double VecInnerProd(const Vec2D &v0, const Vec2D &v1);

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_MATH_VEC2D_UTILS_H_
