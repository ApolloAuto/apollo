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

#include "modules/common/math/vec2d.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace common {
namespace math {

Vec2d Vec2d::CreateUnitVec2d(const double angle) {
  return Vec2d(cos(angle), sin(angle));
}

double Vec2d::Length() const { return std::hypot(x_, y_); }

double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec2d::Angle() const { return std::atan2(y_, x_); }

void Vec2d::Normalize() {
  const double l = Length();
  if (l > kMathEpsilon) {
    x_ /= l;
    y_ /= l;
  }
}

double Vec2d::DistanceTo(const Vec2d &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec2d::DistanceSquareTo(const Vec2d &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Vec2d::CrossProd(const Vec2d &other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec2d::InnerProd(const Vec2d &other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec2d Vec2d::rotate(const double angle) const {
  return Vec2d(x_ * cos(angle) - y_ * sin(angle),
               x_ * sin(angle) + y_ * cos(angle));
}

void Vec2d::SelfRotate(const double angle) {
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2d Vec2d::operator+(const Vec2d &other) const {
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-(const Vec2d &other) const {
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const {
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const {
  CHECK_GT(std::abs(ratio), kMathEpsilon);
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d &Vec2d::operator+=(const Vec2d &other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d &Vec2d::operator-=(const Vec2d &other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d &Vec2d::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d &Vec2d::operator/=(const double ratio) {
  CHECK_GT(std::abs(ratio), kMathEpsilon);
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

std::string Vec2d::DebugString() const {
  return util::StrCat("vec2d ( x = ", x_, "  y = ", y_, " )");
}

}  // namespace math
}  // namespace common
}  // namespace apollo
