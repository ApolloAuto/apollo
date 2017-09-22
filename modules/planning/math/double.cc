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

#include "modules/planning/math/double.h"

#include <cmath>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

Double::Double(const double value) : value_(value) {
  CHECK(!std::isnan(value));
}

double Double::Value() const { return value_; }

int Double::Compare(const double d1, const double d2, const double epsilon) {
  CHECK(!std::isnan(d1));
  CHECK(!std::isnan(d2));

  if (DefinitelyGreaterThan(d1, d2, epsilon)) {
    return 1;
  } else if (DefinitelyLessThan(d1, d2, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::Compare(const double d1, const double d2) {
  return Compare(d1, d2, kEpsilon_);
}

int Double::Compare(const Double& d1, const Double& d2, const double epsilon) {
  return Compare(d1.Value(), d2.Value(), epsilon);
}

int Double::Compare(const Double& d1, const Double& d2) {
  return Compare(d1.Value(), d2.Value());
}

int Double::CompareTo(const double d1, const double epsilon) const {
  CHECK(!std::isnan(d1));
  if (DefinitelyGreaterThan(value_, d1, epsilon)) {
    return 1;
  } else if (DefinitelyLessThan(value_, d1, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::CompareTo(const double d1) const {
  return CompareTo(d1, kEpsilon_);
}

int Double::CompareTo(const Double& d1, const double epsilon) const {
  return CompareTo(d1.Value(), epsilon);
}

int Double::CompareTo(const Double& d1) const {
  return CompareTo(d1.Value(), kEpsilon_);
}

Double& Double::operator=(const Double& other) {
  value_ = other.Value();
  return *this;
}

Double Double::operator+(const Double& other) const {
  return Double(value_ + other.Value());
}

Double Double::operator-(const Double& other) const {
  return Double(value_ - other.Value());
}

Double Double::operator*(const Double& other) const {
  return Double(value_ * other.Value());
}

Double Double::operator/(const Double& other) const {
  return Double(value_ / other.Value());
}

Double& Double::operator+=(const Double& other) {
  value_ += other.Value();
  return *this;
}

Double& Double::operator-=(const Double& other) {
  value_ -= other.Value();
  return *this;
}

Double& Double::operator*=(const Double& other) {
  value_ *= other.Value();
  return *this;
}

Double& Double::operator/=(const Double& other) {
  value_ /= other.Value();
  return *this;
}

bool Double::operator>(const Double& other) const {
  return DefinitelyGreaterThan(value_, other.Value(), kEpsilon_);
}

bool Double::operator>=(const Double& other) const {
  return !((*this) < other);
}

bool Double::operator<(const Double& other) const {
  return DefinitelyLessThan(value_, other.Value(), kEpsilon_);
}

bool Double::operator<=(const Double& other) const {
  return !((*this) > other);
}

bool Double::operator==(const Double& other) const {
  return EssentiallyEqual(value_, other.Value(), kEpsilon_);
}

bool Double::ApproximatelyEqual(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::EssentiallyEqual(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmin(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::DefinitelyGreaterThan(double a, double b, double epsilon) {
  return (a - b) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::DefinitelyLessThan(double a, double b, double epsilon) {
  return (b - a) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

}  // namespace planning
}  // namespace apollo
