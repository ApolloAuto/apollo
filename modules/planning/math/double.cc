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

double Double::value() const { return value_; }

int Double::compare(const double d1, const double d2, const double epsilon) {
  CHECK(!std::isnan(d1));
  CHECK(!std::isnan(d2));

  if (definitely_greater_than(d1, d2, epsilon)) {
    return 1;
  } else if (definitely_less_than(d1, d2, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::compare(const double d1, const double d2) {
  return compare(d1, d2, kEpsilon_);
}

int Double::compare(const Double& d1, const Double& d2, const double epsilon) {
  return compare(d1.value(), d2.value(), epsilon);
}

int Double::compare(const Double& d1, const Double& d2) {
  return compare(d1.value(), d2.value());
}

Double Double::sqrt(const Double& d1) { return Double(std::sqrt(d1.value())); }

int Double::compare_to(const double d1, const double epsilon) const {
  CHECK(!std::isnan(d1));
  if (definitely_greater_than(value_, d1, epsilon)) {
    return 1;
  } else if (definitely_less_than(value_, d1, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::compare_to(const double d1) const {
  return compare_to(d1, kEpsilon_);
}

int Double::compare_to(const Double& d1, const double epsilon) const {
  return compare_to(d1.value(), epsilon);
}

int Double::compare_to(const Double& d1) const {
  return compare_to(d1.value(), kEpsilon_);
}

Double& Double::operator=(const Double& other) {
  value_ = other.value();
  return *this;
}

Double Double::operator+(const Double& other) const {
  return Double(value_ + other.value());
}

Double Double::operator-(const Double& other) const {
  return Double(value_ - other.value());
}

Double Double::operator*(const Double& other) const {
  return Double(value_ * other.value());
}

Double Double::operator/(const Double& other) const {
  return Double(value_ / other.value());
}

Double& Double::operator+=(const Double& other) {
  value_ += other.value();
  return *this;
}

Double& Double::operator-=(const Double& other) {
  value_ -= other.value();
  return *this;
}

Double& Double::operator*=(const Double& other) {
  value_ *= other.value();
  return *this;
}

Double& Double::operator/=(const Double& other) {
  value_ /= other.value();
  return *this;
}

bool Double::operator>(const Double& other) const {
  return definitely_greater_than(value_, other.value(), kEpsilon_);
}

bool Double::operator>=(const Double& other) const {
  return !((*this) < other);
}

bool Double::operator<(const Double& other) const {
  return definitely_less_than(value_, other.value(), kEpsilon_);
}

bool Double::operator<=(const Double& other) const {
  return !((*this) > other);
}

bool Double::operator==(const Double& other) const {
  return essentially_equal(value_, other.value(), kEpsilon_);
}

bool Double::approximately_equal(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::essentially_equal(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmin(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::definitely_greater_than(double a, double b, double epsilon) {
  return (a - b) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::definitely_less_than(double a, double b, double epsilon) {
  return (b - a) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

}  // namespace planning
}  // namespace apollo
