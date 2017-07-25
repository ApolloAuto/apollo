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
 * @file double.h
 **/

#ifndef MODULES_PLANNING_MATH_DOUBLE_H_
#define MODULES_PLANNING_MATH_DOUBLE_H_

#include <cmath>
#include <limits>

namespace apollo {
namespace planning {

class Double {
 public:
  explicit Double(const double value);
  ~Double() = default;

  double value() const;

  static int compare(const double d1, const double d2, const double epsilon);
  static int compare(const double d1, const double d2);
  static int compare(const Double& d1, const Double& d2, const double epsilon);
  static int compare(const Double& d1, const Double& d2);

  static Double sqrt(const Double& d1);

  int compare_to(const double d1, const double epsilon) const;
  int compare_to(const double d1) const;
  int compare_to(const Double& d1, const double epsilon) const;
  int compare_to(const Double& d1) const;

  Double operator+(const Double& other) const;
  Double operator-(const Double& other) const;
  Double operator*(const Double& other) const;
  Double operator/(const Double& other) const;

  Double& operator=(const Double& other);
  Double& operator+=(const Double& other);
  Double& operator-=(const Double& other);
  Double& operator*=(const Double& other);
  Double& operator/=(const Double& other);

  bool operator>(const Double& other) const;
  bool operator>=(const Double& other) const;
  bool operator<(const Double& other) const;
  bool operator<=(const Double& other) const;
  bool operator==(const Double& other) const;

 private:
  double value_ = 0.0;
  static constexpr double kEpsilon_ = std::numeric_limits<double>::epsilon();
  static bool approximately_equal(double a, double b, double epsilon);
  static bool essentially_equal(double a, double b, double epsilon);
  static bool definitely_greater_than(double a, double b, double epsilon);
  static bool definitely_less_than(double a, double b, double epsilon);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_FACTORY_H_
