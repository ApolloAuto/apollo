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
 * @file
 * @brief Defines the templated Angle class.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @class Angle
 * @brief The Angle class uses an integer to represent an angle, and supports
 * commonly-used operations such as addition and subtraction,
 * as well as the use of trigonometric functions.
 *
 * Having a specialized Angle class prevents code repetition, namely for tasks
 * such as computing angle differences or finding equivalent angles in some
 * specified interval, typically [-pi, pi).
 * Representing angles by integers has the following advantages:
 * 1) Finer level of precision control (< means "less precise than"):
 * Angle8 < Angle16 < float < Angle32 < double < Angle64.
 * 2) Angle8 and Angle16 allow super fast trigonometric functions
 * via a 64-KiB lookup table.
 * 3) Higher precision with the same representation size.
 * The use of the Angle class is encouraged.
 * In particular, Angle32 should be used for latitude/longitude (<1cm error).
 * Angle16 is precise enough for localization/object detection.

 * @param T signed integer type
*/
template <typename T>
class Angle {
 public:
  static_assert(std::numeric_limits<T>::is_integer &&
                    std::numeric_limits<T>::is_signed,
                "T must be a signed integer type");

  /**
   * @brief Constructs an Angle object from an angle in degrees (factory).
   * @param value Angle in degrees
   * @return Angle object
   */
  static Angle from_deg(const double value) {
    return Angle(static_cast<T>(std::lround(value * DEG_TO_RAW)));
  }

  /**
   * @brief Constructs an Angle object from an angle in radians (factory).
   * @param value Angle in radians
   * @return Angle object
   */
  static Angle from_rad(const double value) {
    return Angle(static_cast<T>(std::lround(value * RAD_TO_RAW)));
  }

  /**
   * @brief Constructs an Angle object from raw internal value.
   * @param value Angle in degrees
   * @return Angle object
   */
  explicit Angle(const T value = 0) : value_(value) {}

  /// Internal representation of pi
  static constexpr T RAW_PI = std::numeric_limits<T>::min();

  /// Internal representation of pi/2
  static constexpr T RAW_PI_2 =
      static_cast<T>(-(std::numeric_limits<T>::min() >> 1));

  /// Used for converting angle units
  static constexpr double DEG_TO_RAW = RAW_PI / -180.0;

  /// Used for converting angle units
  static constexpr double RAD_TO_RAW = RAW_PI * -M_1_PI;

  /// Used for converting angle units
  static constexpr double RAW_TO_DEG = -180.0 / RAW_PI;

  /// Used for converting angle units
  static constexpr double RAW_TO_RAD = -M_PI / RAW_PI;

  /**
   * @brief Getter of value_.
   * @return Internal unsigned integer representation of angle
   */
  T raw() const { return value_; }

  /**
   * @brief Converts the internal representation to degrees.
   * @return angle in degrees
   */
  double to_deg() const { return value_ * RAW_TO_DEG; }

  /**
   * @brief Converts the internal representation to radians.
   * @return angle in radians
   */
  double to_rad() const { return value_ * RAW_TO_RAD; }

  /**
   * @brief Sums another angle to the current one.
   * @param other Another Angle object
   * @return Result of sum
   */
  Angle operator+=(Angle other) {
    value_ = static_cast<T>(value_ + other.value_);
    return *this;
  }

  /**
   * @brief Subtracts another angle from the current one.
   * @param other Another Angle object
   * @return Result of subtraction
   */
  Angle operator-=(Angle other) {
    value_ = static_cast<T>(value_ - other.value_);
    return *this;
  }

  /**
   * @brief Multiplies angle by scalar
   * @param s A scalar
   * @return Result of multiplication
   */
  template <typename Scalar>
  Angle operator*=(Scalar s) {
    value_ = static_cast<T>(std::lround(value_ * s));
    return *this;
  }

  /**
   * @brief Divides angle by scalar
   * @param s A scalar
   * @return Result of division
   */
  template <typename Scalar>
  Angle operator/=(Scalar s) {
    value_ = static_cast<T>(std::lround(value_ / s));
    return *this;
  }

 private:
  T value_;
};

using Angle8 = Angle<int8_t>;
using Angle16 = Angle<int16_t>;
using Angle32 = Angle<int32_t>;
using Angle64 = Angle<int64_t>;

/**
 * @brief Sums two angles
 * @param lhs An Angle object
 * @param rhs An Angle object
 * @return Result of addition
 */
template <typename T>
Angle<T> operator+(Angle<T> lhs, Angle<T> rhs) {
  lhs += rhs;
  return lhs;
}

/**
 * @brief Subtracts two angles
 * @param lhs An Angle object
 * @param rhs An Angle object
 * @return Result of subtraction
 */
template <typename T>
Angle<T> operator-(Angle<T> lhs, Angle<T> rhs) {
  lhs -= rhs;
  return lhs;
}

/**
 * @brief Multiplies an Angle by a scalar
 * @param lhs An Angle object
 * @param rhs A scalar
 * @return Result of multiplication
 */
template <typename T, typename Scalar>
Angle<T> operator*(Angle<T> lhs, Scalar rhs) {
  lhs *= rhs;
  return lhs;
}

/**
 * @brief Multiplies an Angle by a scalar
 * @param lhs An Angle object
 * @param rhs A scalar
 * @return Result of multiplication
 */
template <typename T, typename Scalar>
Angle<T> operator*(Scalar lhs, Angle<T> rhs) {
  rhs *= lhs;
  return rhs;
}

/**
 * @brief Divides an Angle by a scalar
 * @param lhs An Angle object
 * @param rhs A scalar
 * @return Result of division
 */
template <typename T, typename Scalar>
Angle<T> operator/(Angle<T> lhs, Scalar rhs) {
  lhs /= rhs;
  return lhs;
}

/**
 * @brief Divides an Angle by a scalar
 * @param lhs An Angle object
 * @param rhs A scalar
 * @return Result of division
 */
template <typename T>
double operator/(Angle<T> lhs, Angle<T> rhs) {
  return static_cast<double>(lhs.raw()) / rhs.raw();
}

/**
 * @brief Tests two Angle objects for equality
 * @param lhs An Angle object
 * @param rhs An Angle object
 * @return lhs == rhs
 */
template <typename T>
bool operator==(Angle<T> lhs, Angle<T> rhs) {
  return lhs.raw() == rhs.raw();
}

/**
 * @brief Tests two Angle objects for inequality
 * @param lhs An Angle object
 * @param rhs An Angle object
 * @return lhs != rhs
 */
template <typename T>
bool operator!=(Angle<T> lhs, Angle<T> rhs) {
  return !(lhs == rhs);
}

// Fast trigonometric functions. Single precision is sufficient for Angle16 and
// Angle8.
float sin(Angle16 a);
float cos(Angle16 a);
float tan(Angle16 a);
float sin(Angle8 a);
float cos(Angle8 a);
float tan(Angle8 a);

}  // namespace math
}  // namespace common
}  // namespace apollo
