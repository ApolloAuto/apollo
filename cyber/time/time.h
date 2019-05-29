/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_TIME_TIME_H_
#define CYBER_TIME_TIME_H_

#include <limits>
#include <string>

#include "cyber/time/duration.h"

namespace apollo {
namespace cyber {

/**
 * @brief Cyber has builtin time type Time.
 */
class Time {
 public:
  static const Time MAX;
  static const Time MIN;
  Time() {}
  explicit Time(uint64_t nanoseconds);
  explicit Time(int nanoseconds);
  explicit Time(double seconds);
  Time(uint32_t seconds, uint32_t nanoseconds);
  Time(const Time& other);
  Time& operator=(const Time& other);
  ~Time() {}

  /**
   * @brief get the current time.
   *
   * @return return the current time.
   */
  static Time Now();
  static Time MonoTime();

  /**
   * @brief Sleep Until time.
   *
   * @param time the Time object.
   */
  static void SleepUntil(const Time& time);

  /**
   * @brief convert time to second.
   *
   * @return return a double value unit is second.
   */
  double ToSecond() const;

  /**
   * @brief convert time to nanosecond.
   *
   * @return return a unit64_t value unit is nanosecond.
   */
  uint64_t ToNanosecond() const;

  /**
   * @brief convert time to a string.
   *
   * @return return a string.
   */
  std::string ToString() const;

  /**
   * @brief determine if time is 0
   *
   * @return return true if time is 0
   */
  bool IsZero() const;

  Duration operator-(const Time& rhs) const;
  Time operator+(const Duration& rhs) const;
  Time operator-(const Duration& rhs) const;
  Time& operator+=(const Duration& rhs);
  Time& operator-=(const Duration& rhs);
  bool operator==(const Time& rhs) const;
  bool operator!=(const Time& rhs) const;
  bool operator>(const Time& rhs) const;
  bool operator<(const Time& rhs) const;
  bool operator>=(const Time& rhs) const;
  bool operator<=(const Time& rhs) const;

 private:
  uint64_t nanoseconds_ = 0;
};

std::ostream& operator<<(std::ostream& os, const Time& rhs);

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIME_TIME_H_
