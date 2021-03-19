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

#ifndef CYBER_TIME_DURATION_H_
#define CYBER_TIME_DURATION_H_

#include <cstdint>
#include <iostream>

namespace apollo {
namespace cyber {

class Duration {
 public:
  Duration() = default;
  explicit Duration(int64_t nanoseconds);
  explicit Duration(int nanoseconds);
  explicit Duration(double seconds);
  Duration(uint32_t seconds, uint32_t nanoseconds);
  Duration(const Duration &other);
  Duration &operator=(const Duration &other);
  ~Duration() = default;

  double ToSecond() const;
  int64_t ToNanosecond() const;
  bool IsZero() const;
  void Sleep() const;

  Duration operator+(const Duration &rhs) const;
  Duration operator-(const Duration &rhs) const;
  Duration operator-() const;
  Duration operator*(double scale) const;
  Duration &operator+=(const Duration &rhs);
  Duration &operator-=(const Duration &rhs);
  Duration &operator*=(double scale);
  bool operator==(const Duration &rhs) const;
  bool operator!=(const Duration &rhs) const;
  bool operator>(const Duration &rhs) const;
  bool operator<(const Duration &rhs) const;
  bool operator>=(const Duration &rhs) const;
  bool operator<=(const Duration &rhs) const;

 private:
  int64_t nanoseconds_ = 0;
};

std::ostream &operator<<(std::ostream &os, const Duration &rhs);

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIME_DURATION_H_
