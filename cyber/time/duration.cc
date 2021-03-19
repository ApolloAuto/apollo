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

#include "cyber/time/duration.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace apollo {
namespace cyber {

Duration::Duration(int64_t nanoseconds) { nanoseconds_ = nanoseconds; }

Duration::Duration(int nanoseconds) {
  nanoseconds_ = static_cast<int64_t>(nanoseconds);
}

Duration::Duration(double seconds) {
  nanoseconds_ = static_cast<int64_t>(seconds * 1000000000UL);
}

Duration::Duration(uint32_t seconds, uint32_t nanoseconds) {
  nanoseconds_ = static_cast<uint64_t>(seconds) * 1000000000UL + nanoseconds;
}

Duration::Duration(const Duration &other) { nanoseconds_ = other.nanoseconds_; }

Duration &Duration::operator=(const Duration &other) {
  this->nanoseconds_ = other.nanoseconds_;
  return *this;
}

double Duration::ToSecond() const {
  return static_cast<double>(nanoseconds_) / 1000000000UL;
}

int64_t Duration::ToNanosecond() const { return nanoseconds_; }

bool Duration::IsZero() const { return nanoseconds_ == 0; }

void Duration::Sleep() const {
  auto sleep_time = std::chrono::nanoseconds(nanoseconds_);
  std::this_thread::sleep_for(sleep_time);
}

Duration Duration::operator+(const Duration &rhs) const {
  return Duration(nanoseconds_ + rhs.nanoseconds_);
}

Duration Duration::operator-(const Duration &rhs) const {
  return Duration(nanoseconds_ - rhs.nanoseconds_);
}

Duration Duration::operator-() const { return Duration(-nanoseconds_); }

Duration Duration::operator*(double scale) const {
  return Duration(int64_t(static_cast<double>(nanoseconds_) * scale));
}

Duration &Duration::operator+=(const Duration &rhs) {
  *this = *this + rhs;
  return *this;
}

Duration &Duration::operator-=(const Duration &rhs) {
  *this = *this - rhs;
  return *this;
}

Duration &Duration::operator*=(double scale) {
  *this = Duration(int64_t(static_cast<double>(nanoseconds_) * scale));
  return *this;
}

bool Duration::operator==(const Duration &rhs) const {
  return nanoseconds_ == rhs.nanoseconds_;
}

bool Duration::operator!=(const Duration &rhs) const {
  return nanoseconds_ != rhs.nanoseconds_;
}

bool Duration::operator>(const Duration &rhs) const {
  return nanoseconds_ > rhs.nanoseconds_;
}

bool Duration::operator<(const Duration &rhs) const {
  return nanoseconds_ < rhs.nanoseconds_;
}

bool Duration::operator>=(const Duration &rhs) const {
  return nanoseconds_ >= rhs.nanoseconds_;
}

bool Duration::operator<=(const Duration &rhs) const {
  return nanoseconds_ <= rhs.nanoseconds_;
}

std::ostream &operator<<(std::ostream &os, const Duration &rhs) {
  std::ios::fmtflags before(os.flags());
  os << std::fixed << std::setprecision(9) << rhs.ToSecond() << "s";
  os.flags(before);
  return os;
}

}  // namespace cyber
}  // namespace apollo
