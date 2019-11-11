/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_PY_WRAPPER_PY_TIME_H_
#define CYBER_PY_WRAPPER_PY_TIME_H_

#include <unistd.h>
#include <memory>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {

class PyTime {
 public:
  PyTime() = default;
  explicit PyTime(uint64_t nanoseconds) { time_ = Time(nanoseconds); }

  static PyTime now() {
    PyTime t;
    t.time_ = Time::Now();
    return t;
  }

  static PyTime mono_time() {
    PyTime t;
    t.time_ = Time::MonoTime();
    return t;
  }

  static void sleep_until(uint64_t nanoseconds) {
    Time::SleepUntil(Time(nanoseconds));
  }

  double to_sec() const { return time_.ToSecond(); }

  uint64_t to_nsec() const { return time_.ToNanosecond(); }

 private:
  Time time_;
};

class PyDuration {
 public:
  explicit PyDuration(int64_t nanoseconds) {
    duration_ = std::make_shared<Duration>(nanoseconds);
  }

  void sleep() const { return duration_->Sleep(); }

 private:
  std::shared_ptr<Duration> duration_ = nullptr;
};

class PyRate {
 public:
  explicit PyRate(uint64_t nanoseconds) {
    rate_ = std::make_shared<Rate>(nanoseconds);
  }

  void sleep() const { return rate_->Sleep(); }
  void reset() const { return rate_->Reset(); }
  uint64_t get_cycle_time() const { return rate_->CycleTime().ToNanosecond(); }
  uint64_t get_expected_cycle_time() const {
    return rate_->ExpectedCycleTime().ToNanosecond();
  }

 private:
  std::shared_ptr<Rate> rate_ = nullptr;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PY_WRAPPER_PY_TIME_H_
