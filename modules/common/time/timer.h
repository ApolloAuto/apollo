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

#pragma once

#include <stdint.h>
#include <chrono>
#include <string>

#include "cyber/common/macros.h"

namespace apollo {
namespace common {
namespace time {

using TimePoint = std::chrono::system_clock::time_point;

class Timer {
 public:
  Timer() = default;

  // no-thread safe.
  void Start();

  // return the elapsed time,
  // also output msg and time in glog.
  // automatically start a new timer.
  // no-thread safe.
  uint64_t End(const std::string &msg);

 private:
  // in ms.
  TimePoint start_time_;
  TimePoint end_time_;

  DISALLOW_COPY_AND_ASSIGN(Timer);
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string &msg) : msg_(msg) { timer_.Start(); }

  ~TimerWrapper() { timer_.End(msg_); }

 private:
  Timer timer_;
  std::string msg_;

  DISALLOW_COPY_AND_ASSIGN(TimerWrapper);
};

}  // namespace time
}  // namespace common
}  // namespace apollo

#define PERF_FUNCTION(function_name) \
  apollo::common::time::TimerWrapper _timer_wrapper_(function_name)

#define PERF_BLOCK_START()             \
  apollo::common::time::Timer _timer_; \
  _timer_.Start()

#define PERF_BLOCK_END(msg) _timer_.End(msg)
