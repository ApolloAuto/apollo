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

#ifndef MODULES_PERCEPTION_LIB_BASE_TIMER_H_
#define MODULES_PERCEPTION_LIB_BASE_TIMER_H_

#include <stdint.h>
#include <chrono>
#include <string>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

using TimePoint = std::chrono::system_clock::time_point;

class Timer {
 public:
  Timer() = default;

  // no-thread safe.
  void start();

  // return the elapsed time,
  // also output msg and time in glog.
  // automatically start a new timer.
  // no-thread safe.
  uint64_t end(const std::string& msg);

 private:
  // in ms.
  TimePoint _start_time;
  TimePoint _end_time;

  DISALLOW_COPY_AND_ASSIGN(Timer);
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string& msg) : _msg(msg) {
    _timer.start();
  }

  ~TimerWrapper() {
    _timer.end(_msg);
  }

 private:
  Timer _timer;
  std::string _msg;

  DISALLOW_COPY_AND_ASSIGN(TimerWrapper);
};

}  // namespace perception
}  // namespace apollo

#define PERF_FUNCTION(function_name) \
  apollo::perception::TimerWrapper _timer_wrapper_(function_name)

#define PERF_BLOCK_START()              \
  apollo::perception::Timer _timer_; \
  _timer_.start()

#define PERF_BLOCK_END(msg) _timer_.end(msg)

#endif  // MODULES_PERCEPTION_LIB_BASE_TIMER_H_
