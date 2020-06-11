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

#pragma once

#include <string>

namespace apollo {
namespace perception {
namespace lib {

class Timer {
 public:
  Timer() : start_time_(0), end_time_(0) {}

  // no-thread safe.
  void Start();

  // return the elapsed time,
  // also output msg and time in glog.
  // automatically start a new timer.
  // no-thread safe.
  uint64_t End(const std::string &msg);

  Timer(const Timer &) = delete;
  Timer &operator=(const Timer &) = delete;

 private:
  // in ms.
  uint64_t start_time_;
  uint64_t end_time_;
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string &msg) : msg_(msg) { timer_.Start(); }

  ~TimerWrapper() { timer_.End(msg_); }

  TimerWrapper(const TimerWrapper &) = delete;
  TimerWrapper &operator=(const TimerWrapper &) = delete;

 private:
  Timer timer_;
  std::string msg_;
};

}  // namespace lib
}  // namespace perception
}  // namespace apollo
