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

#ifndef CYBER_TIMER_TIMER_MANAGER_H_
#define CYBER_TIMER_TIMER_MANAGER_H_

#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "cyber/common/macros.h"
#include "cyber/time/duration.h"
#include "cyber/timer/timing_wheel.h"

namespace apollo {
namespace cyber {

class TimerManager {
 public:
  virtual ~TimerManager();
  void Start();
  void Shutdown();
  bool IsRunning();
  uint64_t Add(uint64_t interval, std::function<void()> handler, bool oneshot);
  void Remove(uint64_t timer_id);

 private:
  TimingWheel timing_wheel_;
  Duration time_gran_;
  bool running_ = false;
  mutable std::mutex running_mutex_;
  std::thread scheduler_thread_;
  void ThreadFuncImpl();

  DECLARE_SINGLETON(TimerManager)
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMER_MANAGER_H_
