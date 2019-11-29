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

#ifndef CYBER_TIMER_TIMING_WHEEL_H_
#define CYBER_TIMER_TIMING_WHEEL_H_

#include <future>
#include <list>
#include <memory>
#include <thread>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/time/rate.h"
#include "cyber/timer/timer_bucket.h"

namespace apollo {
namespace cyber {

struct TimerTask;

static const uint64_t WORK_WHEEL_SIZE = 512;
static const uint64_t ASSISTANT_WHEEL_SIZE = 64;
static const uint64_t TIMER_RESOLUTION_MS = 2;
static const uint64_t TIMER_MAX_INTERVAL_MS =
    WORK_WHEEL_SIZE * ASSISTANT_WHEEL_SIZE * TIMER_RESOLUTION_MS;

class TimingWheel {
 public:
  ~TimingWheel() {
    if (running_) {
      Shutdown();
    }
  }

  void Start();

  void Shutdown();

  void Tick();

  void AddTask(const std::shared_ptr<TimerTask>& task);

  void AddTask(const std::shared_ptr<TimerTask>& task,
               const uint64_t current_work_wheel_index);

  void Cascade(const uint64_t assistant_wheel_index);

  void TickFunc();

  inline uint64_t TickCount() const { return tick_count_; }

 private:
  inline uint64_t GetWorkWheelIndex(const uint64_t index) {
    return index & (WORK_WHEEL_SIZE - 1);
  }
  inline uint64_t GetAssistantWheelIndex(const uint64_t index) {
    return index & (ASSISTANT_WHEEL_SIZE - 1);
  }

  bool running_ = false;
  uint64_t tick_count_ = 0;
  std::mutex running_mutex_;
  TimerBucket work_wheel_[WORK_WHEEL_SIZE];
  TimerBucket assistant_wheel_[ASSISTANT_WHEEL_SIZE];
  uint64_t current_work_wheel_index_ = 0;
  std::mutex current_work_wheel_index_mutex_;
  uint64_t current_assistant_wheel_index_ = 0;
  std::mutex current_assistant_wheel_index_mutex_;
  std::thread tick_thread_;

  DECLARE_SINGLETON(TimingWheel)
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMING_WHEEL_H_
