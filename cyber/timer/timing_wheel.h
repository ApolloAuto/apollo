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

#ifndef CYBER_TIMER_TIMING_WHEEL_H_
#define CYBER_TIMER_TIMING_WHEEL_H_

#include <algorithm>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/base/bounded_queue.h"
#include "cyber/time/duration.h"
#include "cyber/timer/timing_slot.h"

namespace apollo {
namespace cyber {

using apollo::cyber::base::BoundedQueue;
using CallHandler = std::function<void()>;

static const int TIMING_WHEEL_SIZE = 128;
static const int THREAD_POOL_SIZE = 4;
static const uint64_t BOUNDED_QUEUE_SIZE = 200;
static const int TIMER_TASK_MAX_INTERVAL = 1000;

class TimerTask;

class TimingWheel {
 public:
  TimingWheel();
  explicit TimingWheel(const Duration& tick_duration);
  ~TimingWheel() = default;

  uint64_t StartTimer(uint64_t interval, CallHandler handler, bool oneshot);

  void StopTimer(uint64_t timer_id);

  void Step();

 private:
  void FillAddSlot();
  void FillRepeatSlot();
  void FillSlot(const std::shared_ptr<TimerTask>& task);

  void RemoveCancelledTasks(uint64_t slot_index);

  uint64_t id_counter_ = 0;

  uint64_t tick_ = 0;

  uint64_t start_time_ = 0;

  TimingSlot time_slots_[TIMING_WHEEL_SIZE];

  uint64_t mask_ = TIMING_WHEEL_SIZE - 1;

  uint64_t tick_duration_ = 10 * 1000 * 1000;  // 10ms
  uint64_t resolution_ = 10;                   // 10ms

  // we need implement a lock-free high performance concurrent queue.
  // Now, just a blocking-queue just for works.
  std::list<uint64_t> cancelled_list_;
  std::mutex cancelled_mutex_;
  BoundedQueue<std::shared_ptr<TimerTask>> add_queue_;
  BoundedQueue<std::shared_ptr<TimerTask>> repeat_queue_;
  BoundedQueue<HandlePackage> handler_queue_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMING_WHEEL_H_
