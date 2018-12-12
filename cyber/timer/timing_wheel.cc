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

#include "cyber/timer/timing_wheel.h"

#include <algorithm>

#include "cyber/base/for_each.h"
#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "cyber/time/time.h"
#include "cyber/timer/timer_task.h"

namespace apollo {
namespace cyber {

TimingWheel::TimingWheel() {
  if (!add_queue_.Init(BOUNDED_QUEUE_SIZE)) {
    AERROR << "Add queue init failed.";
    throw std::runtime_error("Add queue init failed.");
  }
  if (!repeat_queue_.Init(BOUNDED_QUEUE_SIZE)) {
    AERROR << "Repeated task queue init failed.";
    throw std::runtime_error("Repeated queue init failed.");
  }
  if (!handler_queue_.Init(BOUNDED_QUEUE_SIZE)) {
    AERROR << "Handler queue init failed.";
    throw std::runtime_error("Handler queue init failed.");
  }
}

TimingWheel::TimingWheel(const Duration& tick_duration) {
  tick_duration_ = tick_duration.ToNanosecond();
  resolution_ = tick_duration_ / 1000000UL;
  if (!add_queue_.Init(BOUNDED_QUEUE_SIZE)) {
    AERROR << "Add queue init failed.";
    throw std::runtime_error("Add queue init failed.");
  }
  if (!repeat_queue_.Init(BOUNDED_QUEUE_SIZE)) {
    AERROR << "Repeated task queue init failed.";
    throw std::runtime_error("Repeated queue init failed.");
  }
  if (!handler_queue_.Init(BOUNDED_QUEUE_SIZE)) {
    AERROR << "Handler queue init failed.";
    throw std::runtime_error("Handler queue init failed.");
  }
}

uint64_t TimingWheel::StartTimer(uint64_t interval, CallHandler handler,
                                 bool oneshot) {
  if (id_counter_ > UINT64_MAX) {
    AERROR << "Timer ID pool is full.";
    return -1;
  } else if (interval > TIMER_TASK_MAX_INTERVAL) {
    AERROR << "The interval of timer task MUST less than or equal "
           << TIMER_TASK_MAX_INTERVAL << "ms.";
    return -1;
  } else if (interval < resolution_) {
    AERROR << "The interval of timer task MUST larger than or equal "
           << resolution_ << "ms.";
    return -1;
  }
  auto now = Time::Now().ToNanosecond();
  auto task = std::make_shared<TimerTask>(
      ++id_counter_, now, interval,
      [handler](void) {
        static std::mutex task_mutex;
        std::lock_guard<std::mutex> lock(task_mutex);
        handler();
      },
      oneshot);
  if (add_queue_.Enqueue(task)) {
    ADEBUG << "start timer id: " << id_counter_;
    return id_counter_;
  } else {
    --id_counter_;
    AERROR << "add queue is full, Enqueue failed!";
    return -1;
  }
}

void TimingWheel::Step() {
  if (start_time_ == 0) {
    start_time_ = Time::Now().ToNanosecond();
  }
  uint64_t deadline = tick_duration_ * (tick_ + 1);
  uint64_t idx = tick_ & mask_;
  RemoveCancelledTasks(idx);
  FillAddSlot();
  time_slots_[idx].EnumTaskList(deadline, true, &handler_queue_,
                                &repeat_queue_);

  // timing wheel tick one time
  tick_++;

  FillRepeatSlot();

  while (!handler_queue_.Empty()) {
    HandlePackage hp;
    if (handler_queue_.Dequeue(&hp)) {
      cyber::Async(hp.handle);
    }
  }
}

void TimingWheel::StopTimer(uint64_t timer_id) {
  {
    std::lock_guard<std::mutex> lg(cancelled_mutex_);
    cancelled_list_.push_back(timer_id);
  }
}

void TimingWheel::RemoveCancelledTasks(uint64_t slot_index) {
  if (slot_index >= TIMING_WHEEL_SIZE) {
    return;
  }
  {
    std::lock_guard<std::mutex> lg(cancelled_mutex_);
    for (auto id : cancelled_list_) {
      FOR_EACH(i, 0, TIMING_WHEEL_SIZE) {
        time_slots_[i].RemoveTask(id);
      }
    }
    cancelled_list_.clear();
  }
}

void TimingWheel::FillAddSlot() {
  std::shared_ptr<TimerTask> task;
  while (!add_queue_.Empty()) {
    if (!add_queue_.Dequeue(&task)) {
      return;
    }
    FillSlot(task);
  }
}

void TimingWheel::FillRepeatSlot() {
  std::shared_ptr<TimerTask> task;
  while (!repeat_queue_.Empty()) {
    if (!repeat_queue_.Dequeue(&task)) {
      return;
    }
    FillSlot(task);
  }
}

void TimingWheel::FillSlot(const std::shared_ptr<TimerTask>& task) {
  task->deadline_ = task->init_time_ +
                    (task->fire_count_ + 1) * (task->interval_) * 1000 * 1000 -
                    start_time_;

  // Calculate how many tickes have been run since the time wheel start
  uint64_t t = task->deadline_ / tick_duration_;  // perTick = 1ms
  if (t < tick_) {
    task->rest_rounds_ = 0;
  } else {
    task->rest_rounds_ = (t - tick_) / TIMING_WHEEL_SIZE;
  }
  uint64_t ticks = std::max(t, tick_);  // right now
  uint64_t idx = ticks & mask_;
  time_slots_[idx].AddTask(task);

  ADEBUG << "task id " << task->Id() << " insert to index " << idx;
}
}  // namespace cyber
}  // namespace apollo
