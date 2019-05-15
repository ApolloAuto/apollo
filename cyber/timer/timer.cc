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

#include "cyber/timer/timer.h"

#include "cyber/common/global_data.h"

namespace apollo {
namespace cyber {

namespace {
static std::atomic<uint64_t> global_timer_id = {0};
static uint64_t GenerateTimerId() { return global_timer_id.fetch_add(1); }
}  // namespace

Timer::Timer() {
  timing_wheel_ = TimingWheel::Instance();
  timer_id_ = GenerateTimerId();
}

Timer::Timer(TimerOption opt) : timer_opt_(opt) {
  timing_wheel_ = TimingWheel::Instance();
  timer_id_ = GenerateTimerId();
}

Timer::Timer(uint32_t period, std::function<void()> callback, bool oneshot) {
  timing_wheel_ = TimingWheel::Instance();
  timer_id_ = GenerateTimerId();
  timer_opt_.period = period;
  timer_opt_.callback = callback;
  timer_opt_.oneshot = oneshot;
}

void Timer::SetTimerOption(TimerOption opt) { timer_opt_ = opt; }

bool Timer::InitTimerTask() {
  if (timer_opt_.period == 0) {
    AERROR << "Max interval must great than 0";
    return false;
  }

  if (timer_opt_.period >= TIMER_MAX_INTERVAL_MS) {
    AERROR << "Max interval must less than " << TIMER_MAX_INTERVAL_MS;
    return false;
  }

  task_.reset(new TimerTask(timer_id_));
  task_->interval_ms = timer_opt_.period;
  task_->next_fire_duration_ms = task_->interval_ms;

  if (timer_opt_.oneshot) {
    task_->callback = [this](const uint64_t index) {
      (void)index;
      this->timer_opt_.callback();
    };
  } else {
    std::weak_ptr<TimerTask> task_weak_ptr = task_;
    task_->callback = [this, task_weak_ptr](const uint64_t index) {
      this->timer_opt_.callback();
      if (!started_.load()) {
        return;
      }

      if (target_tick_counts_ != 0) {
        target_tick_counts_ += task_->interval_ms;
      } else {
        target_tick_counts_ = index;
      }
      auto timing_tick_counts = this->timing_wheel_->TickCount();
      uint64_t target_index = target_tick_counts_ + task_->interval_ms;
      auto task = task_weak_ptr.lock();
      if (task) {
        if (target_index <= timing_tick_counts) {
          task->next_fire_duration_ms = 1;
          this->timing_wheel_->AddTask(task,
                                       this->timing_wheel_->WorkWheelIndex());
          if (compensate_) {
            target_tick_counts_ = this->timing_wheel_->TickCount();
          } else {
            compensate_ = true;
          }
        } else {
          if (compensate_) {
            task->next_fire_duration_ms = 1;
            this->timing_wheel_->AddTask(task,
                                         this->timing_wheel_->WorkWheelIndex());
            compensate_ = false;
            target_tick_counts_ = target_index - task_->interval_ms;
          } else {
						task->next_fire_duration_ms = task_->interval_ms;
            this->timing_wheel_->AddTask(task, index);
          }
        }
      }
    };
  }
  return true;
}

void Timer::Start() {
  if (!common::GlobalData::Instance()->IsRealityMode()) {
    return;
  }

  if (!started_.exchange(true)) {
    if (InitTimerTask()) {
      timing_wheel_->AddTask(task_);
    }
  }
}

void Timer::Stop() {
  if (started_.exchange(false)) {
    task_.reset();
  }
}

Timer::~Timer() {
  if (task_) {
    Stop();
  }
}

}  // namespace cyber
}  // namespace apollo
