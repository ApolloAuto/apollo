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

#include "cyber/timer/timer_manager.h"

#include "cyber/common/log.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/time/duration.h"
#include "cyber/time/rate.h"

namespace apollo {
namespace cyber {

TimerManager::TimerManager()
    : timing_wheel_(Duration(0.01)),
      time_gran_(Duration(0.01)),
      running_(false) {}  // default time gran = 1ms

TimerManager::~TimerManager() {
  if (running_) {
    Shutdown();
  }
}

void TimerManager::Start() {
  std::lock_guard<std::mutex> lock(running_mutex_);
  if (!running_) {
    ADEBUG << "TimerManager->Start() ok";
    running_ = true;
    scheduler_thread_ = std::thread([this]() { this->ThreadFuncImpl(); });
    scheduler::Instance()->SetInnerThreadAttr(&scheduler_thread_, "timer");
  }
}

void TimerManager::Shutdown() {
  std::lock_guard<std::mutex> lock(running_mutex_);
  if (running_) {
    running_ = false;
    if (scheduler_thread_.joinable()) {
      scheduler_thread_.join();
    }
  }
}

uint64_t TimerManager::Add(uint64_t interval, std::function<void()> handler,
                           bool oneshot) {
  if (!running_) {
    Start();
  }
  uint64_t timer_id = timing_wheel_.StartTimer(interval, handler, oneshot);
  return timer_id;
}

void TimerManager::Remove(uint64_t timer_id) {
  timing_wheel_.StopTimer(timer_id);
}

bool TimerManager::IsRunning() { return running_; }

void TimerManager::ThreadFuncImpl() {
  Rate rate(time_gran_);
  while (running_) {
    timing_wheel_.Step();
    rate.Sleep();
  }
}

}  // namespace cyber
}  // namespace apollo
