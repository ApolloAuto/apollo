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

Timer::Timer() { tm_ = TimerManager::Instance(); }

Timer::Timer(TimerOption opt) : timer_opt_(opt) {
  tm_ = TimerManager::Instance();
}

Timer::Timer(uint32_t period, std::function<void()> callback, bool oneshot) {
  tm_ = TimerManager::Instance();
  timer_opt_.period = period;
  timer_opt_.callback = callback;
  timer_opt_.oneshot = oneshot;
}

void Timer::SetTimerOption(TimerOption opt) { timer_opt_ = opt; }

void Timer::Start() {
  if (!common::GlobalData::Instance()->IsRealityMode()) {
    return;
  }

  if (!started_.exchange(true)) {
    timer_id_ =
        tm_->Add(timer_opt_.period, timer_opt_.callback, timer_opt_.oneshot);
  }
}

void Timer::Stop() {
  if (started_.exchange(false)) {
    ADEBUG << "stop timer " << timer_id_;
    tm_->Remove(timer_id_);
    timer_id_ = 0;
  }
}

Timer::~Timer() {
  if (timer_id_ != 0) {
    tm_->Remove(timer_id_);
  }
}

}  // namespace cyber
}  // namespace apollo
