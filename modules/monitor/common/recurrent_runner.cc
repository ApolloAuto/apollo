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

#include "modules/monitor/common/recurrent_runner.h"

#include <utility>

#include "modules/common/log.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace monitor {

using apollo::common::time::Clock;

RecurrentRunner::RecurrentRunner(const std::string &name,
                                 const double interval)
    : name_(name)
    , interval_(interval) {
}

void RecurrentRunner::Tick(const double current_time) {
  if (next_round_ <= current_time) {
    ADEBUG << "RecurrentRunner " << name_ << " runs at " << current_time;
    next_round_ = current_time + interval_;
    RunOnce(current_time);
  }
}

RecurrentRunnerThread::RecurrentRunnerThread(const double interval)
    : interval_ms_(interval * 1000) {
}

void RecurrentRunnerThread::RegisterRunner(
    std::unique_ptr<RecurrentRunner> runner) {
  runners_.emplace_back(std::move(runner));
}

void RecurrentRunnerThread::Start() {
  CHECK(!thread_) << "Thread has already started.";
  thread_.reset(new std::thread([this]() {
    while (true) {
      {
        // Check stop sign.
        std::lock_guard<std::mutex> guard(stop_mutex_);
        if (stop_) {
          return;
        }
      }

      // Tick runners.
      const double current_time = Clock::NowInSeconds();
      for (auto &runner : runners_) {
        runner->Tick(current_time);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }
  }));
}

void RecurrentRunnerThread::Stop() {
  CHECK(thread_) << "Thread has already stopped.";
  {
    std::lock_guard<std::mutex> guard(stop_mutex_);
    stop_ = true;
  }
  thread_->join();
  thread_.reset(nullptr);
}

}  // namespace monitor
}  // namespace apollo
