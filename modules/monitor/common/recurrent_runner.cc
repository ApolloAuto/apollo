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

#include "cyber/common/log.h"

namespace apollo {
namespace monitor {

RecurrentRunner::RecurrentRunner(const std::string &name, const double interval)
    : name_(name), interval_(interval) {}

void RecurrentRunner::Tick(const double current_time) {
  if (name_ == "ProcessMonitor" &&
      MonitorManager::Instance()->GetStatus()->detect_immediately()) {
    RunOnce(current_time);
  } else if (next_round_ <= current_time) {
    ++round_count_;
    AINFO_EVERY(100) << name_ << " is running round #" << round_count_;
    next_round_ = current_time + interval_;
    RunOnce(current_time);
  }
}

}  // namespace monitor
}  // namespace apollo
