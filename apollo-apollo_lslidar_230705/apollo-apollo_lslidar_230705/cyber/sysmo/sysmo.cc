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

#include "cyber/sysmo/sysmo.h"

#include "cyber/common/environment.h"

namespace apollo {
namespace cyber {

using apollo::cyber::common::GetEnv;

SysMo::SysMo() { Start(); }

void SysMo::Start() {
  auto sysmo_start = GetEnv("sysmo_start");
  if (sysmo_start != "" && std::stoi(sysmo_start)) {
    start_ = true;
    sysmo_ = std::thread(&SysMo::Checker, this);
  }
}

void SysMo::Shutdown() {
  if (!start_ || shut_down_.exchange(true)) {
    return;
  }

  cv_.notify_all();
  if (sysmo_.joinable()) {
    sysmo_.join();
  }
}

void SysMo::Checker() {
  while (cyber_unlikely(!shut_down_.load())) {
    scheduler::Instance()->CheckSchedStatus();
    std::unique_lock<std::mutex> lk(lk_);
    cv_.wait_for(lk, std::chrono::milliseconds(sysmo_interval_ms_));
  }
}

}  // namespace cyber
}  // namespace apollo
