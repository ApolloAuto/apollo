/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "cyber/time/clock.h"

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/util.h"

namespace apollo {
namespace cyber {

using GlobalData = ::apollo::cyber::common::GlobalData;

using AtomicRWLock = ::apollo::cyber::base::AtomicRWLock;
using AtomicWriteLockGuard =
    ::apollo::cyber::base::WriteLockGuard<AtomicRWLock>;
using AtomicReadLockGuard = ::apollo::cyber::base::ReadLockGuard<AtomicRWLock>;

Clock::Clock() {
  const auto& cyber_config = GlobalData::Instance()->Config();
  const auto& clock_mode = cyber_config.run_mode_conf().clock_mode();
  mode_ = clock_mode;
  mock_now_ = Time(0);
}

Time Clock::Now() {
  auto clock = Instance();

  AtomicReadLockGuard lg(clock->rwlock_);
  switch (clock->mode_) {
    case ClockMode::MODE_CYBER:
      return Time::Now();
    case ClockMode::MODE_MOCK:
      return clock->mock_now_;
    default:
      AFATAL << "Unsupported clock mode: "
             << apollo::cyber::common::ToInt(clock->mode_);
  }
  return Time::Now();
}

double Clock::NowInSeconds() { return Now().ToSecond(); }

void Clock::SetMode(ClockMode mode) {
  auto clock = Instance();
  AtomicWriteLockGuard lg(clock->rwlock_);
  switch (mode) {
    case ClockMode::MODE_MOCK: {
      clock->mode_ = mode;
      break;
    }
    case ClockMode::MODE_CYBER: {
      clock->mode_ = mode;
      break;
    }
    default:
      AERROR << "Unknown ClockMode: " << mode;
  }
  clock->mock_now_ = Time(0);
}

ClockMode Clock::mode() {
  auto clock = Instance();
  AtomicReadLockGuard lg(clock->rwlock_);
  return clock->mode_;
}

void Clock::SetNow(const Time& now) {
  auto clock = Instance();
  AtomicWriteLockGuard lg(clock->rwlock_);
  if (clock->mode_ != ClockMode::MODE_MOCK) {
    AERROR << "SetSimNow only works for ClockMode::MOCK";
    return;
  }
  clock->mock_now_ = now;
}

}  // namespace cyber
}  // namespace apollo
