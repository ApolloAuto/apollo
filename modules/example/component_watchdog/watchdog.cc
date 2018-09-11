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

#include "modules/example/component_watchdog/watchdog.h"
#include "cybertron/common/log.h"
#include "cybertron/time/time.h"

using apollo::cybertron::Time;

Watchdog::Watchdog() {}

Watchdog::Watchdog(const DogConfig& cfg) { cfg_ = cfg; }

Watchdog::~Watchdog() {}

SurvivalCondition Watchdog::Feed(const std::string& food) {
  uint64_t now = Time::Now().ToNanosecond() / 1000;
  std::lock_guard<std::mutex> lock(mutex_);
  if (stat_.condition != SurvivalCondition::ALIVE) {
    AERROR << "your dog is dead.";
    return stat_.condition;
  }

  if (food != cfg_.food) {
    ADEBUG << "please feed your dog with [" << cfg_.food << "] instead of ["
           << food << "].";
    return stat_.condition;
  }

  if (!stat_.activated) {
    stat_.last_eat_timestamp = now - cfg_.min_feed_interval;
    stat_.activated = true;
  }

  uint64_t interval = now - stat_.last_eat_timestamp;
  if (interval > cfg_.max_feed_interval) {
    stat_.condition = SurvivalCondition::DEAD;
    AERROR << "your dog died of hunger.";
    return stat_.condition;
  }

  if (interval < cfg_.min_feed_interval) {
    stat_.condition = SurvivalCondition::DEAD;
    AERROR << "your dog died of obesity.";
    return stat_.condition;
  }

  stat_.last_eat_timestamp = now;
  return stat_.condition;
}
