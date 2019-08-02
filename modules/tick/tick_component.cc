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

#include "modules/tick/tick_component.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace tick {

using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;

bool TickComponent::Init() {
  if (!FLAGS_use_sim_time) {
    Clock::SetMode(FLAGS_use_cyber_time ? Clock::ClockMode::CYBER
                                        : Clock::ClockMode::SYSTEM);
    return true;
  }

  if (!GetProtoConfig(&conf_)) {
    AERROR << "Parse conf file failed, " << ConfigFilePath();
    return false;
  }

  Clock::SetMode(Clock::ClockMode::MOCK);
  if (conf_.whole_stack_sim()) {
    tick_listener_ = node_->CreateReader<Tick>(
      FLAGS_tick_topic,
      [this](const std::shared_ptr<Tick>& tick) {
        Clock::SetNowInSeconds(tick->header().timestamp_sec());
      });
  } else {
    localization_estimate_listener_ =
      node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate>&
                                   localization_estimate) {
        Clock::SetNowInSeconds(localization_estimate->header().timestamp_sec());
      });
  }
  return true;
}

}  // namespace tick
}  // namespace apollo
