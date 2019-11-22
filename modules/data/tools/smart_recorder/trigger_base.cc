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

#include "modules/data/tools/smart_recorder/trigger_base.h"

#include "cyber/common/log.h"
#include "modules/data/tools/smart_recorder/interval_pool.h"

namespace apollo {
namespace data {

bool TriggerBase::Init(const SmartRecordTrigger& trigger_conf) {
  LockTrigger(trigger_conf);
  if (trigger_obj_ == nullptr) {
    AERROR << "failed to lock trigger " << GetTriggerName();
    return false;
  }
  return true;
}

uint64_t TriggerBase::SecondsToNanoSeconds(const double seconds) const {
  static constexpr uint64_t kSecondsToNanoSecondsFactor = 1000000000UL;
  return static_cast<uint64_t>(kSecondsToNanoSecondsFactor * seconds);
}

void TriggerBase::LockTrigger(const SmartRecordTrigger& trigger_conf) {
  for (const auto& trigger : trigger_conf.triggers()) {
    if (trigger.trigger_name() == trigger_name_) {
      trigger_obj_.reset(new Trigger(trigger));
      break;
    }
  }
}

uint64_t TriggerBase::GetValidValueInRange(const double desired_value,
                                           const double min_limit,
                                           const double max_limit) const {
  return SecondsToNanoSeconds(desired_value < min_limit
                                  ? min_limit
                                  : desired_value > max_limit ? max_limit
                                                              : desired_value);
}

void TriggerBase::TriggerIt(const uint64_t msg_time) const {
  static constexpr float kMaxBackwardTime = 30.0;
  static constexpr float kMaxForwardTime = 15.0;
  static constexpr uint64_t kZero = 0.0;
  const uint64_t backward_time = GetValidValueInRange(
      trigger_obj_->backward_time(), kZero, kMaxBackwardTime);
  const uint64_t forward_time = GetValidValueInRange(
      trigger_obj_->forward_time(), kZero, kMaxForwardTime);
  IntervalPool::Instance()->AddInterval(msg_time - backward_time,
                                        msg_time + forward_time);
  IntervalPool::Instance()->LogIntervalEvent(
      trigger_obj_->trigger_name(), trigger_obj_->description(), msg_time,
      backward_time, forward_time);
}

}  // namespace data
}  // namespace apollo
