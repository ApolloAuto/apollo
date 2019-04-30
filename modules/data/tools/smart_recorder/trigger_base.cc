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
  for (const auto& iter : trigger_obj_->wanted_channel_types()) {
    InsertChannelType(iter.first, iter.second);
  }
  return true;
}

void TriggerBase::LockTrigger(const SmartRecordTrigger& trigger_conf) {
  for (const auto& trigger : trigger_conf.triggers()) {
    if (trigger.trigger_name() == trigger_name_) {
      trigger_obj_.reset(new Trigger(trigger));
      break;
    }
  }
}

void TriggerBase::TriggerIt(const uint64_t msg_time) const {
  const uint64_t backword_time =
      trigger_obj_->backward_time() < 0 || trigger_obj_->backward_time() > 60
          ? 0
          : static_cast<uint64_t>(trigger_obj_->backward_time() * 1000000000UL);
  const uint64_t forward_time =
      trigger_obj_->forward_time() < 0 || trigger_obj_->forward_time() > 60
          ? 0
          : static_cast<uint64_t>(trigger_obj_->forward_time() * 1000000000UL);
  const uint64_t begin_time = msg_time - backword_time;
  const uint64_t end_time = msg_time + forward_time;
  IntervalPool::Instance()->AddInterval(begin_time, end_time,
                                        channels_types_map_);
}

}  // namespace data
}  // namespace apollo
