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

#include "modules/data/tools/smart_recorder/hard_brake_trigger.h"

#include <cmath>
#include <memory>

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo {
namespace data {

using apollo::canbus::Chassis;

HardBrakeTrigger::HardBrakeTrigger() { trigger_name_ = "HardBrakeTrigger"; }

void HardBrakeTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }

  if (msg.channel_name == FLAGS_chassis_topic) {
    Chassis chassis_msg;
    chassis_msg.ParseFromString(msg.content);
    const float speed = chassis_msg.speed_mps();

    if (IsNoisy(speed)) {
      return;
    }

    EnqueueMessage(speed);

    if (IsHardBrake()) {
      AINFO << "hard break trigger is pulled: " << msg.time << " - "
            << msg.channel_name;
      TriggerIt(msg.time);
    }
  }
}

bool HardBrakeTrigger::IsNoisy(const float speed) const {
  const float pre_speed_mps =
      (current_speed_queue_.empty() ? 0.0f : current_speed_queue_.back());
  return fabs(pre_speed_mps - speed) > noisy_diff_;
}

bool HardBrakeTrigger::IsHardBrake() const {
  if (current_speed_queue_.size() < queue_size_ ||
      history_speed_queue_.size() < queue_size_) {
    return false;
  }
  const float delta =
      (history_total_ - current_total_) / static_cast<float>(queue_size_);
  return delta > max_delta_;
}

void HardBrakeTrigger::EnqueueMessage(const float speed) {
  current_speed_queue_.emplace_back(speed);
  current_total_ += speed;
  if (current_speed_queue_.size() > queue_size_) {
    const float current_front = current_speed_queue_.front();
    current_speed_queue_.pop_front();
    current_total_ -= current_front;

    history_speed_queue_.emplace_back(current_front);
    history_total_ += current_front;
    if (history_speed_queue_.size() > queue_size_) {
      history_total_ -= history_speed_queue_.front();
      history_speed_queue_.pop_front();
    }
  }
}

}  // namespace data
}  // namespace apollo
