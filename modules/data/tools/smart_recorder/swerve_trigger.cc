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

#include "modules/data/tools/smart_recorder/swerve_trigger.h"

#include <cmath>
#include <memory>

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo {
namespace data {

using apollo::canbus::Chassis;
constexpr float MAX_STEER_PER = 100.0;
constexpr float MIN_STEER_PER = -100.0;

SwerveTrigger::SwerveTrigger() { trigger_name_ = "SwerveTrigger"; }

void SwerveTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }

  if (msg.channel_name == FLAGS_chassis_topic) {
    Chassis chassis_msg;
    chassis_msg.ParseFromString(msg.content);
    const float steer_per = chassis_msg.steering_percentage();

    if (IsNoisy(steer_per)) {
      return;
    }

    EnqueueMessage(steer_per);

    if (IsSwerve()) {
      AINFO << "swerve trigger is pulled: " << msg.time << " - "
            << msg.channel_name;
      TriggerIt(msg.time);
    }
  }
}

bool SwerveTrigger::IsNoisy(const float steer) const {
  if (steer > MAX_STEER_PER || steer < MIN_STEER_PER) {
    return true;
  }
  const float pre_steer_mps =
      (current_steer_queue_.empty() ? 0.0f : current_steer_queue_.back());
  return fabs(pre_steer_mps - steer) > noisy_diff_;
}

bool SwerveTrigger::IsSwerve() const {
  if (current_steer_queue_.size() < queue_size_ ||
      history_steer_queue_.size() < queue_size_) {
    return false;
  }
  const float delta =
      (history_total_ - current_total_) / static_cast<float>(queue_size_);
  return delta > max_delta_;
}

// TODO(Leisheng Mu): reuse the code with hard_brake_trigger in next iteration
void SwerveTrigger::EnqueueMessage(const float steer) {
  current_steer_queue_.emplace_back(steer);
  current_total_ += steer;
  if (current_steer_queue_.size() > queue_size_) {
    const float current_front = current_steer_queue_.front();
    current_steer_queue_.pop_front();
    current_total_ -= current_front;

    history_steer_queue_.emplace_back(current_front);
    history_total_ += current_front;
    if (history_steer_queue_.size() > queue_size_) {
      history_total_ -= history_steer_queue_.front();
      history_steer_queue_.pop_front();
    }
  }
}

}  // namespace data
}  // namespace apollo
