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

constexpr int ITEM_SIZE = 10;
constexpr float MAX_DELTA = 10.0f;
constexpr float MAX_DIFF = 20.0f;
constexpr float FLOAT_ZERO = 0.0001f;

HardBrakeTrigger::HardBrakeTrigger() {
  trigger_name_ = "HardBrakeTrigger";
}

void HardBrakeTrigger::Pull(const RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }

  if (msg.channel_name == FLAGS_chassis_topic) {
    std::shared_ptr<apollo::canbus::Chassis> chassis_msg =
      std::make_shared<apollo::canbus::Chassis>();
    chassis_msg->ParseFromString(msg.content);
    if (IsNoisy(chassis_msg)) {
      return;
    }
    PushToList(chassis_msg);
    if (IsHardBrake()) {
      AINFO << "hard break trigger is pulled: " << msg.time << " - "
            << msg.channel_name;
      TriggerIt(msg.time);
    }
  }
}

bool HardBrakeTrigger::IsNoisy(const std::shared_ptr<canbus::Chassis>& msg) {
  float pre_speed_mps = 0.0f;
  float cur_speed_mps = msg->speed_mps();
  if (cur_msg_list_.size() > 0) {
    size_t num = cur_msg_list_.size() - 1;
    pre_speed_mps = cur_msg_list_[num]->speed_mps();
  }
  return  (fabs(pre_speed_mps - cur_speed_mps) > MAX_DIFF);
}

bool HardBrakeTrigger::IsHardBrake() {
  float his_speed_mean_value = GetMeanSpeed(his_msg_list_);
  float cur_speed_mean_value = GetMeanSpeed(cur_msg_list_);

  float delta = his_speed_mean_value - cur_speed_mean_value;
  return (delta > MAX_DELTA);
}

float HardBrakeTrigger::GetMeanSpeed(
    const std::vector<std::shared_ptr<canbus::Chassis>> &msg_list) {
  float speed_points = 0.0f;
  if (msg_list.empty()) {
    return 0.0f;
  }
  for (const auto& msg : msg_list) {
    speed_points += msg->speed_mps();
  }
  return speed_points / static_cast<float>(msg_list.size());
}

void HardBrakeTrigger::PushToList(
  const std::shared_ptr<canbus::Chassis>& msg) {
  cur_msg_list_.push_back(msg);
  if (cur_msg_list_.size() > ITEM_SIZE) {
    size_t offset = cur_msg_list_.size() - ITEM_SIZE;
    for (size_t i = 0; i < offset; i++) {
      his_msg_list_.push_back(cur_msg_list_[i]);
    }
    cur_msg_list_.erase(cur_msg_list_.begin(), cur_msg_list_.begin() + offset);
  }
  if (his_msg_list_.size() > ITEM_SIZE) {
    size_t offset = his_msg_list_.size() - ITEM_SIZE;
    his_msg_list_.erase(his_msg_list_.begin(), his_msg_list_.begin() + offset);
  }
}

}  // namespace data
}  // namespace apollo
