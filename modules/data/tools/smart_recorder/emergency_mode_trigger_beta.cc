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

#include "modules/data/tools/smart_recorder/emergency_mode_trigger_beta.h"

#include <memory>

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo {
namespace data {

using apollo::canbus::Chassis;

constexpr int ITEM_SIZE = 10;
constexpr float MAX_DELTA = 10.0f;

EmergencyModeTriggerBeta::EmergencyModeTriggerBeta() {
  trigger_name_ = "EmergencyModeTriggerBeta";
}

void EmergencyModeTriggerBeta::Pull(const RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }
  std::shared_ptr<EmergencyMessage> emergency_msg =
    std::make_shared<EmergencyMessage>();
  emergency_msg->Initial(msg);

  if (msg.channel_name == FLAGS_chassis_topic) {
    if (cur_driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE &&
        emergency_msg->chassis_msg_->driving_mode() ==
        Chassis::EMERGENCY_MODE && IsEmergency(emergency_msg)) {
      AINFO << "emergency mode trigger is pulled: " << msg.time << " - "
            << msg.channel_name;
      TriggerIt(msg.time);
    }
    cur_driving_mode_ = emergency_msg->chassis_msg_->driving_mode();
  }
  PushToHistoryList(emergency_msg);
}

bool EmergencyModeTriggerBeta::IsEmergency(
  const std::shared_ptr<EmergencyMessage>& msg) {
  float his_speed_mean_value = GetHistoryMeanSpeed();
  float delta = his_speed_mean_value - msg->chassis_msg_->speed_mps();
  if (delta > MAX_DELTA) {
    return true;
  }
  return false;
}

float EmergencyModeTriggerBeta::GetHistoryMeanSpeed() {
  float speed_points = 0.0f;
  if (msg_list_.size() == 0) {
    return 0.0f;
  }
  for (size_t  i =0; i < msg_list_.size(); i ++) {
    speed_points += msg_list_[i]->chassis_msg_->speed_mps();
  }
  return speed_points/static_cast<float>(msg_list_.size());
}

void EmergencyModeTriggerBeta::PushToHistoryList(
  const std::shared_ptr<EmergencyMessage>& msg) {
  msg_list_.push_back(msg);
  if (msg_list_.size() > ITEM_SIZE) {
    size_t offset = msg_list_.size() - ITEM_SIZE;
    msg_list_.erase(msg_list_.begin(), msg_list_.begin() + offset);
  }
}

bool EmergencyMessage::Initial(const RecordMessage& msg) {
  record_msg_ = std::make_shared<RecordMessage>(msg);
  apollo::canbus::Chassis chassis_msg;
  chassis_msg.ParseFromString(msg.content);
  chassis_msg_ = std::make_shared<apollo::canbus::Chassis>(chassis_msg);
  return true;
}

}  // namespace data
}  // namespace apollo
