/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/data/tools/smart_recorder/bumper_crash_trigger.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace data {

using apollo::canbus::Chassis;

BumperCrashTrigger::BumperCrashTrigger()
    : previous_check_event_trigger_(false) {
  trigger_name_ = "BumperCrashTrigger";
}

void BumperCrashTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }
  if (msg.channel_name == FLAGS_chassis_topic) {
    apollo::canbus::Chassis chassis_msg;
    chassis_msg.ParseFromString(msg.content);
    bool is_front_bumper_trigger = false;
    bool is_back_bumper_trigger = false;

    if (chassis_msg.has_front_bumper_event()) {
      if (chassis_msg.front_bumper_event() == canbus::Chassis::BUMPER_PRESSED) {
        is_front_bumper_trigger = true;
      }
    }
    if (chassis_msg.has_back_bumper_event()) {
      if (chassis_msg.back_bumper_event() == canbus::Chassis::BUMPER_PRESSED) {
        is_back_bumper_trigger = true;
      }
    }
    bool check_event_trigger
        = is_front_bumper_trigger || is_back_bumper_trigger;

    if (check_event_trigger && (!previous_check_event_trigger_)) {
      AINFO << "Chassis has crash event.";
      AINFO << "crash bumper trigger is pulled: " << msg.time << " - "
            << msg.channel_name;
      TriggerIt(msg.time);
    }
    previous_check_event_trigger_ = check_event_trigger;
  }
}

}  // namespace data
}  // namespace apollo
