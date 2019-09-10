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

#pragma once

#include <vector>
#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"
#include "modules/data/tools/smart_recorder/trigger_base.h"

namespace apollo {
namespace data {

using apollo::canbus::Chassis;

struct EmergencyMessage {
  std::shared_ptr<RecordMessage> record_msg_;
  std::shared_ptr<canbus::Chassis> chassis_msg_;

  bool Initial(const RecordMessage& msg);
};

/**
 * @class EmergencyModeTrigger
 * @brief EmergencyMode trigger that fires when emergency mode is engaged
 */
class EmergencyModeTriggerBeta: public TriggerBase {
 public:
  EmergencyModeTriggerBeta();

  void Pull(const RecordMessage& msg) override;
  bool ShouldRestore(const RecordMessage& msg) const override { return false; };

  virtual ~EmergencyModeTriggerBeta() = default;

 private:
  bool IsEmergency(const std::shared_ptr<EmergencyMessage>& msg);
  float GetHistoryMeanSpeed();
  void PushToHistoryList(const std::shared_ptr<EmergencyMessage>& msg);
 private:
  Chassis::DrivingMode cur_driving_mode_ = Chassis::COMPLETE_MANUAL;
  std::vector<std::shared_ptr<EmergencyMessage>> msg_list_;
};

}  // namespace data
}  // namespace apollo
