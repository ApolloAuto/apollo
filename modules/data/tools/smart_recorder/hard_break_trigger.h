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

struct HardBreakMessage {
  std::shared_ptr<RecordMessage> record_msg_;
  std::shared_ptr<canbus::Chassis> chassis_msg_;

  bool Initial(const RecordMessage& msg);
};

/**
 * @class HardBreakTrigger
 * @brief HardBreak trigger that fires when hard break is engaged
 */
class HardBreakTrigger: public TriggerBase {
 public:
  HardBreakTrigger();

  void Pull(const RecordMessage& msg) override;
  bool ShouldRestore(const RecordMessage& msg) const override { return false; };

  virtual ~HardBreakTrigger() = default;

 private:
  bool IsHardBreak(const std::shared_ptr<HardBreakMessage>& msg);
  bool IsNoisy(const std::shared_ptr<HardBreakMessage>& msg);
  float GetHistoryMeanSpeed();
  float GetMeanSpeed(const std::vector<std::shared_ptr<HardBreakMessage>>
    &msg_list);
  void PushToList(const std::shared_ptr<HardBreakMessage>& msg);
 private:
  Chassis::DrivingMode cur_driving_mode_ = Chassis::COMPLETE_MANUAL;
  std::vector<std::shared_ptr<HardBreakMessage>> his_msg_list_;
  std::vector<std::shared_ptr<HardBreakMessage>> cur_msg_list_;
};

}  // namespace data
}  // namespace apollo
