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

/**
 * @file
 */

#ifndef MODEULES_GUARDIAN_GUARDIAN_COMPONENT_H_
#define MODEULES_GUARDIAN_GUARDIAN_COMPONENT_H_

#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "cybertron/common/macros.h"
#include "cybertron/component/timer_component.h"
#include "cybertron/cybertron.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/guardian/proto/guardian_conf.pb.h"
#include "modules/monitor/proto/system_status.pb.h"

/**
 * @namespace apollo::guardian
 * @brief apollo::guardian
 */
namespace apollo {
namespace guardian {

class GuardianComponent : public apollo::cybertron::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  apollo::guardian::GuardianConf guardian_conf_;
  apollo::canbus::Chassis chassis_;
  apollo::monitor::SystemStatus system_status_;
  apollo::control::ControlCommand control_cmd_;
  apollo::guardian::GuardianCommand guardian_cmd_;

  std::shared_ptr<apollo::cybertron::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  std::shared_ptr<apollo::cybertron::Reader<apollo::control::ControlCommand>>
      control_cmd_reader_;
  std::shared_ptr<apollo::cybertron::Reader<apollo::monitor::SystemStatus>>
      system_status_reader_;
  std::shared_ptr<apollo::cybertron::Writer<apollo::guardian::GuardianCommand>>
      guardian_writer_;

  std::mutex mutex_;
};

CYBERTRON_REGISTER_COMPONENT(GuardianComponent)

}  // namespace guardian
}  // namespace apollo

#endif  // MODEULES_GUARDIAN_GUARDIAN_COMPONENT_H_
