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

#ifndef MODEULES_GUARDIAN_GUARDIAN_H_
#define MODEULES_GUARDIAN_GUARDIAN_H_

#include <map>
#include <mutex>
#include <queue>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::guardian
 * @brief apollo::guardian
 */
namespace apollo {
namespace guardian {

class Guardian : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  void OnTimer(const ros::TimerEvent&);
  void OnChassis(const apollo::canbus::Chassis& message);
  void OnControl(const apollo::control::ControlCommand& message);
  void OnSystemStatus(const apollo::monitor::SystemStatus& message);
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  apollo::canbus::Chassis chassis_;
  apollo::monitor::SystemStatus system_status_;
  apollo::control::ControlCommand control_cmd_;
  apollo::guardian::GuardianCommand guardian_cmd_;

  std::mutex mutex_;

  ros::Timer timer_;
};

}  // namespace guardian
}  // namespace apollo

#endif  // MODULES_GUARDIAN_GUARDIAN_H_
