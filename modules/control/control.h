/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CONTROL_CONTROL_H_
#define MODULES_CONTROL_CONTROL_H_

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/util/util.h"
#include "modules/control/controller/controller_agent.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chasiss, and
 * pad data to compute throttle, brake and steer values.
 */
class Control : public apollo::common::ApolloApp {
  friend class ControlTestBase;

 public:
  Control() :
      monitor_logger_(apollo::common::monitor::MonitorMessageItem::CONTROL) {}

  /**
   * @brief module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~Control() = default;

 private:
  // Upon receiving pad message
  void OnPad(const apollo::control::PadMessage &pad);

  // Upon receiving monitor message
  void OnMonitor(
      const apollo::common::monitor::MonitorMessage &monitor_message);

  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  common::Status ProduceControlCommand(ControlCommand *control_command);
  common::Status CheckInput();
  common::Status CheckTimestamp();
  common::Status CheckPad();

  void SendCmd(ControlCommand *control_command);

 private:
  localization::LocalizationEstimate localization_;
  canbus::Chassis chassis_;
  planning::ADCTrajectory trajectory_;
  PadMessage pad_msg_;

  ControllerAgent controller_agent_;

  bool estop_ = false;
  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  ControlConf control_conf_;

  apollo::common::monitor::MonitorLogger monitor_logger_;
  ros::Timer timer_;
};

}  // namespace control
}  // namespace apollo

#endif  // MODULES_CONTROL_CONTROL_H_
