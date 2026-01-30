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

/**
 * @file
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/guardian_msgs/guardian.pb.h"

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class Canbus
 *
 * @brief canbus module main class.
 * It processes the control data to send protocol messages to can card.
 */
class CanbusComponent final : public apollo::cyber::TimerComponent {
 public:
  CanbusComponent();
  /**
   * @brief obtain module name
   * @return module name
   */
  std::string Name() const;

 private:
  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init() override;

  /**
   * @brief module on_time function
   */
  bool Proc() override;

  /**
   * @brief module cleanup function
   */
  void Clear() override;

  void PublishChassis();
  void OnControlCommand(const apollo::control::ControlCommand &control_command);
  void OnControlCommandCheck(
      const apollo::control::ControlCommand &control_command);
  void OnChassisCommand(
      const apollo::external_command::ChassisCommand &chassis_command);
  void OnGuardianCommand(
      const apollo::guardian::GuardianCommand &guardian_command);
  void OnGuardianCommandCheck(
      const apollo::guardian::GuardianCommand &guardian_command);
  apollo::common::Status OnError(const std::string &error_msg);
  void ProcessTimeoutByClearCanSender();
  void ProcessGuardianCmdTimeout(
      apollo::guardian::GuardianCommand *guardian_command);

  CanbusConf canbus_conf_;
  std::shared_ptr<::apollo::canbus::AbstractVehicleFactory> vehicle_object_ =
      nullptr;
  std::shared_ptr<cyber::Reader<apollo::guardian::GuardianCommand>>
      guardian_cmd_reader_;
  std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>>
      control_command_reader_;
  std::shared_ptr<cyber::Reader<apollo::external_command::ChassisCommand>>
      chassis_command_reader_;
  double last_timestamp_controlcmd_ = 0.0;
  int64_t last_timestamp_chassiscmd_ = 0;
  ::apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  std::shared_ptr<cyber::Writer<Chassis>> chassis_writer_;
  bool is_control_cmd_time_delay_ = false;
  bool is_control_cmd_time_delay_previous_ = false;
  bool is_chassis_communication_fault_ = false;
};

CYBER_REGISTER_COMPONENT(CanbusComponent)

}  // namespace canbus
}  // namespace apollo
