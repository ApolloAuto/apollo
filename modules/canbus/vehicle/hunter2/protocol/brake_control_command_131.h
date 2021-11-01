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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

class Brakecontrolcommand131 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Brakecontrolcommand131();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'enum': {0: 'BRAKE_CONTROL_COMMAND_BRAKE_RELEASE', 1: 'BRAKE_CONTROL_COMMAND_BRAKE_LOCK'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_control_command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Brakecontrolcommand131* set_brake_control_command(Brake_control_command_131::Brake_control_commandType brake_control_command);

 private:

  // config detail: {'bit': 7, 'enum': {0: 'BRAKE_CONTROL_COMMAND_BRAKE_RELEASE', 1: 'BRAKE_CONTROL_COMMAND_BRAKE_LOCK'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_control_command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_brake_control_command(uint8_t* data, Brake_control_command_131::Brake_control_commandType brake_control_command);

 private:
  Brake_control_command_131::Brake_control_commandType brake_control_command_;
};

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo


