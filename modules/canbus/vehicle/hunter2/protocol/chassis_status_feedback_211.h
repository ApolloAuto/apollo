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

class Chassisstatusfeedback211 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Chassisstatusfeedback211();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'count', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int count(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 55, 'enum': {0: 'BRAKE_STATUS_BRAKE_RELEASE', 1: 'BRAKE_STATUS_BRAKE_LOCK'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Chassis_status_feedback_211::Brake_statusType brake_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name': 'fault_message', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fault_message(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double battery_voltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 15, 'enum': {0: 'CONTROL_MODE_STANDBY', 1: 'CONTROL_MODE_CAN_COMMAND_CONTROL', 2: 'CONTROL_MODE_REMOTE_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Chassis_status_feedback_211::Control_modeType control_mode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 7, 'enum': {0: 'CURRENT_VEHICLE_STATUS_SYSTEM_OK', 1: 'CURRENT_VEHICLE_STATUS_EMERGENCY_SHUTDOWND_MODE', 2: 'CURRENT_VEHICLE_STATUS_SYSTEM_EXCEPTION'}, 'is_signed_var': False, 'len': 8, 'name': 'current_vehicle_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Chassis_status_feedback_211::Current_vehicle_statusType current_vehicle_status(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo


