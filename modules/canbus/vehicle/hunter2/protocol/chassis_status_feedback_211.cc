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

#include "modules/canbus/vehicle/hunter2/protocol/chassis_status_feedback_211.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Chassisstatusfeedback211::Chassisstatusfeedback211() {}
const int32_t Chassisstatusfeedback211::ID = 0x211;

void Chassisstatusfeedback211::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_chassis_status_feedback_211()->set_count(count(bytes, length));
  chassis->mutable_hunter2()->mutable_chassis_status_feedback_211()->set_brake_status(brake_status(bytes, length));
  chassis->mutable_hunter2()->mutable_chassis_status_feedback_211()->set_fault_message(fault_message(bytes, length));
  chassis->mutable_hunter2()->mutable_chassis_status_feedback_211()->set_battery_voltage(battery_voltage(bytes, length));
  chassis->mutable_hunter2()->mutable_chassis_status_feedback_211()->set_control_mode(control_mode(bytes, length));
  chassis->mutable_hunter2()->mutable_chassis_status_feedback_211()->set_current_vehicle_status(current_vehicle_status(bytes, length));
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'count', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassisstatusfeedback211::count(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 55, 'enum': {0: 'BRAKE_STATUS_BRAKE_RELEASE', 1: 'BRAKE_STATUS_BRAKE_LOCK'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_feedback_211::Brake_statusType Chassisstatusfeedback211::brake_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_feedback_211::Brake_statusType ret =  static_cast<Chassis_status_feedback_211::Brake_statusType>(x);
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name': 'fault_message', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassisstatusfeedback211::fault_message(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|6553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Chassisstatusfeedback211::battery_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 15, 'enum': {0: 'CONTROL_MODE_STANDBY', 1: 'CONTROL_MODE_CAN_COMMAND_CONTROL', 2: 'CONTROL_MODE_REMOTE_CONTROL'}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_feedback_211::Control_modeType Chassisstatusfeedback211::control_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_feedback_211::Control_modeType ret =  static_cast<Chassis_status_feedback_211::Control_modeType>(x);
  return ret;
}

// config detail: {'bit': 7, 'enum': {0: 'CURRENT_VEHICLE_STATUS_SYSTEM_OK', 1: 'CURRENT_VEHICLE_STATUS_EMERGENCY_SHUTDOWND_MODE', 2: 'CURRENT_VEHICLE_STATUS_SYSTEM_EXCEPTION'}, 'is_signed_var': False, 'len': 8, 'name': 'current_vehicle_status', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_feedback_211::Current_vehicle_statusType Chassisstatusfeedback211::current_vehicle_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_feedback_211::Current_vehicle_statusType ret =  static_cast<Chassis_status_feedback_211::Current_vehicle_statusType>(x);
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
