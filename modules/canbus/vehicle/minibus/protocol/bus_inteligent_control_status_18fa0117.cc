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

#include "modules/canbus/vehicle/minibus/protocol/bus_inteligent_control_status_18fa0117.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Businteligentcontrolstatus18fa0117::Businteligentcontrolstatus18fa0117() {}
const int32_t Businteligentcontrolstatus18fa0117::ID = 0x38fa0117;

void Businteligentcontrolstatus18fa0117::Parse(const std::uint8_t* bytes,
                                               int32_t length,
                                               ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_innerdoor_daysensor(bus_innerdoor_daysensor(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_outdoor_daysensor(bus_outdoor_daysensor(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_outdoor_key(bus_outdoor_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_innerdoor_key(bus_innerdoor_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_breaking_key(bus_breaking_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_reversing_key(bus_reversing_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_drive_key(bus_drive_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_joystick_right_key(bus_joystick_right_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_joystick_left_key(bus_joystick_left_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_joystick_behind_key(bus_joystick_behind_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_joystick_front_key(bus_joystick_front_key(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_parking_request_sw(bus_parking_request_sw(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_enginestar_sw(bus_enginestar_sw(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_disable_drivekeyboard(bus_disable_drivekeyboard(bytes, length));
  chassis->mutable_minibus()
      ->mutable_bus_inteligent_control_status_18fa0117()
      ->set_bus_emergency_sw(bus_emergency_sw(bytes, length));
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_innerdoor_daysensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_innerdoor_daysensor(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 26, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_outdoor_daysensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_outdoor_daysensor(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_outdoor_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_outdoor_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 22, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_innerdoor_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_innerdoor_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 20, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_breaking_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_breaking_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 18, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_reversing_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_reversing_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_drive_key', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_drive_key(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 14, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_joystick_right_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_joystick_right_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_joystick_left_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_joystick_left_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_joystick_behind_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_joystick_behind_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_joystick_front_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_joystick_front_key(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 6, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_parking_request_sw', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_parking_request_sw(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_enginestar_sw', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_enginestar_sw(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 2, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_disable_drivekeyboard', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Businteligentcontrolstatus18fa0117::bus_disable_drivekeyboard(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name':
// 'bus_emergency_sw', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Businteligentcontrolstatus18fa0117::bus_emergency_sw(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
