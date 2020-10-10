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

#include "modules/canbus/vehicle/lexus/protocol/global_rpt_10.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Globalrpt10::Globalrpt10() {}
const int32_t Globalrpt10::ID = 0x10;

void Globalrpt10::Parse(const std::uint8_t* bytes, int32_t length,
                        ChassisDetail* chassis) const {
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_config_fault_active(
      config_fault_active(bytes, length));
  chassis->mutable_lexus()
      ->mutable_global_rpt_10()
      ->set_pacmod_subsystem_timeout(pacmod_subsystem_timeout(bytes, length));
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_pacmod_system_enabled(
      pacmod_system_enabled(bytes, length));
  chassis->mutable_lexus()
      ->mutable_global_rpt_10()
      ->set_pacmod_system_override_active(
          pacmod_system_override_active(bytes, length));
  chassis->mutable_lexus()
      ->mutable_global_rpt_10()
      ->set_pacmod_system_fault_active(
          pacmod_system_fault_active(bytes, length));
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_veh_can_timeout(
      veh_can_timeout(bytes, length));
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_str_can_timeout(
      str_can_timeout(bytes, length));
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_brk_can_timeout(
      brk_can_timeout(bytes, length));
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_usr_can_timeout(
      usr_can_timeout(bytes, length));
  chassis->mutable_lexus()->mutable_global_rpt_10()->set_usr_can_read_errors(
      usr_can_read_errors(bytes, length));
}

// config detail: {'name': 'config_fault_active', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 15, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Globalrpt10::config_fault_active(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pacmod_subsystem_timeout', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 5, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Globalrpt10::pacmod_subsystem_timeout(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pacmod_system_enabled', 'enum': {0:
// 'PACMOD_SYSTEM_ENABLED_CONTROL_DISABLED', 1:
// 'PACMOD_SYSTEM_ENABLED_CONTROL_ENABLED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Global_rpt_10::Pacmod_system_enabledType Globalrpt10::pacmod_system_enabled(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Global_rpt_10::Pacmod_system_enabledType ret =
      static_cast<Global_rpt_10::Pacmod_system_enabledType>(x);
  return ret;
}

// config detail: {'name': 'pacmod_system_override_active', 'enum': {0:
// 'PACMOD_SYSTEM_OVERRIDE_ACTIVE_NOT_OVERRIDDEN', 1:
// 'PACMOD_SYSTEM_OVERRIDE_ACTIVE_OVERRIDDEN'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Global_rpt_10::Pacmod_system_override_activeType
Globalrpt10::pacmod_system_override_active(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Global_rpt_10::Pacmod_system_override_activeType ret =
      static_cast<Global_rpt_10::Pacmod_system_override_activeType>(x);
  return ret;
}

// config detail: {'name': 'pacmod_system_fault_active', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 7, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Globalrpt10::pacmod_system_fault_active(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'veh_can_timeout', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Globalrpt10::veh_can_timeout(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'str_can_timeout', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Globalrpt10::str_can_timeout(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'brk_can_timeout', 'enum': {0:
// 'BRK_CAN_TIMEOUT_NO_ACTIVE_CAN_TIMEOUT', 1:
// 'BRK_CAN_TIMEOUT_ACTIVE_CAN_TIMEOUT'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Global_rpt_10::Brk_can_timeoutType Globalrpt10::brk_can_timeout(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  Global_rpt_10::Brk_can_timeoutType ret =
      static_cast<Global_rpt_10::Brk_can_timeoutType>(x);
  return ret;
}

// config detail: {'name': 'usr_can_timeout', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Globalrpt10::usr_can_timeout(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'usr_can_read_errors', 'offset': 0.0,
// 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
// '[0|65535]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit':
// ''}
int Globalrpt10::usr_can_read_errors(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
