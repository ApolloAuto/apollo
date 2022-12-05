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

#pragma once

#include "modules/canbus_vehicle/lexus/proto/lexus.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

class Globalrpt10 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::Lexus> {
 public:
  static const int32_t ID;
  Globalrpt10();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Lexus* chassis) const override;

 private:
  // config detail: {'name': 'CONFIG_FAULT_ACTIVE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 15, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool config_fault_active(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'name': 'PACMOD_SUBSYSTEM_TIMEOUT', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 5, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool pacmod_subsystem_timeout(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'PACMOD_SYSTEM_ENABLED', 'enum': {0:
  // 'PACMOD_SYSTEM_ENABLED_CONTROL_DISABLED', 1:
  // 'PACMOD_SYSTEM_ENABLED_CONTROL_ENABLED'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Global_rpt_10::Pacmod_system_enabledType pacmod_system_enabled(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'PACMOD_SYSTEM_OVERRIDE_ACTIVE', 'enum': {0:
  // 'PACMOD_SYSTEM_OVERRIDE_ACTIVE_NOT_OVERRIDDEN', 1:
  // 'PACMOD_SYSTEM_OVERRIDE_ACTIVE_OVERRIDDEN'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Global_rpt_10::Pacmod_system_override_activeType
  pacmod_system_override_active(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'name': 'PACMOD_SYSTEM_FAULT_ACTIVE', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 7, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool pacmod_system_fault_active(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'name': 'VEH_CAN_TIMEOUT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool veh_can_timeout(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'STR_CAN_TIMEOUT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool str_can_timeout(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BRK_CAN_TIMEOUT', 'enum': {0:
  // 'BRK_CAN_TIMEOUT_NO_ACTIVE_CAN_TIMEOUT', 1:
  // 'BRK_CAN_TIMEOUT_ACTIVE_CAN_TIMEOUT'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Global_rpt_10::Brk_can_timeoutType brk_can_timeout(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'USR_CAN_TIMEOUT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool usr_can_timeout(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'USR_CAN_READ_ERRORS', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|65535]', 'bit': 55, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': ''}
  int usr_can_read_errors(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
