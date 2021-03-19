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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Globalrpt6a : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Globalrpt6a();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'PACMOD_STATUS', 'enum': {0:
  // 'PACMOD_STATUS_CONTROL_DISABLED', 1: 'PACMOD_STATUS_CONTROL_ENABLED'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Global_rpt_6a::Pacmod_statusType pacmod_status(const std::uint8_t* bytes,
                                                 const int32_t length) const;

  // config detail: {'name': 'OVERRIDE_STATUS', 'enum': {0:
  // 'OVERRIDE_STATUS_NOT_OVERRIDDEN', 1: 'OVERRIDE_STATUS_OVERRIDDEN'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Global_rpt_6a::Override_statusType override_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VEH_CAN_TIMEOUT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
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
  Global_rpt_6a::Brk_can_timeoutType brk_can_timeout(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'USR_CAN_TIMEOUT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool usr_can_timeout(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'USR_CAN_READ_ERRORS', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|65535]', 'bit': 55, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': ''}
  int usr_can_read_errors(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
