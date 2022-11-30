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

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Throttlestatus510 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Throttlestatus510();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'throttle pedal enable
  // bit(Status)', 'enum': {0: 'THROTTLE_PEDAL_EN_STS_DISABLE', 1:
  // 'THROTTLE_PEDAL_EN_STS_ENABLE', 2: 'THROTTLE_PEDAL_EN_STS_TAKEOVER'},
  // 'is_signed_var': False, 'len': 8, 'name': 'THROTTLE_PEDAL_EN_STS',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|2]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Throttle_status__510::Throttle_pedal_en_stsType throttle_pedal_en_sts(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'Percentage of throttle
  // pedal(Status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'THROTTLE_PEDAL_STS', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int throttle_pedal_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'enum': {0: 'DRIVE_MOTOR_ERR_NOERR', 1:
  // 'DRIVE_MOTOR_ERR_DRV_MOTOR_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'DRIVE_MOTOR_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Throttle_status__510::Drive_motor_errType drive_motor_err(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'enum': {0: 'BATTERY_BMS_ERR_NOERR', 1:
  // 'BATTERY_BMS_ERR_BATTERY_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'BATTERY_BMS_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Throttle_status__510::Battery_bms_errType battery_bms_err(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
