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

#include "modules/canbus_vehicle/ch/protocol/throttle_status__510.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Throttlestatus510::Throttlestatus510() {}
const int32_t Throttlestatus510::ID = 0x510;

void Throttlestatus510::Parse(const std::uint8_t* bytes, int32_t length,
                              Ch* chassis) const {
  chassis
      ->mutable_throttle_status__510()
      ->set_throttle_pedal_en_sts(throttle_pedal_en_sts(bytes, length));
  chassis->mutable_throttle_status__510()->set_throttle_pedal_sts(
      throttle_pedal_sts(bytes, length));
  chassis->mutable_throttle_status__510()->set_drive_motor_err(
      drive_motor_err(bytes, length));
  chassis->mutable_throttle_status__510()->set_battery_bms_err(
      battery_bms_err(bytes, length));
}

// config detail: {'description': 'throttle pedal enable bit(Status)', 'enum':
// {0: 'THROTTLE_PEDAL_EN_STS_DISABLE', 1: 'THROTTLE_PEDAL_EN_STS_ENABLE', 2:
// 'THROTTLE_PEDAL_EN_STS_TAKEOVER'}, 'precision': 1.0, 'len': 8, 'name':
// 'throttle_pedal_en_sts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Throttle_status__510::Throttle_pedal_en_stsType
Throttlestatus510::throttle_pedal_en_sts(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Throttle_status__510::Throttle_pedal_en_stsType ret =
      static_cast<Throttle_status__510::Throttle_pedal_en_stsType>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': 'Percentage of throttle
// pedal(Status)', 'is_signed_var': False, 'len': 8, 'name':
// 'throttle_pedal_sts', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int Throttlestatus510::throttle_pedal_sts(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'enum': {0: 'DRIVE_MOTOR_ERR_NOERR', 1:
// 'DRIVE_MOTOR_ERR_DRV_MOTOR_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
// 'drive_motor_err', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Throttle_status__510::Drive_motor_errType Throttlestatus510::drive_motor_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Throttle_status__510::Drive_motor_errType ret =
      static_cast<Throttle_status__510::Drive_motor_errType>(x);
  return ret;
}

// config detail: {'bit': 24, 'enum': {0: 'BATTERY_BMS_ERR_NOERR', 1:
// 'BATTERY_BMS_ERR_BATTERY_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
// 'battery_bms_err', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Throttle_status__510::Battery_bms_errType Throttlestatus510::battery_bms_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Throttle_status__510::Battery_bms_errType ret =
      static_cast<Throttle_status__510::Battery_bms_errType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
