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

#include "modules/canbus/vehicle/ch/protocol/steer_status__512.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Steerstatus512::Steerstatus512() {}
const int32_t Steerstatus512::ID = 0x512;

void Steerstatus512::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_steer_status__512()->set_steer_angle_en_sts(
      steer_angle_en_sts(bytes, length));
  chassis->mutable_ch()->mutable_steer_status__512()->set_steer_angle_sts(
      steer_angle_sts(bytes, length));
  chassis->mutable_ch()->mutable_steer_status__512()->set_steer_err(
      steer_err(bytes, length));
  chassis->mutable_ch()->mutable_steer_status__512()->set_sensor_err(
      sensor_err(bytes, length));
  chassis->mutable_check_response()->set_is_eps_online(
      steer_angle_en_sts(bytes, length) == 1);
}

// config detail: {'description': 'steering angle enable bit(Status)', 'enum':
// {0: 'STEER_ANGLE_EN_STS_DISABLE', 1: 'STEER_ANGLE_EN_STS_ENABLE', 2:
// 'STEER_ANGLE_EN_STS_TAKEOVER'}, 'precision': 1.0, 'len': 8, 'name':
// 'steer_angle_en_sts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Steer_status__512::Steer_angle_en_stsType Steerstatus512::steer_angle_en_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Steer_status__512::Steer_angle_en_stsType ret =
      static_cast<Steer_status__512::Steer_angle_en_stsType>(x);
  return ret;
}

// config detail: {'description': 'Current steering angle(Status)', 'offset':
// 0.0, 'precision': 0.001, 'len': 16, 'name': 'steer_angle_sts',
// 'is_signed_var': True, 'physical_range': '[-0.524|0.524]', 'bit': 8, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'radian'}
double Steerstatus512::steer_angle_sts(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'steer_err', 'enum': {0: 'STEER_ERR_NOERR', 1:
// 'STEER_ERR_STEER_MOTOR_ERR'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
Steer_status__512::Steer_errType Steerstatus512::steer_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Steer_status__512::Steer_errType ret =
      static_cast<Steer_status__512::Steer_errType>(x);
  return ret;
}

// config detail: {'name': 'sensor_err', 'enum': {0: 'SENSOR_ERR_NOERR', 1:
// 'SENSOR_ERR_STEER_SENSOR_ERR'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
Steer_status__512::Sensor_errType Steerstatus512::sensor_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Steer_status__512::Sensor_errType ret =
      static_cast<Steer_status__512::Sensor_errType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
