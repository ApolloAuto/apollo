/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/devkit/protocol/vcu_report_505.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Vcureport505::Vcureport505() {}
const int32_t Vcureport505::ID = 0x505;

void Vcureport505::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_vehicle_mode_state(
      vehicle_mode_state(bytes, length));
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_frontcrash_state(
      frontcrash_state(bytes, length));
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_backcrash_state(
      backcrash_state(bytes, length));
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_aeb_state(
      aeb_state(bytes, length));
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_acc(
      acc(bytes, length));
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_speed(
      speed(bytes, length));
}

// config detail: {'bit': 36, 'enum': {0:
// 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE',
// 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3:
// 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'is_signed_var': False, 'len': 2, 'name':
// 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Vehicle_mode_stateType Vcureport505::vehicle_mode_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 2);

  Vcu_report_505::Vehicle_mode_stateType ret =
      static_cast<Vcu_report_505::Vehicle_mode_stateType>(x);
  return ret;
}

// config detail: {'bit': 33, 'enum': {0: 'FRONTCRASH_STATE_NO_EVENT', 1:
// 'FRONTCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name':
// 'frontcrash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Frontcrash_stateType Vcureport505::frontcrash_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  Vcu_report_505::Frontcrash_stateType ret =
      static_cast<Vcu_report_505::Frontcrash_stateType>(x);
  return ret;
}

// config detail: {'bit': 34, 'enum': {0: 'BACKCRASH_STATE_NO_EVENT', 1:
// 'BACKCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name':
// 'backcrash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Backcrash_stateType Vcureport505::backcrash_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Vcu_report_505::Backcrash_stateType ret =
      static_cast<Vcu_report_505::Backcrash_stateType>(x);
  return ret;
}

// config detail: {'bit': 32, 'enum': {0: 'AEB_STATE_INACTIVE', 1:
// 'AEB_STATE_ACTIVE'}, 'is_signed_var': False, 'len': 1, 'name': 'aeb_state',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_report_505::Aeb_stateType Vcureport505::aeb_state(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Vcu_report_505::Aeb_stateType ret =
      static_cast<Vcu_report_505::Aeb_stateType>(x);
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'acc',
// 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]',
// 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
double Vcureport505::acc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name':
// 'speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65.535]',
// 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double Vcureport505::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
