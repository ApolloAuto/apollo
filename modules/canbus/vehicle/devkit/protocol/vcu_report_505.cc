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
  chassis->mutable_devkit()->mutable_vcu_report_505()->set_battery_soc(
      battery_soc(bytes, length));
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

// config detail: {'name': 'battery_soc', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47,
// 'type': 'int', 'order': 'motorola', 'physical_unit': '%'}
int Vcureport505::battery_soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vehicle_mode_state', 'enum': {0:
// 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE',
// 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3:
// 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'precision': 1.0, 'len': 2,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 36,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vcu_report_505::Vehicle_mode_stateType Vcureport505::vehicle_mode_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 2);

  Vcu_report_505::Vehicle_mode_stateType ret =
      static_cast<Vcu_report_505::Vehicle_mode_stateType>(x);
  return ret;
}

// config detail: {'name': 'frontcrash_state', 'enum': {0:
// 'FRONTCRASH_STATE_NO_EVENT', 1: 'FRONTCRASH_STATE_CRASH_EVENT'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 33, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vcu_report_505::Frontcrash_stateType Vcureport505::frontcrash_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  Vcu_report_505::Frontcrash_stateType ret =
      static_cast<Vcu_report_505::Frontcrash_stateType>(x);
  return ret;
}

// config detail: {'name': 'backcrash_state', 'enum': {0:
// 'BACKCRASH_STATE_NO_EVENT', 1: 'BACKCRASH_STATE_CRASH_EVENT'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 34, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Vcu_report_505::Backcrash_stateType Vcureport505::backcrash_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Vcu_report_505::Backcrash_stateType ret =
      static_cast<Vcu_report_505::Backcrash_stateType>(x);
  return ret;
}

// config detail: {'name': 'aeb_state', 'enum': {0: 'AEB_STATE_INACTIVE', 1:
// 'AEB_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 32, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Vcu_report_505::Aeb_stateType Vcureport505::aeb_state(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Vcu_report_505::Aeb_stateType ret =
      static_cast<Vcu_report_505::Aeb_stateType>(x);
  return ret;
}

// config detail: {'name': 'acc', 'offset': 0.0, 'precision': 0.01, 'len': 12,
// 'is_signed_var': True, 'physical_range': '[-10|10]', 'bit': 7, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
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

// config detail: {'name': 'speed', 'offset': 0.0, 'precision': 0.001, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 23,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
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
