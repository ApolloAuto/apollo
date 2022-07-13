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

#include "modules/canbus/vehicle/minibus/protocol/vcu_basic_message_18ffea97.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Vcubasicmessage18ffea97::Vcubasicmessage18ffea97() {}
const int32_t Vcubasicmessage18ffea97::ID = 0x38ffea97;

void Vcubasicmessage18ffea97::Parse(const std::uint8_t* bytes, int32_t length,
                                    ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_onebit(vcu_basic_onebit(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_hp_halt(vcu_basic_hp_halt(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_geatdefault_code(
          vcu_basic_geatdefault_code(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_sysdefault_level(
          vcu_basic_sysdefault_level(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_motocontroller_tempreture(
          vcu_basic_motocontroller_tempreture(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_motor_tempreture(
          vcu_basic_motor_tempreture(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_charge_status(vcu_basic_charge_status(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_real_highvoltage_sta(
          vcu_basic_real_highvoltage_sta(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_real_gear(vcu_basic_real_gear(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_basic_message_18ffea97()
      ->set_vcu_basic_motor_speed(vcu_basic_motor_speed(bytes, length));
}

// config detail: {'bit': 62, 'is_signed_var': False, 'len': 1, 'name':
// 'vcu_basic_onebit', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcubasicmessage18ffea97::vcu_basic_onebit(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name':
// 'vcu_basic_hp_halt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Vcubasicmessage18ffea97::vcu_basic_hp_halt(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': True, 'len': 2, 'name':
// 'vcu_basic_geatdefault_code', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Vcubasicmessage18ffea97::vcu_basic_geatdefault_code(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 2);

  x <<= 30;
  x >>= 30;

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 3, 'name':
// 'vcu_basic_sysdefault_level', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Vcubasicmessage18ffea97::vcu_basic_sysdefault_level(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': True, 'len': 8, 'name':
// 'vcu_basic_motocontroller_tempreture', 'offset': -40.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Vcubasicmessage18ffea97::vcu_basic_motocontroller_tempreture(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name':
// 'vcu_basic_motor_tempreture', 'offset': -40.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Vcubasicmessage18ffea97::vcu_basic_motor_tempreture(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 37, 'is_signed_var': False, 'len': 3, 'name':
// 'vcu_basic_charge_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcubasicmessage18ffea97::vcu_basic_charge_status(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 3, 'name':
// 'vcu_basic_real_highvoltage_sta', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Vcubasicmessage18ffea97::vcu_basic_real_highvoltage_sta(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 3);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'enum': {0: 'VCU_BASIC_REAL_GEAR_INVALID', 1:
// 'VCU_BASIC_REAL_GEAR_R', 2: 'VCU_BASIC_REAL_GEAR_N', 3:
// 'VCU_BASIC_REAL_GEAR_D'}, 'is_signed_var': False, 'len': 2, 'name':
// 'vcu_basic_real_gear', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Vcu_basic_message_18ffea97::Vcu_basic_real_gearType
Vcubasicmessage18ffea97::vcu_basic_real_gear(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Vcu_basic_message_18ffea97::Vcu_basic_real_gearType ret =
      static_cast<Vcu_basic_message_18ffea97::Vcu_basic_real_gearType>(x);
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'vcu_basic_motor_speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'rpm', 'precision': 0.5, 'type': 'double'}
double Vcubasicmessage18ffea97::vcu_basic_motor_speed(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.500000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
