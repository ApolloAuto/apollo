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

#include "modules/canbus_vehicle/devkit/protocol/brake_command_101.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakecommand101::ID = 0x101;

// public
Brakecommand101::Brakecommand101() { Reset(); }

uint32_t Brakecommand101::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Brakecommand101::Parse(const std::uint8_t* bytes, int32_t length,
                               Devkit* chassis) const {
  chassis->mutable_brake_command_101()->set_brake_en_ctrl(
      brake_en_ctrl(bytes, length));
  chassis->mutable_brake_command_101()->set_aeb_en_ctrl(
      aeb_en_ctrl(bytes, length));
  chassis->mutable_brake_command_101()->set_brake_dec(
      brake_dec(bytes, length));
  chassis->mutable_brake_command_101()->set_brake_pedal_target(
      brake_pedal_target(bytes, length));
  chassis->mutable_brake_command_101()->set_checksum_101(
      checksum_101(bytes, length));
}

void Brakecommand101::UpdateData(uint8_t* data) {
  set_p_aeb_en_ctrl(data, aeb_en_ctrl_);
  set_p_brake_dec(data, brake_dec_);
  set_p_brake_pedal_target(data, brake_pedal_target_);
  set_p_brake_en_ctrl(data, brake_en_ctrl_);
  checksum_101_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_101(data, checksum_101_);
}

void Brakecommand101::Reset() {
  // TODO(All) :  you should check this manually
  aeb_en_ctrl_ = Brake_command_101::AEB_EN_CTRL_DISABLE_AEB;
  brake_dec_ = 0.0;
  checksum_101_ = 0;
  brake_pedal_target_ = 0.0;
  brake_en_ctrl_ = Brake_command_101::BRAKE_EN_CTRL_DISABLE;
}

Brakecommand101* Brakecommand101::set_brake_en_ctrl(
    Brake_command_101::Brake_en_ctrlType brake_en_ctrl) {
  brake_en_ctrl_ = brake_en_ctrl;
  return this;
}

// config detail: {'bit': 0, 'enum': {0: 'BRAKE_EN_CTRL_DISABLE', 1:
// 'BRAKE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
// 'Brake_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Brakecommand101::set_p_brake_en_ctrl(
    uint8_t* data, Brake_command_101::Brake_en_ctrlType brake_en_ctrl) {
  int x = brake_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Brakecommand101* Brakecommand101::set_aeb_en_ctrl(
    Brake_command_101::Aeb_en_ctrlType aeb_en_ctrl) {
  aeb_en_ctrl_ = aeb_en_ctrl;
  return this;
}

// config detail: {'bit': 1, 'enum': {0: 'AEB_EN_CTRL_DISABLE_AEB', 1:
// 'AEB_EN_CTRL_ENABLE_AEB'}, 'is_signed_var': False, 'len': 1, 'name':
// 'AEB_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Brakecommand101::set_p_aeb_en_ctrl(
    uint8_t* data, Brake_command_101::Aeb_en_ctrlType aeb_en_ctrl) {
  int x = aeb_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 1, 1);
}

Brakecommand101* Brakecommand101::set_brake_dec(double brake_dec) {
  brake_dec_ = brake_dec;
  return this;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name':
// 'Brake_Dec', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10]',
// 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
void Brakecommand101::set_p_brake_dec(uint8_t* data, double brake_dec) {
  brake_dec = ProtocolData::BoundedValue(0.0, 10.0, brake_dec);
  int x = brake_dec / 0.010000;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

Brakecommand101* Brakecommand101::set_brake_pedal_target(
    double brake_pedal_target) {
  brake_pedal_target_ = brake_pedal_target;
  return this;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name':
// 'Brake_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void Brakecommand101::set_p_brake_pedal_target(uint8_t* data,
                                               double brake_pedal_target) {
  brake_pedal_target =
      ProtocolData::BoundedValue(0.0, 100.0, brake_pedal_target);
  int x = brake_pedal_target / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Brakecommand101* Brakecommand101::set_checksum_101(int checksum_101) {
  checksum_101_ = checksum_101;
  return this;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
// 'CheckSum_101', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand101::set_p_checksum_101(uint8_t* data, int checksum_101) {
  checksum_101 = ProtocolData::BoundedValue(0, 255, checksum_101);
  int x = checksum_101;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Brake_command_101::Brake_en_ctrlType Brakecommand101::brake_en_ctrl(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Brake_command_101::Brake_en_ctrlType ret =
      static_cast<Brake_command_101::Brake_en_ctrlType>(x);
  return ret;
}

Brake_command_101::Aeb_en_ctrlType Brakecommand101::aeb_en_ctrl(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Brake_command_101::Aeb_en_ctrlType ret =
      static_cast<Brake_command_101::Aeb_en_ctrlType>(x);
  return ret;
}

double Brakecommand101::brake_dec(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(6, 2);
  x <<= 8;
  x |= t;

  double ret = x * 0.01;
  return ret;
}

double Brakecommand101::brake_pedal_target(const std::uint8_t* bytes,
                                           const int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.1;
  return ret;
}

int Brakecommand101::checksum_101(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
