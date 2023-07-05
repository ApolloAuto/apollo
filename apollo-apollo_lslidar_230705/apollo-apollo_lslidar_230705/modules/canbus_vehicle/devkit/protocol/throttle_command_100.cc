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

#include "modules/canbus_vehicle/devkit/protocol/throttle_command_100.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

const int32_t Throttlecommand100::ID = 0x100;

// public
Throttlecommand100::Throttlecommand100() { Reset(); }

uint32_t Throttlecommand100::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Throttlecommand100::Parse(const std::uint8_t* bytes, int32_t length,
                               Devkit* chassis) const {
  chassis->mutable_throttle_command_100()->set_throttle_en_ctrl(
      throttle_en_ctrl(bytes, length));
  chassis->mutable_throttle_command_100()->set_throttle_acc(
      throttle_acc(bytes, length));
  chassis->mutable_throttle_command_100()->set_throttle_pedal_target(
      throttle_pedal_target(bytes, length));
  chassis->mutable_throttle_command_100()->set_speed_target(
      speed_target(bytes, length));
  chassis->mutable_throttle_command_100()->set_checksum_100(
      checksum_100(bytes, length));
}

void Throttlecommand100::UpdateData(uint8_t* data) {
  set_p_speed_target(data, speed_target_);
  set_p_throttle_acc(data, throttle_acc_);
  set_p_throttle_pedal_target(data, throttle_pedal_target_);
  set_p_throttle_en_ctrl(data, throttle_en_ctrl_);
  checksum_100_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_100(data, checksum_100_);
}

void Throttlecommand100::Reset() {
  // TODO(All) :  you should check this manually
  throttle_en_ctrl_ = Throttle_command_100::THROTTLE_EN_CTRL_DISABLE;
  throttle_acc_ = 0.0;
  throttle_pedal_target_ = 0.0;
  speed_target_ = 0.0;
  checksum_100_ = 0;
}

Throttlecommand100* Throttlecommand100::set_throttle_en_ctrl(
    Throttle_command_100::Throttle_en_ctrlType throttle_en_ctrl) {
  throttle_en_ctrl_ = throttle_en_ctrl;
  return this;
}

// config detail: {'name': 'Throttle_EN_CTRL', 'enum': {0:
// 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'}, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Throttlecommand100::set_p_throttle_en_ctrl(
    uint8_t* data,
    Throttle_command_100::Throttle_en_ctrlType throttle_en_ctrl) {
  int x = throttle_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Throttlecommand100* Throttlecommand100::set_throttle_acc(double throttle_acc) {
  throttle_acc_ = throttle_acc;
  return this;
}

// config detail: {'name': 'Throttle_Acc', 'offset': 0.0, 'precision': 0.01,
// 'len': 10, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 15,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s^2'}
void Throttlecommand100::set_p_throttle_acc(uint8_t* data,
                                            double throttle_acc) {
  throttle_acc = ProtocolData::BoundedValue(0.0, 10.0, throttle_acc);
  int x = throttle_acc / 0.010000;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_throttle_pedal_target(
    double throttle_pedal_target) {
  throttle_pedal_target_ = throttle_pedal_target;
  return this;
}

// config detail: {'name': 'Throttle_Pedal_Target', 'offset': 0.0, 'precision':
// 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
// 31, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
void Throttlecommand100::set_p_throttle_pedal_target(
    uint8_t* data, double throttle_pedal_target) {
  throttle_pedal_target =
      ProtocolData::BoundedValue(0.0, 100.0, throttle_pedal_target);
  int x = throttle_pedal_target / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_speed_target(double speed_target) {
  speed_target_ = speed_target;
  return this;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 10, 'name':
// 'Speed_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|10.23]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
void Throttlecommand100::set_p_speed_target(uint8_t* data,
                                            double speed_target) {
  speed_target = ProtocolData::BoundedValue(0.0, 10.23, speed_target);
  int x = speed_target / 0.010000;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}

Throttlecommand100* Throttlecommand100::set_checksum_100(int checksum_100) {
  checksum_100_ = checksum_100;
  return this;
}

// config detail: {'name': 'CheckSum_100', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Throttlecommand100::set_p_checksum_100(uint8_t* data, int checksum_100) {
  checksum_100 = ProtocolData::BoundedValue(0, 255, checksum_100);
  int x = checksum_100;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Throttle_command_100::Throttle_en_ctrlType Throttlecommand100::throttle_en_ctrl(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Throttle_command_100::Throttle_en_ctrlType ret =
      static_cast<Throttle_command_100::Throttle_en_ctrlType>(x);
  return ret;
}

double Throttlecommand100::throttle_acc(const std::uint8_t* bytes,
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

double Throttlecommand100::throttle_pedal_target(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

double Throttlecommand100::speed_target(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(6, 2);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

int Throttlecommand100::checksum_100(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
