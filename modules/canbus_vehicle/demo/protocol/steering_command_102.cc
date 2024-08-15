/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/protocol/steering_command_102.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

const int32_t Steeringcommand102::ID = 0x102;

// public
Steeringcommand102::Steeringcommand102() { Reset(); }

uint32_t Steeringcommand102::GetPeriod() const {
  // TODO(All) : modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Steeringcommand102::Parse(const std::uint8_t* bytes, int32_t length,
                               Demo* chassis) const {
  chassis->mutable_steering_command_102()->set_heartbeat_102(
      heartbeat_102(bytes, length));
  chassis->mutable_steering_command_102()->set_steer_en_ctrl(
      steer_en_ctrl(bytes, length));
  chassis->mutable_steering_command_102()->set_steer_angle_target(
      steer_angle_target(bytes, length));
  chassis->mutable_steering_command_102()->set_steer_angle_spd_target(
      steer_angle_spd_target(bytes, length));
  chassis->mutable_steering_command_102()->set_checksum_102(
      checksum_102(bytes, length));
}

void Steeringcommand102::UpdateData_Heartbeat(uint8_t* data) {
  // TODO(All) : you should add the heartbeat manually
  ++heartbeat_102_;
  heartbeat_102_ = (heartbeat_102_) % 16;
  set_p_heartbeat_102(data, heartbeat_102_);
  checksum_102_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_102(data, checksum_102_);
}

void Steeringcommand102::UpdateData(uint8_t* data) {
  set_p_steer_en_ctrl(data, steer_en_ctrl_);
  set_p_steer_angle_target(data, steer_angle_target_);
  set_p_steer_angle_spd_target(data, steer_angle_spd_target_);
}

void Steeringcommand102::Reset() {
  // TODO(All) :  you should check this manually
  heartbeat_102_ = 0;
  steer_en_ctrl_ = Steering_command_102::STEER_EN_CTRL_DISABLE;
  steer_angle_target_ = 0;
  steer_angle_spd_target_ = 0;
  checksum_102_ = 0;
}

Steeringcommand102* Steeringcommand102::set_heartbeat_102(int heartbeat_102) {
  heartbeat_102_ = heartbeat_102;
  return this;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
// 'Heartbeat_102', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcommand102::set_p_heartbeat_102(uint8_t* data, int heartbeat_102) {
  heartbeat_102 = ProtocolData::BoundedValue(0, 15, heartbeat_102);
  int x = heartbeat_102;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Steeringcommand102* Steeringcommand102::set_steer_en_ctrl(
    Steering_command_102::Steer_en_ctrlType steer_en_ctrl) {
  steer_en_ctrl_ = steer_en_ctrl;
  return this;
}

// config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
// 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'is_signed_var': False,
// 'len': 1, 'name': 'Steer_EN_CTRL', 'offset': 0.0, 'order': 'motorola',
// 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'signal_type': 'enable',
// 'type': 'enum'}
void Steeringcommand102::set_p_steer_en_ctrl(
    uint8_t* data, Steering_command_102::Steer_en_ctrlType steer_en_ctrl) {
  int x = steer_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Steeringcommand102* Steeringcommand102::set_steer_angle_target(
    int steer_angle_target) {
  steer_angle_target_ = steer_angle_target;
  return this;
}

// config detail: {'bit': 31, 'description': 'command', 'is_signed_var': False,
// 'len': 16, 'name': 'Steer_ANGLE_Target', 'offset': -500.0, 'order':
// 'motorola', 'physical_range': '[-500|500]', 'physical_unit': 'deg',
// 'precision': 1.0, 'signal_type': 'command', 'type': 'int'}
void Steeringcommand102::set_p_steer_angle_target(uint8_t* data,
                                                  int steer_angle_target) {
  steer_angle_target =
      ProtocolData::BoundedValue(-500, 500, steer_angle_target);
  int x = (steer_angle_target - -500.000000);
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Steeringcommand102* Steeringcommand102::set_steer_angle_spd_target(
    int steer_angle_spd_target) {
  steer_angle_spd_target_ = steer_angle_spd_target;
  return this;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name':
// 'Steer_ANGLE_SPD_Target', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0,
// 'type': 'int'}
void Steeringcommand102::set_p_steer_angle_spd_target(
    uint8_t* data, int steer_angle_spd_target) {
  steer_angle_spd_target =
      ProtocolData::BoundedValue(0, 250, steer_angle_spd_target);
  int x = steer_angle_spd_target;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

Steeringcommand102* Steeringcommand102::set_checksum_102(int checksum_102) {
  checksum_102_ = checksum_102;
  return this;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
// 'CheckSum_102', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcommand102::set_p_checksum_102(uint8_t* data, int checksum_102) {
  checksum_102 = ProtocolData::BoundedValue(0, 255, checksum_102);
  int x = checksum_102;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

int Steeringcommand102::heartbeat_102(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

Steering_command_102::Steer_en_ctrlType Steeringcommand102::steer_en_ctrl(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Steering_command_102::Steer_en_ctrlType ret =
      static_cast<Steering_command_102::Steer_en_ctrlType>(x);
  return ret;
}

int Steeringcommand102::steer_angle_target(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -500.000000;
  return ret;
}

int Steeringcommand102::steer_angle_spd_target(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Steeringcommand102::checksum_102(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
