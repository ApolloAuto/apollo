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

#include "modules/canbus_vehicle/demo/protocol/gear_command_103.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

const int32_t Gearcommand103::ID = 0x103;

// public
Gearcommand103::Gearcommand103() { Reset(); }

uint32_t Gearcommand103::GetPeriod() const {
  // TODO(All) : modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Gearcommand103::Parse(const std::uint8_t* bytes, int32_t length,
                           Demo* chassis) const {
  chassis->mutable_gear_command_103()->set_heartbeat_103(
      heartbeat_103(bytes, length));
  chassis->mutable_gear_command_103()->set_gear_target(
      gear_target(bytes, length));
  chassis->mutable_gear_command_103()->set_gear_en_ctrl(
      gear_en_ctrl(bytes, length));
  chassis->mutable_gear_command_103()->set_checksum_103(
      checksum_103(bytes, length));
}

void Gearcommand103::UpdateData_Heartbeat(uint8_t* data) {
  // TODO(All) : you should add the heartbeat manually
  ++heartbeat_103_;
  heartbeat_103_ = (heartbeat_103_) % 16;
  set_p_heartbeat_103(data, heartbeat_103_);
  checksum_103_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_103(data, checksum_103_);
}

void Gearcommand103::UpdateData(uint8_t* data) {
  set_p_gear_target(data, gear_target_);
  set_p_gear_en_ctrl(data, gear_en_ctrl_);
}

void Gearcommand103::Reset() {
  // TODO(All) :  you should check this manually
  heartbeat_103_ = 0;
  gear_target_ = Gear_command_103::GEAR_TARGET_INVALID;
  gear_en_ctrl_ = Gear_command_103::GEAR_EN_CTRL_DISABLE;
  checksum_103_ = 0;
}

Gearcommand103* Gearcommand103::set_heartbeat_103(int heartbeat_103) {
  heartbeat_103_ = heartbeat_103;
  return this;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
// 'Heartbeat_103', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Gearcommand103::set_p_heartbeat_103(uint8_t* data, int heartbeat_103) {
  heartbeat_103 = ProtocolData::BoundedValue(0, 15, heartbeat_103);
  int x = heartbeat_103;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Gearcommand103* Gearcommand103::set_gear_target(
    Gear_command_103::Gear_targetType gear_target) {
  gear_target_ = gear_target;
  return this;
}

// config detail: {'bit': 10, 'description': 'command', 'enum': {0:
// 'GEAR_TARGET_INVALID', 1: 'GEAR_TARGET_PARK', 2: 'GEAR_TARGET_REVERSE', 3:
// 'GEAR_TARGET_NEUTRAL', 4: 'GEAR_TARGET_DRIVE'}, 'is_signed_var': False,
// 'len': 3, 'name': 'Gear_Target', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|4]', 'physical_unit': '', 'precision': 1.0,
// 'signal_type': 'command', 'type': 'enum'}
void Gearcommand103::set_p_gear_target(
    uint8_t* data, Gear_command_103::Gear_targetType gear_target) {
  int x = gear_target;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 3);
}

Gearcommand103* Gearcommand103::set_gear_en_ctrl(
    Gear_command_103::Gear_en_ctrlType gear_en_ctrl) {
  gear_en_ctrl_ = gear_en_ctrl;
  return this;
}

// config detail: {'bit': 0, 'description': 'enable', 'enum': {0:
// 'GEAR_EN_CTRL_DISABLE', 1: 'GEAR_EN_CTRL_ENABLE'}, 'is_signed_var': False,
// 'len': 1, 'name': 'Gear_EN_CTRL', 'offset': 0.0, 'order': 'motorola',
// 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'signal_type': 'enable',
// 'type': 'enum'}
void Gearcommand103::set_p_gear_en_ctrl(
    uint8_t* data, Gear_command_103::Gear_en_ctrlType gear_en_ctrl) {
  int x = gear_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Gearcommand103* Gearcommand103::set_checksum_103(int checksum_103) {
  checksum_103_ = checksum_103;
  return this;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
// 'CheckSum_103', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Gearcommand103::set_p_checksum_103(uint8_t* data, int checksum_103) {
  checksum_103 = ProtocolData::BoundedValue(0, 255, checksum_103);
  int x = checksum_103;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

int Gearcommand103::heartbeat_103(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

Gear_command_103::Gear_targetType Gearcommand103::gear_target(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Gear_command_103::Gear_targetType ret =
      static_cast<Gear_command_103::Gear_targetType>(x);
  return ret;
}

Gear_command_103::Gear_en_ctrlType Gearcommand103::gear_en_ctrl(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Gear_command_103::Gear_en_ctrlType ret =
      static_cast<Gear_command_103::Gear_en_ctrlType>(x);
  return ret;
}

int Gearcommand103::checksum_103(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
