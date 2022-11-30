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

#include "modules/canbus_vehicle/devkit/protocol/park_command_104.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

const int32_t Parkcommand104::ID = 0x104;

// public
Parkcommand104::Parkcommand104() { Reset(); }

uint32_t Parkcommand104::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Parkcommand104::Parse(const std::uint8_t* bytes, int32_t length,
                           Devkit* chassis) const {
  chassis->mutable_park_command_104()->set_park_target(
      park_target(bytes, length));
  chassis->mutable_park_command_104()->set_park_en_ctrl(
      park_en_ctrl(bytes, length));
  chassis->mutable_park_command_104()->set_checksum_104(
      checksum_104(bytes, length));
}

void Parkcommand104::UpdateData(uint8_t* data) {
  set_p_park_target(data, park_target_);
  set_p_park_en_ctrl(data, park_en_ctrl_);
  checksum_104_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_104(data, checksum_104_);
}

void Parkcommand104::Reset() {
  // TODO(All) :  you should check this manually
  checksum_104_ = 0;
  park_target_ = Park_command_104::PARK_TARGET_RELEASE;
  park_en_ctrl_ = Park_command_104::PARK_EN_CTRL_DISABLE;
}

Parkcommand104* Parkcommand104::set_checksum_104(int checksum_104) {
  checksum_104_ = checksum_104;
  return this;
}

// config detail: {'name': 'CheckSum_104', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Parkcommand104::set_p_checksum_104(uint8_t* data, int checksum_104) {
  checksum_104 = ProtocolData::BoundedValue(0, 255, checksum_104);
  int x = checksum_104;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Parkcommand104* Parkcommand104::set_park_target(
    Park_command_104::Park_targetType park_target) {
  park_target_ = park_target;
  return this;
}

// config detail: {'name': 'Park_Target', 'enum': {0: 'PARK_TARGET_RELEASE', 1:
// 'PARK_TARGET_PARKING_TRIGGER'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
void Parkcommand104::set_p_park_target(
    uint8_t* data, Park_command_104::Park_targetType park_target) {
  int x = park_target;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}

Parkcommand104* Parkcommand104::set_park_en_ctrl(
    Park_command_104::Park_en_ctrlType park_en_ctrl) {
  park_en_ctrl_ = park_en_ctrl;
  return this;
}

// config detail: {'name': 'Park_EN_CTRL', 'enum': {0: 'PARK_EN_CTRL_DISABLE',
// 1: 'PARK_EN_CTRL_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
void Parkcommand104::set_p_park_en_ctrl(
    uint8_t* data, Park_command_104::Park_en_ctrlType park_en_ctrl) {
  int x = park_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Park_command_104::Park_targetType Parkcommand104::park_target(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Park_command_104::Park_targetType ret =
      static_cast<Park_command_104::Park_targetType>(x);
  return ret;
}

Park_command_104::Park_en_ctrlType Parkcommand104::park_en_ctrl(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Park_command_104::Park_en_ctrlType ret =
      static_cast<Park_command_104::Park_en_ctrlType>(x);
  return ret;
}

int Parkcommand104::checksum_104(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
