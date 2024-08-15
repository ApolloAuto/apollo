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

#include "modules/canbus_vehicle/demo/protocol/vehicle_mode_command_105.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

const int32_t Vehiclemodecommand105::ID = 0x105;

// public
Vehiclemodecommand105::Vehiclemodecommand105() { Reset(); }

uint32_t Vehiclemodecommand105::GetPeriod() const {
  // TODO(All) : modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Vehiclemodecommand105::Parse(const std::uint8_t* bytes, int32_t length,
                                  Demo* chassis) const {
  chassis->mutable_vehicle_mode_command_105()->set_heartbeat_105(
      heartbeat_105(bytes, length));
  chassis->mutable_vehicle_mode_command_105()->set_checksum_105(
      checksum_105(bytes, length));
  chassis->mutable_vehicle_mode_command_105()->set_turn_light_ctrl(
      turn_light_ctrl(bytes, length));
  chassis->mutable_vehicle_mode_command_105()->set_vin_req(
      vin_req(bytes, length));
  chassis->mutable_vehicle_mode_command_105()->set_drive_mode_ctrl(
      drive_mode_ctrl(bytes, length));
  chassis->mutable_vehicle_mode_command_105()->set_steer_mode_ctrl(
      steer_mode_ctrl(bytes, length));
}

void Vehiclemodecommand105::UpdateData_Heartbeat(uint8_t* data) {
  // TODO(All) : you should add the heartbeat manually
  ++heartbeat_105_;
  heartbeat_105_ = (heartbeat_105_) % 16;
  set_p_heartbeat_105(data, heartbeat_105_);
  checksum_105_ =
      data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
  set_p_checksum_105(data, checksum_105_);
}

void Vehiclemodecommand105::UpdateData(uint8_t* data) {
  set_p_turn_light_ctrl(data, turn_light_ctrl_);
  set_p_vin_req(data, vin_req_);
  set_p_drive_mode_ctrl(data, drive_mode_ctrl_);
  set_p_steer_mode_ctrl(data, steer_mode_ctrl_);
}

void Vehiclemodecommand105::Reset() {
  // TODO(All) :  you should check this manually
  heartbeat_105_ = 0;
  checksum_105_ = 0;
  turn_light_ctrl_ = Vehicle_mode_command_105::TURN_LIGHT_CTRL_TURNLAMP_OFF;
  vin_req_ = Vehicle_mode_command_105::VIN_REQ_VIN_REQ_DISABLE;
  drive_mode_ctrl_ =
      Vehicle_mode_command_105::DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE;
  steer_mode_ctrl_ = Vehicle_mode_command_105::STEER_MODE_CTRL_STANDARD_STEER;
}

Vehiclemodecommand105* Vehiclemodecommand105::set_heartbeat_105(
    int heartbeat_105) {
  heartbeat_105_ = heartbeat_105;
  return this;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 4, 'name':
// 'Heartbeat_105', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Vehiclemodecommand105::set_p_heartbeat_105(uint8_t* data,
                                                int heartbeat_105) {
  heartbeat_105 = ProtocolData::BoundedValue(0, 15, heartbeat_105);
  int x = heartbeat_105;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_checksum_105(
    int checksum_105) {
  checksum_105_ = checksum_105;
  return this;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
// 'CheckSum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Vehiclemodecommand105::set_p_checksum_105(uint8_t* data,
                                               int checksum_105) {
  checksum_105 = ProtocolData::BoundedValue(0, 255, checksum_105);
  int x = checksum_105;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_turn_light_ctrl(
    Vehicle_mode_command_105::Turn_light_ctrlType turn_light_ctrl) {
  turn_light_ctrl_ = turn_light_ctrl;
  return this;
}

// config detail: {'bit': 17, 'enum': {0: 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1:
// 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2: 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON',
// 3: 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False,
// 'len': 2, 'name': 'Turn_Light_CTRL', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
void Vehiclemodecommand105::set_p_turn_light_ctrl(
    uint8_t* data,
    Vehicle_mode_command_105::Turn_light_ctrlType turn_light_ctrl) {
  int x = turn_light_ctrl;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 2);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_vin_req(
    Vehicle_mode_command_105::Vin_reqType vin_req) {
  vin_req_ = vin_req;
  return this;
}

// config detail: {'bit': 24, 'enum': {0: 'VIN_REQ_VIN_REQ_DISABLE', 1:
// 'VIN_REQ_VIN_REQ_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name':
// 'VIN_Req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand105::set_p_vin_req(
    uint8_t* data, Vehicle_mode_command_105::Vin_reqType vin_req) {
  int x = vin_req;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 1);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_drive_mode_ctrl(
    Vehicle_mode_command_105::Drive_mode_ctrlType drive_mode_ctrl) {
  drive_mode_ctrl_ = drive_mode_ctrl;
  return this;
}

// config detail: {'bit': 10, 'enum': {0:
// 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'},
// 'is_signed_var': False, 'len': 3, 'name': 'Drive_Mode_CTRL', 'offset': 0.0,
// 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand105::set_p_drive_mode_ctrl(
    uint8_t* data,
    Vehicle_mode_command_105::Drive_mode_ctrlType drive_mode_ctrl) {
  int x = drive_mode_ctrl;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 3);
}

Vehiclemodecommand105* Vehiclemodecommand105::set_steer_mode_ctrl(
    Vehicle_mode_command_105::Steer_mode_ctrlType steer_mode_ctrl) {
  steer_mode_ctrl_ = steer_mode_ctrl;
  return this;
}

// config detail: {'bit': 2, 'enum': {0: 'STEER_MODE_CTRL_STANDARD_STEER', 1:
// 'STEER_MODE_CTRL_NON_DIRECTION_STEER', 2:
// 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'is_signed_var': False, 'len': 3,
// 'name': 'Steer_Mode_CTRL', 'offset': 0.0, 'order': 'motorola',
// 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
void Vehiclemodecommand105::set_p_steer_mode_ctrl(
    uint8_t* data,
    Vehicle_mode_command_105::Steer_mode_ctrlType steer_mode_ctrl) {
  int x = steer_mode_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 3);
}

int Vehiclemodecommand105::heartbeat_105(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

int Vehiclemodecommand105::checksum_105(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

Vehicle_mode_command_105::Turn_light_ctrlType
Vehiclemodecommand105::turn_light_ctrl(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Vehicle_mode_command_105::Turn_light_ctrlType ret =
      static_cast<Vehicle_mode_command_105::Turn_light_ctrlType>(x);
  return ret;
}

Vehicle_mode_command_105::Vin_reqType Vehiclemodecommand105::vin_req(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  Vehicle_mode_command_105::Vin_reqType ret =
      static_cast<Vehicle_mode_command_105::Vin_reqType>(x);
  return ret;
}

Vehicle_mode_command_105::Drive_mode_ctrlType
Vehiclemodecommand105::drive_mode_ctrl(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Vehicle_mode_command_105::Drive_mode_ctrlType ret =
      static_cast<Vehicle_mode_command_105::Drive_mode_ctrlType>(x);
  return ret;
}

Vehicle_mode_command_105::Steer_mode_ctrlType
Vehiclemodecommand105::steer_mode_ctrl(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  Vehicle_mode_command_105::Steer_mode_ctrlType ret =
      static_cast<Vehicle_mode_command_105::Steer_mode_ctrlType>(x);
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
