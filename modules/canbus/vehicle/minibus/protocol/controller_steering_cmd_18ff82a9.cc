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

#include "modules/canbus/vehicle/minibus/protocol/controller_steering_cmd_18ff82a9.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

const int32_t Controllersteeringcmd18ff82a9::ID = 0x38ff82a9;

// public
Controllersteeringcmd18ff82a9::Controllersteeringcmd18ff82a9() { Reset(); }

uint32_t Controllersteeringcmd18ff82a9::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controllersteeringcmd18ff82a9::UpdateData_Heartbeat(uint8_t* data) {
  ++steering_heartbeat_;
  steering_heartbeat_ = steering_heartbeat_ % 255;
  set_p_steering_heartbeat(data, steering_heartbeat_);
}

void Controllersteeringcmd18ff82a9::UpdateData(uint8_t* data) {
  set_p_steering_vcu_status(data, steering_vcu_status_);
  // set_p_steering_heartbeat(data, steering_heartbeat_);
  set_p_steering_velocity(data, steering_velocity_);
  set_p_steering_ctrl_status(data, steering_ctrl_status_);
  set_p_streering_angle(data, streering_angle_);
}

void Controllersteeringcmd18ff82a9::Reset() {
  // TODO(All) :  you should check this manually
  steering_vcu_status_ = 3;
  steering_heartbeat_ = 0;
  steering_velocity_ = 0.0;
  steering_ctrl_status_ =
      Controller_steering_cmd_18ff82a9::STEERING_CTRL_STATUS_READY;
  streering_angle_ = 0.0;
}

Controllersteeringcmd18ff82a9*
Controllersteeringcmd18ff82a9::set_steering_vcu_status(
    int steering_vcu_status) {
  steering_vcu_status_ = steering_vcu_status;
  return this;
}

// config detail: {'bit': 20, 'is_signed_var': False, 'len': 4, 'name':
// 'Steering_VCU_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllersteeringcmd18ff82a9::set_p_steering_vcu_status(
    uint8_t* data, int steering_vcu_status) {
  steering_vcu_status = ProtocolData::BoundedValue(0, 6, steering_vcu_status);
  int x = steering_vcu_status;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 4);
}

Controllersteeringcmd18ff82a9*
Controllersteeringcmd18ff82a9::set_steering_heartbeat(int steering_heartbeat) {
  steering_heartbeat_ = steering_heartbeat;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'Steering_HeartBeat', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllersteeringcmd18ff82a9::set_p_steering_heartbeat(
    uint8_t* data, int steering_heartbeat) {
  steering_heartbeat = ProtocolData::BoundedValue(0, 255, steering_heartbeat);
  int x = steering_heartbeat;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Controllersteeringcmd18ff82a9*
Controllersteeringcmd18ff82a9::set_steering_velocity(double steering_velocity) {
  steering_velocity_ = steering_velocity;
  return this;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'Steering_Velocity', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|540]', 'physical_unit': '', 'precision': 10.0, 'type': 'double'}
void Controllersteeringcmd18ff82a9::set_p_steering_velocity(
    uint8_t* data, double steering_velocity) {
  steering_velocity = ProtocolData::BoundedValue(0.0, 540.0, steering_velocity);
  int x = steering_velocity / 10.000000;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}

Controllersteeringcmd18ff82a9*
Controllersteeringcmd18ff82a9::set_steering_ctrl_status(
    Controller_steering_cmd_18ff82a9::Steering_ctrl_statusType
        steering_ctrl_status) {
  steering_ctrl_status_ = steering_ctrl_status;
  return this;
}

// config detail: {'bit': 16, 'enum': {0: 'STEERING_CTRL_STATUS_READY', 1:
// 'STEERING_CTRL_STATUS_AUTO_DRIVE', 2: 'STEERING_CTRL_STATUS_SPEED_CMD', 3:
// 'STEERING_CTRL_STATUS_NULL', 4: 'STEERING_CTRL_STATUS_MANUAL_CMD', 5:
// 'STEERING_CTRL_STATUS_RESET_FROM_MAUNAL_INTERVENTION', 6:
// 'STEERING_CTRL_STATUS_CLEAR_DEFAULT'}, 'is_signed_var': False, 'len': 4,
// 'name': 'Steering_Ctrl_Status', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
void Controllersteeringcmd18ff82a9::set_p_steering_ctrl_status(
    uint8_t* data, Controller_steering_cmd_18ff82a9::Steering_ctrl_statusType
                       steering_ctrl_status) {
  int x = steering_ctrl_status;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 4);
}

Controllersteeringcmd18ff82a9*
Controllersteeringcmd18ff82a9::set_streering_angle(double streering_angle) {
  streering_angle_ = streering_angle;
  return this;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name':
// 'Streering_Angle', 'offset': -870.0, 'order': 'motorola', 'physical_range':
// '[-870|870]', 'physical_unit': 'degree', 'precision': 0.1, 'type': 'double'}
void Controllersteeringcmd18ff82a9::set_p_streering_angle(
    uint8_t* data, double streering_angle) {
  streering_angle = ProtocolData::BoundedValue(-870.0, 870.0, streering_angle);
  int x = (streering_angle - -870.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
