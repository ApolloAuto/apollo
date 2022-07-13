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

#include "modules/canbus/vehicle/minibus/protocol/controller_pedal_cmd_18ff84a9.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

const int32_t Controllerpedalcmd18ff84a9::ID = 0x38ff84a9;

// public
Controllerpedalcmd18ff84a9::Controllerpedalcmd18ff84a9() { Reset(); }

uint32_t Controllerpedalcmd18ff84a9::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controllerpedalcmd18ff84a9::UpdateData_Heartbeat(uint8_t* data) {
  ++pedal_heartbeat_;
  pedal_heartbeat_ = pedal_heartbeat_ % 255;
  set_p_pedal_heartbeat(data, pedal_heartbeat_);
}

void Controllerpedalcmd18ff84a9::UpdateData(uint8_t* data) {
  set_p_pedal_check(data, pedal_check_);
  // set_p_pedal_heartbeat(data, pedal_heartbeat_);
  set_p_pedal_break(data, pedal_break_);
  set_p_brake_select(data, brake_select_);
  set_p_pedal_gear(data, pedal_gear_);
  set_p_pedal_throttle(data, pedal_throttle_);
  set_p_pedal_ctrl_request(data, pedal_ctrl_request_);
}

void Controllerpedalcmd18ff84a9::Reset() {
  // TODO(All) :  you should check this manually
  pedal_check_ = 0;
  pedal_heartbeat_ = 0;
  pedal_break_ = 0.0;
  brake_select_ = 0;
  pedal_gear_ = Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_N;
  pedal_throttle_ = 0.0;
  pedal_ctrl_request_ = Controller_pedal_cmd_18ff84a9::PEDAL_CTRL_REQUEST_OFF;
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_pedal_check(
    int pedal_check) {
  pedal_check_ = pedal_check;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'Pedal_Check', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerpedalcmd18ff84a9::set_p_pedal_check(uint8_t* data,
                                                   int pedal_check) {
  pedal_check = ProtocolData::BoundedValue(0, 0, pedal_check);
  int x = pedal_check;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_pedal_heartbeat(
    int pedal_heartbeat) {
  pedal_heartbeat_ = pedal_heartbeat;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'Pedal_HeartBeat', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerpedalcmd18ff84a9::set_p_pedal_heartbeat(uint8_t* data,
                                                       int pedal_heartbeat) {
  pedal_heartbeat = ProtocolData::BoundedValue(0, 255, pedal_heartbeat);
  int x = pedal_heartbeat;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_pedal_break(
    double pedal_break) {
  pedal_break_ = pedal_break;
  return this;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
// 'Pedal_Break', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|100]',
// 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
void Controllerpedalcmd18ff84a9::set_p_pedal_break(uint8_t* data,
                                                   double pedal_break) {
  pedal_break = ProtocolData::BoundedValue(0.0, 100.0, pedal_break);
  int x = pedal_break / 0.400000;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_brake_select(
    int brake_select) {
  brake_select_ = brake_select;
  return this;
}

// config detail: {'bit': 22, 'is_signed_var': False, 'len': 2, 'name':
// 'Brake_Select', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerpedalcmd18ff84a9::set_p_brake_select(uint8_t* data,
                                                    int brake_select) {
  brake_select = ProtocolData::BoundedValue(0, 0, brake_select);
  int x = brake_select;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_pedal_gear(
    Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear) {
  pedal_gear_ = pedal_gear;
  return this;
}

// config detail: {'bit': 16, 'enum': {0: 'PEDAL_GEAR_INVALID', 1:
// 'PEDAL_GEAR_R', 2: 'PEDAL_GEAR_N', 3: 'PEDAL_GEAR_D'}, 'is_signed_var':
// False, 'len': 4, 'name': 'Pedal_Gear', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
void Controllerpedalcmd18ff84a9::set_p_pedal_gear(
    uint8_t* data, Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear) {
  int x = pedal_gear;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 4);
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_pedal_throttle(
    double pedal_throttle) {
  pedal_throttle_ = pedal_throttle;
  return this;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'Pedal_Throttle', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
void Controllerpedalcmd18ff84a9::set_p_pedal_throttle(uint8_t* data,
                                                      double pedal_throttle) {
  pedal_throttle = ProtocolData::BoundedValue(0.0, 100.0, pedal_throttle);
  int x = pedal_throttle / 0.400000;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

Controllerpedalcmd18ff84a9* Controllerpedalcmd18ff84a9::set_pedal_ctrl_request(
    Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request) {
  pedal_ctrl_request_ = pedal_ctrl_request;
  return this;
}

// config detail: {'bit': 0, 'enum': {0: 'PEDAL_CTRL_REQUEST_OFF', 1:
// 'PEDAL_CTRL_REQUEST_ON', 2: 'PEDAL_CTRL_REQUEST_DEFAULT', 3:
// 'PEDAL_CTRL_REQUEST_INVALIT'}, 'is_signed_var': False, 'len': 2, 'name':
// 'Pedal_Ctrl_Request', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Controllerpedalcmd18ff84a9::set_p_pedal_ctrl_request(
    uint8_t* data,
    Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request) {
  int x = pedal_ctrl_request;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 2);
}

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
