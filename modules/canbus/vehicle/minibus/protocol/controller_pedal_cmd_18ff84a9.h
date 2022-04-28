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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Controllerpedalcmd18ff84a9
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controllerpedalcmd18ff84a9();

  uint32_t GetPeriod() const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_Check', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Controllerpedalcmd18ff84a9* set_pedal_check(int pedal_check);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_HeartBeat', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Controllerpedalcmd18ff84a9* set_pedal_heartbeat(int pedal_heartbeat);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_Break', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
  Controllerpedalcmd18ff84a9* set_pedal_break(double pedal_break);

  // config detail: {'bit': 22, 'is_signed_var': False, 'len': 2, 'name':
  // 'Brake_Select', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Controllerpedalcmd18ff84a9* set_brake_select(int brake_select);

  // config detail: {'bit': 16, 'enum': {0: 'PEDAL_GEAR_INVALID', 1:
  // 'PEDAL_GEAR_R', 2: 'PEDAL_GEAR_N', 3: 'PEDAL_GEAR_D'}, 'is_signed_var':
  // False, 'len': 4, 'name': 'Pedal_Gear', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Controllerpedalcmd18ff84a9* set_pedal_gear(
      Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_Throttle', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
  Controllerpedalcmd18ff84a9* set_pedal_throttle(double pedal_throttle);

  // config detail: {'bit': 0, 'enum': {0: 'PEDAL_CTRL_REQUEST_OFF', 1:
  // 'PEDAL_CTRL_REQUEST_ON', 2: 'PEDAL_CTRL_REQUEST_DEFAULT', 3:
  // 'PEDAL_CTRL_REQUEST_INVALIT'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'Pedal_Ctrl_Request', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Controllerpedalcmd18ff84a9* set_pedal_ctrl_request(
      Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request);

 private:
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_Check', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_pedal_check(uint8_t* data, int pedal_check);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_HeartBeat', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_pedal_heartbeat(uint8_t* data, int pedal_heartbeat);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_Break', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
  void set_p_pedal_break(uint8_t* data, double pedal_break);

  // config detail: {'bit': 22, 'is_signed_var': False, 'len': 2, 'name':
  // 'Brake_Select', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_brake_select(uint8_t* data, int brake_select);

  // config detail: {'bit': 16, 'enum': {0: 'PEDAL_GEAR_INVALID', 1:
  // 'PEDAL_GEAR_R', 2: 'PEDAL_GEAR_N', 3: 'PEDAL_GEAR_D'}, 'is_signed_var':
  // False, 'len': 4, 'name': 'Pedal_Gear', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  void set_p_pedal_gear(
      uint8_t* data, Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'Pedal_Throttle', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
  void set_p_pedal_throttle(uint8_t* data, double pedal_throttle);

  // config detail: {'bit': 0, 'enum': {0: 'PEDAL_CTRL_REQUEST_OFF', 1:
  // 'PEDAL_CTRL_REQUEST_ON', 2: 'PEDAL_CTRL_REQUEST_DEFAULT', 3:
  // 'PEDAL_CTRL_REQUEST_INVALIT'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'Pedal_Ctrl_Request', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_pedal_ctrl_request(
      uint8_t* data,
      Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request);

 private:
  int pedal_check_;
  int pedal_heartbeat_;
  double pedal_break_;
  int brake_select_;
  Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear_;
  double pedal_throttle_;
  Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request_;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
