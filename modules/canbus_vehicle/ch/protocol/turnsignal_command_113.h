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

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Turnsignalcommand113 : public ::apollo::drivers::canbus::ProtocolData<
                                 ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;

  Turnsignalcommand113();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 8, 'description': 'Lighting control(Command)',
  // 'enum': {0: 'LOW_BEAM_CMD_OFF', 1: 'LOW_BEAM_CMD_ON'}, 'is_signed_var':
  // False, 'len': 2, 'name': 'LOW_BEAM_CMD', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Turnsignalcommand113* set_low_beam_cmd(
      Turnsignal_command_113::Low_beam_cmdType low_beam_cmd);

  // config detail: {'bit': 0, 'description': 'Lighting control(Command)',
  // 'enum': {0: 'TURN_SIGNAL_CMD_NONE', 1: 'TURN_SIGNAL_CMD_LEFT', 2:
  // 'TURN_SIGNAL_CMD_RIGHT', 3: 'TURN_SIGNAL_CMD_HAZARD_WARNING_LAMPSTS'},
  // 'is_signed_var': False, 'len': 8, 'name': 'TURN_SIGNAL_CMD', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Turnsignalcommand113* set_turn_signal_cmd(
      Turnsignal_command_113::Turn_signal_cmdType turn_signal_cmd);

 private:
  // config detail: {'bit': 8, 'description': 'Lighting control(Command)',
  // 'enum': {0: 'LOW_BEAM_CMD_OFF', 1: 'LOW_BEAM_CMD_ON'}, 'is_signed_var':
  // False, 'len': 2, 'name': 'LOW_BEAM_CMD', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  void set_p_low_beam_cmd(
      uint8_t* data, Turnsignal_command_113::Low_beam_cmdType low_beam_cmd);

  // config detail: {'bit': 0, 'description': 'Lighting control(Command)',
  // 'enum': {0: 'TURN_SIGNAL_CMD_NONE', 1: 'TURN_SIGNAL_CMD_LEFT', 2:
  // 'TURN_SIGNAL_CMD_RIGHT', 3: 'TURN_SIGNAL_CMD_HAZARD_WARNING_LAMPSTS'},
  // 'is_signed_var': False, 'len': 8, 'name': 'TURN_SIGNAL_CMD', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  void set_p_turn_signal_cmd(
      uint8_t* data,
      Turnsignal_command_113::Turn_signal_cmdType turn_signal_cmd);

 private:
  Turnsignal_command_113::Turn_signal_cmdType turn_signal_cmd_;
  Turnsignal_command_113::Low_beam_cmdType low_beam_cmd_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
