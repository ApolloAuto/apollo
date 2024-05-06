/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/gem/proto/gem.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Shiftcmd65 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::Gem> {
 public:
  static const int32_t ID;

  Shiftcmd65();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description':
  // 'FORWARD_is_also_LOW_on_vehicles_with_LOW/HIGH,_PARK_and_HIGH_only_available_on_certain_Vehicles',
  // 'enum': {0: 'SHIFT_CMD_PARK', 1: 'SHIFT_CMD_REVERSE', 2:
  // 'SHIFT_CMD_NEUTRAL', 3: 'SHIFT_CMD_FORWARD', 4: 'SHIFT_CMD_LOW'},
  // 'precision': 1.0, 'len': 8, 'name': 'SHIFT_CMD', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Shiftcmd65* set_shift_cmd(Shift_cmd_65::Shift_cmdType shift_cmd);

 private:
  // config detail: {'description':
  // 'FORWARD_is_also_LOW_on_vehicles_with_LOW/HIGH,_PARK_and_HIGH_only_available_on_certain_Vehicles',
  // 'enum': {0: 'SHIFT_CMD_PARK', 1: 'SHIFT_CMD_REVERSE', 2:
  // 'SHIFT_CMD_NEUTRAL', 3: 'SHIFT_CMD_FORWARD', 4: 'SHIFT_CMD_LOW'},
  // 'precision': 1.0, 'len': 8, 'name': 'SHIFT_CMD', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_shift_cmd(uint8_t* data, Shift_cmd_65::Shift_cmdType shift_cmd);

 private:
  Shift_cmd_65::Shift_cmdType shift_cmd_;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
