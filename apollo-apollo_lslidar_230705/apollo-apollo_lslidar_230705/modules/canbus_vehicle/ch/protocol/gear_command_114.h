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

class Gearcommand114 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;

  Gearcommand114();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'description': 'PRND control(Command)', 'enum':
  // {1: 'GEAR_CMD_PARK', 2: 'GEAR_CMD_REVERSE', 3: 'GEAR_CMD_NEUTRAL', 4:
  // 'GEAR_CMD_DRIVE'}, 'is_signed_var': False, 'len': 8, 'name': 'GEAR_CMD',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[1|4]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Gearcommand114* set_gear_cmd(Gear_command_114::Gear_cmdType gear_cmd);

 private:
  // config detail: {'bit': 0, 'description': 'PRND control(Command)', 'enum':
  // {1: 'GEAR_CMD_PARK', 2: 'GEAR_CMD_REVERSE', 3: 'GEAR_CMD_NEUTRAL', 4:
  // 'GEAR_CMD_DRIVE'}, 'is_signed_var': False, 'len': 8, 'name': 'GEAR_CMD',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[1|4]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_gear_cmd(uint8_t* data, Gear_command_114::Gear_cmdType gear_cmd);

 private:
  Gear_command_114::Gear_cmdType gear_cmd_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
