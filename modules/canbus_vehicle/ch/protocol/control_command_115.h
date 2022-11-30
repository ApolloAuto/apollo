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

class Controlcommand115 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;

  Controlcommand115();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Take control(Command)', 'enum': {0:
  // 'CTRL_CMD_OUT_OF_CONTROL', 1: 'CTRL_CMD_UNDER_CONTROL'}, 'precision': 1.0,
  // 'len': 8, 'name': 'CTRL_CMD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Controlcommand115* set_ctrl_cmd(Control_command_115::Ctrl_cmdType ctrl_cmd);

 private:
  // config detail: {'description': 'Take control(Command)', 'enum': {0:
  // 'CTRL_CMD_OUT_OF_CONTROL', 1: 'CTRL_CMD_UNDER_CONTROL'}, 'precision': 1.0,
  // 'len': 8, 'name': 'CTRL_CMD', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_ctrl_cmd(uint8_t* data,
                      Control_command_115::Ctrl_cmdType ctrl_cmd);

 private:
  Control_command_115::Ctrl_cmdType ctrl_cmd_;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
