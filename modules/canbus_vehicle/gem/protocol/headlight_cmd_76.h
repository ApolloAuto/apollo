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

class Headlightcmd76 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Gem> {
 public:
  static const int32_t ID;

  Headlightcmd76();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'HEADLIGHT_CMD', 'enum': {0:
  // 'HEADLIGHT_CMD_HEADLIGHTS_OFF', 1: 'HEADLIGHT_CMD_LOW_BEAMS', 2:
  // 'HEADLIGHT_CMD_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Headlightcmd76* set_headlight_cmd(
      Headlight_cmd_76::Headlight_cmdType headlight_cmd);

 private:
  // config detail: {'name': 'HEADLIGHT_CMD', 'enum': {0:
  // 'HEADLIGHT_CMD_HEADLIGHTS_OFF', 1: 'HEADLIGHT_CMD_LOW_BEAMS', 2:
  // 'HEADLIGHT_CMD_HIGH_BEAMS'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_headlight_cmd(uint8_t* data,
                           Headlight_cmd_76::Headlight_cmdType headlight_cmd);

 private:
  Headlight_cmd_76::Headlight_cmdType headlight_cmd_;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
