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

class Globalcmd69 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::Gem> {
 public:
  static const int32_t ID;

  Globalcmd69();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'PACMOD_ENABLE', 'enum': {0:
  // 'PACMOD_ENABLE_CONTROL_DISABLED', 1: 'PACMOD_ENABLE_CONTROL_ENABLED'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Globalcmd69* set_pacmod_enable(
      Global_cmd_69::Pacmod_enableType pacmod_enable);

  // config detail: {'name': 'CLEAR_OVERRIDE', 'enum': {0:
  // 'CLEAR_OVERRIDE_DON_T_CLEAR_ACTIVE_OVERRIDES', 1:
  // 'CLEAR_OVERRIDE_CLEAR_ACTIVE_OVERRIDES'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Globalcmd69* set_clear_override(
      Global_cmd_69::Clear_overrideType clear_override);

  // config detail: {'name': 'IGNORE_OVERRIDE', 'enum': {0:
  // 'IGNORE_OVERRIDE_DON_T_IGNORE_USER_OVERRIDES', 1:
  // 'IGNORE_OVERRIDE_IGNORE_USER_OVERRIDES'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Globalcmd69* set_ignore_override(
      Global_cmd_69::Ignore_overrideType ignore_override);

 private:
  // config detail: {'name': 'PACMOD_ENABLE', 'enum': {0:
  // 'PACMOD_ENABLE_CONTROL_DISABLED', 1: 'PACMOD_ENABLE_CONTROL_ENABLED'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pacmod_enable(uint8_t* data,
                           Global_cmd_69::Pacmod_enableType pacmod_enable);

  // config detail: {'name': 'CLEAR_OVERRIDE', 'enum': {0:
  // 'CLEAR_OVERRIDE_DON_T_CLEAR_ACTIVE_OVERRIDES', 1:
  // 'CLEAR_OVERRIDE_CLEAR_ACTIVE_OVERRIDES'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_clear_override(uint8_t* data,
                            Global_cmd_69::Clear_overrideType clear_override);

  // config detail: {'name': 'IGNORE_OVERRIDE', 'enum': {0:
  // 'IGNORE_OVERRIDE_DON_T_IGNORE_USER_OVERRIDES', 1:
  // 'IGNORE_OVERRIDE_IGNORE_USER_OVERRIDES'}, 'precision': 1.0, 'len': 1,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ignore_override(
      uint8_t* data, Global_cmd_69::Ignore_overrideType ignore_override);

 private:
  Global_cmd_69::Pacmod_enableType pacmod_enable_;
  Global_cmd_69::Clear_overrideType clear_override_;
  Global_cmd_69::Ignore_overrideType ignore_override_;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
