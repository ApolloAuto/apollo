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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

class Mediacontrolscmd120 : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Mediacontrolscmd120();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'MEDIA_CONTROLS_CMD', 'enum': {0:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_NONE', 1:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_VOICE_COMMAND', 2:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_MUTE', 3:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_PREV_TRACK_ANSWER', 4:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_NEXT_TRACK_HANG_UP', 5:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_VOL_UP', 6:
  // 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_VOL_DOWN'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
  // 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Media_controls_cmd_120::Media_controls_cmdType media_controls_cmd(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IGNORE_OVERRIDES', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit':
  // ''}
  bool ignore_overrides(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CLEAR_OVERRIDE', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool clear_override(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CLEAR_FAULTS', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool clear_faults(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ENABLE', 'offset': 0.0, 'precision': 1.0, 'len':
  // 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type':
  // 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool enable(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
