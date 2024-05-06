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

#include "modules/canbus_vehicle/lexus/protocol/media_controls_cmd_120.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Mediacontrolscmd120::Mediacontrolscmd120() {}
const int32_t Mediacontrolscmd120::ID = 0x120;

void Mediacontrolscmd120::Parse(const std::uint8_t* bytes, int32_t length,
                                Lexus* chassis) const {
  chassis->mutable_media_controls_cmd_120()->set_media_controls_cmd(
      media_controls_cmd(bytes, length));
  chassis->mutable_media_controls_cmd_120()->set_ignore_overrides(
      ignore_overrides(bytes, length));
  chassis->mutable_media_controls_cmd_120()->set_clear_override(
      clear_override(bytes, length));
  chassis->mutable_media_controls_cmd_120()->set_clear_faults(
      clear_faults(bytes, length));
  chassis->mutable_media_controls_cmd_120()->set_enable(
      enable(bytes, length));
}

// config detail: {'name': 'media_controls_cmd', 'enum': {0:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_NONE', 1:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_VOICE_COMMAND', 2:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_MUTE', 3:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_PREV_TRACK_ANSWER', 4:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_NEXT_TRACK_HANG_UP', 5:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_VOL_UP', 6:
// 'MEDIA_CONTROLS_CMD_MEDIA_CONTROL_VOL_DOWN'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Media_controls_cmd_120::Media_controls_cmdType
Mediacontrolscmd120::media_controls_cmd(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Media_controls_cmd_120::Media_controls_cmdType ret =
      static_cast<Media_controls_cmd_120::Media_controls_cmdType>(x);
  return ret;
}

// config detail: {'name': 'ignore_overrides', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolscmd120::ignore_overrides(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'clear_override', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolscmd120::clear_override(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'clear_faults', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolscmd120::clear_faults(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'enable', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolscmd120::enable(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
