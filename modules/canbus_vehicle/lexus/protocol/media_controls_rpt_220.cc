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

#include "modules/canbus_vehicle/lexus/protocol/media_controls_rpt_220.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Mediacontrolsrpt220::Mediacontrolsrpt220() {}
const int32_t Mediacontrolsrpt220::ID = 0x220;

void Mediacontrolsrpt220::Parse(const std::uint8_t* bytes, int32_t length,
                                Lexus* chassis) const {
  chassis->mutable_media_controls_rpt_220()->set_output_value(
      output_value(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_manual_input(
      manual_input(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_vehicle_fault(
      vehicle_fault(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_pacmod_fault(
      pacmod_fault(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_override_active(
      override_active(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_output_reported_fault(
      output_reported_fault(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_input_output_fault(
      input_output_fault(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_enabled(
      enabled(bytes, length));
  chassis->mutable_media_controls_rpt_220()->set_command_output_fault(
      command_output_fault(bytes, length));
}

// config detail: {'name': 'output_value', 'enum': {0:
// 'OUTPUT_VALUE_MEDIA_CONTROL_NONE', 1:
// 'OUTPUT_VALUE_MEDIA_CONTROL_VOICE_COMMAND', 2:
// 'OUTPUT_VALUE_MEDIA_CONTROL_MUTE', 3:
// 'OUTPUT_VALUE_MEDIA_CONTROL_PREV_TRACK_ANSWER', 4:
// 'OUTPUT_VALUE_MEDIA_CONTROL_NEXT_TRACK_HANG_UP', 5:
// 'OUTPUT_VALUE_MEDIA_CONTROL_VOL_UP', 6:
// 'OUTPUT_VALUE_MEDIA_CONTROL_VOL_DOWN'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Media_controls_rpt_220::Output_valueType Mediacontrolsrpt220::output_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Media_controls_rpt_220::Output_valueType ret =
      static_cast<Media_controls_rpt_220::Output_valueType>(x);
  return ret;
}

// config detail: {'name': 'commanded_value', 'enum': {0:
// 'COMMANDED_VALUE_MEDIA_CONTROL_NONE', 1:
// 'COMMANDED_VALUE_MEDIA_CONTROL_VOICE_COMMAND', 2:
// 'COMMANDED_VALUE_MEDIA_CONTROL_MUTE', 3:
// 'COMMANDED_VALUE_MEDIA_CONTROL_PREV_TRACK_ANSWER', 4:
// 'COMMANDED_VALUE_MEDIA_CONTROL_NEXT_TRACK_HANG_UP', 5:
// 'COMMANDED_VALUE_MEDIA_CONTROL_VOL_UP', 6:
// 'COMMANDED_VALUE_MEDIA_CONTROL_VOL_DOWN'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Media_controls_rpt_220::Commanded_valueType
Mediacontrolsrpt220::commanded_value(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Media_controls_rpt_220::Commanded_valueType ret =
      static_cast<Media_controls_rpt_220::Commanded_valueType>(x);
  return ret;
}

// config detail: {'name': 'manual_input', 'enum': {0:
// 'MANUAL_INPUT_MEDIA_CONTROL_NONE', 1:
// 'MANUAL_INPUT_MEDIA_CONTROL_VOICE_COMMAND', 2:
// 'MANUAL_INPUT_MEDIA_CONTROL_MUTE', 3:
// 'MANUAL_INPUT_MEDIA_CONTROL_PREV_TRACK_ANSWER', 4:
// 'MANUAL_INPUT_MEDIA_CONTROL_NEXT_TRACK_HANG_UP', 5:
// 'MANUAL_INPUT_MEDIA_CONTROL_VOL_UP', 6:
// 'MANUAL_INPUT_MEDIA_CONTROL_VOL_DOWN'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit':
// 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Media_controls_rpt_220::Manual_inputType Mediacontrolsrpt220::manual_input(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Media_controls_rpt_220::Manual_inputType ret =
      static_cast<Media_controls_rpt_220::Manual_inputType>(x);
  return ret;
}

// config detail: {'name': 'vehicle_fault', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::vehicle_fault(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pacmod_fault', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::pacmod_fault(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'override_active', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::override_active(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'output_reported_fault', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::output_reported_fault(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'input_output_fault', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::input_output_fault(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'enabled', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::enabled(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'command_output_fault', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Mediacontrolsrpt220::command_output_fault(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
