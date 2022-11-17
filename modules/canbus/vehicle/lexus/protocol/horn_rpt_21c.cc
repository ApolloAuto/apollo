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

#include "modules/canbus/vehicle/lexus/protocol/horn_rpt_21c.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Hornrpt21c::Hornrpt21c() {}
const int32_t Hornrpt21c::ID = 0x21C;

void Hornrpt21c::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_vehicle_fault(
      vehicle_fault(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_pacmod_fault(
      pacmod_fault(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_override_active(
      override_active(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_output_reported_fault(
      output_reported_fault(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_input_output_fault(
      input_output_fault(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_enabled(
      enabled(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_command_output_fault(
      command_output_fault(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_output_value(
      output_value(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_commanded_value(
      commanded_value(bytes, length));
  chassis->mutable_lexus()->mutable_horn_rpt_21c()->set_manual_input(
      manual_input(bytes, length));
}

// config detail: {'name': 'vehicle_fault', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::vehicle_fault(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pacmod_fault', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::pacmod_fault(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'override_active', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::override_active(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'output_reported_fault', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::output_reported_fault(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'input_output_fault', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::input_output_fault(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'enabled', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::enabled(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'command_output_fault', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Hornrpt21c::command_output_fault(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'output_value', 'enum': {0: 'OUTPUT_VALUE_OFF', 1:
// 'OUTPUT_VALUE_ON'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Horn_rpt_21c::Output_valueType Hornrpt21c::output_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Horn_rpt_21c::Output_valueType ret =
      static_cast<Horn_rpt_21c::Output_valueType>(x);
  return ret;
}

// config detail: {'name': 'commanded_value', 'enum': {0: 'COMMANDED_VALUE_OFF',
// 1: 'COMMANDED_VALUE_ON'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Horn_rpt_21c::Commanded_valueType Hornrpt21c::commanded_value(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Horn_rpt_21c::Commanded_valueType ret =
      static_cast<Horn_rpt_21c::Commanded_valueType>(x);
  return ret;
}

// config detail: {'name': 'manual_input', 'enum': {0: 'MANUAL_INPUT_OFF', 1:
// 'MANUAL_INPUT_ON'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order':
// 'motorola', 'physical_unit': ''}
Horn_rpt_21c::Manual_inputType Hornrpt21c::manual_input(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Horn_rpt_21c::Manual_inputType ret =
      static_cast<Horn_rpt_21c::Manual_inputType>(x);
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
