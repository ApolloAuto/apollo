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

#include "modules/canbus/vehicle/minibus/protocol/vcu_drive_feedback_18ff7097.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Vcudrivefeedback18ff7097::Vcudrivefeedback18ff7097() {}
const int32_t Vcudrivefeedback18ff7097::ID = 0x38ff7097;

void Vcudrivefeedback18ff7097::Parse(const std::uint8_t* bytes, int32_t length,
                                     ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_vcu_drive_feedback_18ff7097()
      ->set_vcu_drive_life_signal(vcu_drive_life_signal(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_drive_feedback_18ff7097()
      ->set_vcu_drive_vehicle_speed(vcu_drive_vehicle_speed(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_drive_feedback_18ff7097()
      ->set_vcu_drive_real_torque(vcu_drive_real_torque(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_drive_feedback_18ff7097()
      ->set_vcu_drive_break_pedal_position(
          vcu_drive_break_pedal_position(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_drive_feedback_18ff7097()
      ->set_vcu_drive_throttle_pedal_position(
          vcu_drive_throttle_pedal_position(bytes, length));
  chassis->mutable_minibus()
      ->mutable_vcu_drive_feedback_18ff7097()
      ->set_vcu_drive_autodrive_confirm(
          vcu_drive_autodrive_confirm(bytes, length));
  chassis->mutable_check_response()->set_is_vcu_online(
      vcu_drive_autodrive_confirm(bytes, length) == 1);
  chassis->mutable_check_response()->set_is_esp_online(
      vcu_drive_autodrive_confirm(bytes, length) == 1);
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'vcu_drive_life_signal', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcudrivefeedback18ff7097::vcu_drive_life_signal(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 16, 'name':
// 'vcu_drive_vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': 'Km/h', 'precision': 1.0, 'type': 'int'}
int Vcudrivefeedback18ff7097::vcu_drive_vehicle_speed(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name':
// 'vcu_drive_real_torque', 'offset': -5000.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': 'Nm', 'precision': 1.0, 'type':
// 'int'}
int Vcudrivefeedback18ff7097::vcu_drive_real_torque(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -5000.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'vcu_drive_break_pedal_position', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type':
// 'double'}
double Vcudrivefeedback18ff7097::vcu_drive_break_pedal_position(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.400000;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'vcu_drive_throttle_pedal_position', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.4, 'type':
// 'double'}
double Vcudrivefeedback18ff7097::vcu_drive_throttle_pedal_position(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.400000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name':
// 'vcu_drive_autodrive_confirm', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Vcudrivefeedback18ff7097::vcu_drive_autodrive_confirm(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
