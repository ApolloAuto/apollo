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

#include "modules/canbus/vehicle/transit/protocol/llc_diag_fault_620.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcdiagfault620::Llcdiagfault620() {}
const int32_t Llcdiagfault620::ID = 0x620;

void Llcdiagfault620::Parse(const std::uint8_t* bytes, int32_t length,
                            ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_disengagecounter_brake(
          llc_disengagecounter_brake(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_disengagecounter_steer(
          llc_disengagecounter_steer(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_disengagecounter_throttle(
          llc_disengagecounter_throttle(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_fbk_faultcounter(llc_fbk_faultcounter(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_disengagecounter_button(
          llc_disengagecounter_button(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_fbk_version_year(llc_fbk_version_year(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_fbk_version_month(llc_fbk_version_month(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_fbk_version_day(llc_fbk_version_day(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_diag_fault_620()
      ->set_llc_fbk_version_hour(llc_fbk_version_hour(bytes, length));
}

// config detail: {'description': 'Counts the number of times that the driver
// has disengaged autonomy by applying the brakes since system reset..',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'llc_disengagecounter_brake', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcdiagfault620::llc_disengagecounter_brake(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Counts the number of times that the driver
// has disengaged autonomy by moving the steering wheel since system reset.',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'llc_disengagecounter_steer', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcdiagfault620::llc_disengagecounter_steer(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Counts the number of times that the driver
// has disengaged autonomy by applying throttle since system reset.', 'offset':
// 0.0, 'precision': 1.0, 'len': 8, 'name': 'llc_disengagecounter_throttle',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Llcdiagfault620::llc_disengagecounter_throttle(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Counts the number of faults that have
// occurred since system reset.', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'name': 'llc_fbk_faultcounter', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcdiagfault620::llc_fbk_faultcounter(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Counts the number of times that the driver
// has disengaged autonomy by applying the brakes since system reset..',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'llc_disengagecounter_button', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcdiagfault620::llc_disengagecounter_button(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Firmware version', 'offset': 2017.0,
// 'precision': 1.0, 'len': 7, 'name': 'llc_fbk_version_year', 'is_signed_var':
// False, 'physical_range': '[2017|2144]', 'bit': 40, 'type': 'int', 'order':
// 'intel', 'physical_unit': 'g'}
int Llcdiagfault620::llc_fbk_version_year(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 7);

  int ret = static_cast<int>(x + 2017.000000);
  return ret;
}

// config detail: {'description': 'Firmware version', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'name': 'llc_fbk_version_month', 'is_signed_var':
// False, 'physical_range': '[0|15]', 'bit': 47, 'type': 'int', 'order':
// 'intel', 'physical_unit': 'Month'}
int Llcdiagfault620::llc_fbk_version_month(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'description': 'Firmware version', 'offset': 0.0,
// 'precision': 1.0, 'len': 5, 'name': 'llc_fbk_version_day', 'is_signed_var':
// False, 'physical_range': '[0|31]', 'bit': 51, 'type': 'int', 'order':
// 'intel', 'physical_unit': 'Day'}
int Llcdiagfault620::llc_fbk_version_day(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Firmware version', 'offset': 0.0,
// 'precision': 1.0, 'len': 5, 'name': 'llc_fbk_version_hour', 'is_signed_var':
// False, 'physical_range': '[0|31]', 'bit': 56, 'type': 'int', 'order':
// 'intel', 'physical_unit': 'Hour'}
int Llcdiagfault620::llc_fbk_version_hour(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 5);

  int ret = x;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
