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

#include "modules/canbus/vehicle/lexus/protocol/wiper_aux_rpt_334.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Wiperauxrpt334::Wiperauxrpt334() {}
const int32_t Wiperauxrpt334::ID = 0x334;

void Wiperauxrpt334::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_lexus()
      ->mutable_wiper_aux_rpt_334()
      ->set_spray_empty_is_valid(spray_empty_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_wiper_aux_rpt_334()->set_spray_empty(
      spray_empty(bytes, length));
  chassis->mutable_lexus()
      ->mutable_wiper_aux_rpt_334()
      ->set_spray_near_empty_is_valid(spray_near_empty_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_wiper_aux_rpt_334()->set_spray_near_empty(
      spray_near_empty(bytes, length));
  chassis->mutable_lexus()
      ->mutable_wiper_aux_rpt_334()
      ->set_rear_spraying_is_valid(rear_spraying_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_wiper_aux_rpt_334()->set_rear_spraying(
      rear_spraying(bytes, length));
  chassis->mutable_lexus()
      ->mutable_wiper_aux_rpt_334()
      ->set_rear_wiping_is_valid(rear_wiping_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_wiper_aux_rpt_334()->set_rear_wiping(
      rear_wiping(bytes, length));
  chassis->mutable_lexus()
      ->mutable_wiper_aux_rpt_334()
      ->set_front_spraying_is_valid(front_spraying_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_wiper_aux_rpt_334()->set_front_spraying(
      front_spraying(bytes, length));
  chassis->mutable_lexus()
      ->mutable_wiper_aux_rpt_334()
      ->set_front_wiping_is_valid(front_wiping_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_wiper_aux_rpt_334()->set_front_wiping(
      front_wiping(bytes, length));
}

// config detail: {'name': 'spray_empty_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::spray_empty_is_valid(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'spray_empty', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::spray_empty(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'spray_near_empty_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::spray_near_empty_is_valid(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'spray_near_empty', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 4,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::spray_near_empty(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_spraying_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::rear_spraying_is_valid(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_spraying', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 3,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::rear_spraying(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_wiping_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 10, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::rear_wiping_is_valid(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_wiping', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::rear_wiping(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'front_spraying_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::front_spraying_is_valid(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'front_spraying', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::front_spraying(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'front_wiping_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::front_wiping_is_valid(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'front_wiping', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Wiperauxrpt334::front_wiping(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
