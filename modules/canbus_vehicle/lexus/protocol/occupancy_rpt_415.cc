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

#include "modules/canbus_vehicle/lexus/protocol/occupancy_rpt_415.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Occupancyrpt415::Occupancyrpt415() {}
const int32_t Occupancyrpt415::ID = 0x415;

void Occupancyrpt415::Parse(const std::uint8_t* bytes, int32_t length,
                            Lexus* chassis) const {
  chassis->mutable_occupancy_rpt_415()->set_rear_seatbelt_buckled_is_valid(
      rear_seatbelt_buckled_is_valid(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_pass_seatbelt_buckled_is_valid(
      pass_seatbelt_buckled_is_valid(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_driver_seatbelt_buckled_is_valid(
      driver_seatbelt_buckled_is_valid(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_rear_seat_occupied_is_valid(
      rear_seat_occupied_is_valid(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_pass_seat_occupied_is_valid(
      pass_seat_occupied_is_valid(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_driver_seat_occupied_is_valid(
      driver_seat_occupied_is_valid(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_rear_seatbelt_buckled(
      rear_seatbelt_buckled(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_pass_seatbelt_buckled(
      pass_seatbelt_buckled(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_driver_seatbelt_buckled(
      driver_seatbelt_buckled(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_rear_seat_occupied(
      rear_seat_occupied(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_pass_seat_occupied(
      pass_seat_occupied(bytes, length));
  chassis->mutable_occupancy_rpt_415()->set_driver_seat_occupied(
      driver_seat_occupied(bytes, length));
}

// config detail: {'name': 'rear_seatbelt_buckled_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::rear_seatbelt_buckled_is_valid(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_seatbelt_buckled_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::pass_seatbelt_buckled_is_valid(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_seatbelt_buckled_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::driver_seatbelt_buckled_is_valid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_seat_occupied_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 10, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::rear_seat_occupied_is_valid(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_seat_occupied_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::pass_seat_occupied_is_valid(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_seat_occupied_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::driver_seat_occupied_is_valid(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_seatbelt_buckled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 5, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::rear_seatbelt_buckled(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_seatbelt_buckled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::pass_seatbelt_buckled(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_seatbelt_buckled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::driver_seatbelt_buckled(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_seat_occupied', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::rear_seat_occupied(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_seat_occupied', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::pass_seat_occupied(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_seat_occupied', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Occupancyrpt415::driver_seat_occupied(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
