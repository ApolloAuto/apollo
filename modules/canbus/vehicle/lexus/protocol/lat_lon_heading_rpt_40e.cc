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

#include "modules/canbus/vehicle/lexus/protocol/lat_lon_heading_rpt_40e.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Latlonheadingrpt40e::Latlonheadingrpt40e() {}
const int32_t Latlonheadingrpt40e::ID = 0x40E;

void Latlonheadingrpt40e::Parse(const std::uint8_t* bytes, int32_t length,
                                ChassisDetail* chassis) const {
  chassis->mutable_lexus()->mutable_lat_lon_heading_rpt_40e()->set_heading(
      heading(bytes, length));
  chassis->mutable_lexus()
      ->mutable_lat_lon_heading_rpt_40e()
      ->set_longitude_seconds(longitude_seconds(bytes, length));
  chassis->mutable_lexus()
      ->mutable_lat_lon_heading_rpt_40e()
      ->set_longitude_minutes(longitude_minutes(bytes, length));
  chassis->mutable_lexus()
      ->mutable_lat_lon_heading_rpt_40e()
      ->set_longitude_degrees(longitude_degrees(bytes, length));
  chassis->mutable_lexus()
      ->mutable_lat_lon_heading_rpt_40e()
      ->set_latitude_seconds(latitude_seconds(bytes, length));
  chassis->mutable_lexus()
      ->mutable_lat_lon_heading_rpt_40e()
      ->set_latitude_minutes(latitude_minutes(bytes, length));
  chassis->mutable_lexus()
      ->mutable_lat_lon_heading_rpt_40e()
      ->set_latitude_degrees(latitude_degrees(bytes, length));
}

// config detail: {'name': 'heading', 'offset': 0.0, 'precision': 0.01, 'len':
// 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]', 'bit': 55,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Latlonheadingrpt40e::heading(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'longitude_seconds', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit': 47,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'sec'}
int Latlonheadingrpt40e::longitude_seconds(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'longitude_minutes', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit': 39,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'min'}
int Latlonheadingrpt40e::longitude_minutes(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'longitude_degrees', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
int Latlonheadingrpt40e::longitude_degrees(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'latitude_seconds', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit': 23,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'sec'}
int Latlonheadingrpt40e::latitude_seconds(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'latitude_minutes', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit': 15,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'min'}
int Latlonheadingrpt40e::latitude_minutes(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'latitude_degrees', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
int Latlonheadingrpt40e::latitude_degrees(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
