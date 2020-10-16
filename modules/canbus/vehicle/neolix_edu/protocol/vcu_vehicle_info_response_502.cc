/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_vehicle_info_response_502.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcuvehicleinforesponse502::Vcuvehicleinforesponse502() {}
const int32_t Vcuvehicleinforesponse502::ID = 0x502;

void Vcuvehicleinforesponse502::Parse(const std::uint8_t* bytes, int32_t length,
                                      ChassisDetail* chassis) const {
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_vehicle_softwareversion_indicati(
          vehicle_softwareversion_indicati(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_project(project(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_manufacturer(manufacturer(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_year(year(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_month(month(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_day(day(bytes, length));
  chassis->mutable_neolix_edu()
      ->mutable_vcu_vehicle_info_response_502()
      ->set_vehicle_serial_number(vehicle_serial_number(bytes, length));
}

// config detail: {'name': 'vehicle_softwareversion_indicati', 'offset': 0.0,
// 'precision': 1.0, 'len': 24, 'is_signed_var': False, 'physical_range':
// '[0|16777215]', 'bit': 7, 'type': 'int', 'order': 'motorola',
// 'physical_unit': ''}
int Vcuvehicleinforesponse502::vehicle_softwareversion_indicati(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'project', 'offset': 0.0, 'precision': 1.0, 'len': 4,
// 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 27, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcuvehicleinforesponse502::project(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'manufacturer', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehicleinforesponse502::manufacturer(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'year', 'offset': 0.0, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehicleinforesponse502::year(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'month', 'offset': 0.0, 'precision': 1.0, 'len': 4,
// 'is_signed_var': False, 'physical_range': '[1|12]', 'bit': 47, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcuvehicleinforesponse502::month(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'day', 'offset': 0.0, 'precision': 1.0, 'len': 5,
// 'is_signed_var': False, 'physical_range': '[1|31]', 'bit': 43, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcuvehicleinforesponse502::day(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'vehicle_serial_number', 'offset': 0.0,
// 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range':
// '[0|32767]', 'bit': 54, 'type': 'int', 'order': 'motorola', 'physical_unit':
// ''}
int Vcuvehicleinforesponse502::vehicle_serial_number(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
