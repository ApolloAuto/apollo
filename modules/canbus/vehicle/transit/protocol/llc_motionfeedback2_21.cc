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

#include "modules/canbus/vehicle/transit/protocol/llc_motionfeedback2_21.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcmotionfeedback221::Llcmotionfeedback221() {}
const int32_t Llcmotionfeedback221::ID = 0x21;

void Llcmotionfeedback221::Parse(const std::uint8_t* bytes, int32_t length,
                                 ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback2_21()
      ->set_llc_fbk_vehiclespeed(llc_fbk_vehiclespeed(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback2_21()
      ->set_llc_motionfeedback2_counter(
          llc_motionfeedback2_counter(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback2_21()
      ->set_llc_motionfeedback2_checksum(
          llc_motionfeedback2_checksum(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback2_21()
      ->set_llc_fbk_steeringrate(llc_fbk_steeringrate(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback2_21()
      ->set_llc_fbk_steeringangle(llc_fbk_steeringangle(bytes, length));
}

// config detail: {'name': 'llc_fbk_vehiclespeed', 'offset': 0.0, 'precision':
// 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|655.35]',
// 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s'}
double Llcmotionfeedback221::llc_fbk_vehiclespeed(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'description': 'Motion feedback 2 heartbeat counter',
// 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
// 'llc_motionfeedback2_counter', 'is_signed_var': False, 'physical_range':
// '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcmotionfeedback221::llc_motionfeedback2_counter(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Motion feedback 2 checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'llc_motionfeedback2_checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Llcmotionfeedback221::llc_motionfeedback2_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Steer wheel angle feedback from SbW motor (?
// rate)', 'offset': 0.0, 'precision': 0.05, 'len': 16, 'name':
// 'llc_fbk_steeringrate', 'is_signed_var': True, 'physical_range':
// '[-1638.4|1638.3]', 'bit': 16, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'deg/s'}
double Llcmotionfeedback221::llc_fbk_steeringrate(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.050000;
  return ret;
}

// config detail: {'description': 'Steering angle feedback', 'offset': 0.0,
// 'precision': 0.05, 'len': 16, 'name': 'llc_fbk_steeringangle',
// 'is_signed_var': True, 'physical_range': '[-1638.4|1638.35]', 'bit': 0,
// 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
double Llcmotionfeedback221::llc_fbk_steeringangle(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.050000;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
