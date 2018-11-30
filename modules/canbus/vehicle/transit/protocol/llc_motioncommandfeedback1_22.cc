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

#include "modules/canbus/vehicle/transit/protocol/llc_motioncommandfeedback1_22.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcmotioncommandfeedback122::Llcmotioncommandfeedback122() {}
const int32_t Llcmotioncommandfeedback122::ID = 0x22;

void Llcmotioncommandfeedback122::Parse(const std::uint8_t* bytes,
                                        int32_t length,
                                        ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_llc_motioncommandfeedback1_22()
      ->set_llc_fbk_steeringanglesetpoint(
          llc_fbk_steeringanglesetpoint(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motioncommandfeedback1_22()
      ->set_llc_fbk_throttlesetpoint(llc_fbk_throttlesetpoint(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motioncommandfeedback1_22()
      ->set_llc_fbk_brakepercentsetpoint(
          llc_fbk_brakepercentsetpoint(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motioncommandfeedback1_22()
      ->set_llc_motioncommandfeedback1_count(
          llc_motioncommandfeedback1_count(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motioncommandfeedback1_22()
      ->set_llc_motioncommandfeedback1_check(
          llc_motioncommandfeedback1_check(bytes, length));
}

// config detail: {'description': 'Steering angle setpoint (after limits)',
// 'offset': 0.0, 'precision': 0.05, 'len': 16, 'name':
// 'llc_fbk_steeringanglesetpoint', 'is_signed_var': True, 'physical_range':
// '[-1638.4|1638.35]', 'bit': 21, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'deg'}
double Llcmotioncommandfeedback122::llc_fbk_steeringanglesetpoint(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(5, 3);
  x <<= 3;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.050000;
  return ret;
}

// config detail: {'description': 'Current throttle setpoint (after limits)',
// 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
// 'llc_fbk_throttlesetpoint', 'is_signed_var': False, 'physical_range':
// '[0|102.3]', 'bit': 11, 'type': 'double', 'order': 'intel', 'physical_unit':
// '%'}
double Llcmotioncommandfeedback122::llc_fbk_throttlesetpoint(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'Front brake pressure setpoint (after
// limits)', 'offset': 0.0, 'precision': 0.0556, 'len': 11, 'name':
// 'llc_fbk_brakepercentsetpoint', 'is_signed_var': False, 'physical_range':
// '[0|113.8132]', 'bit': 0, 'type': 'double', 'order': 'intel',
// 'physical_unit': '%'}
double Llcmotioncommandfeedback122::llc_fbk_brakepercentsetpoint(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.055600;
  return ret;
}

// config detail: {'description': 'Motion command feedback 2 heartbeat counter',
// 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
// 'llc_motioncommandfeedback1_count', 'is_signed_var': False, 'physical_range':
// '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcmotioncommandfeedback122::llc_motioncommandfeedback1_count(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Motion command feedback 1 checksum',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
// 'llc_motioncommandfeedback1_check', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcmotioncommandfeedback122::llc_motioncommandfeedback1_check(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
