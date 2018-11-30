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

#include "modules/canbus/vehicle/transit/protocol/adc_motioncontrollimits1_12.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Adcmotioncontrollimits112::Adcmotioncontrollimits112() {}
const int32_t Adcmotioncontrollimits112::ID = 0x12;

void Adcmotioncontrollimits112::Parse(const std::uint8_t* bytes, int32_t length,
                                      ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_adc_motioncontrollimits1_12()
      ->set_adc_cmd_throttlecommandlimit(
          adc_cmd_throttlecommandlimit(bytes, length));
  chassis->mutable_transit()
      ->mutable_adc_motioncontrollimits1_12()
      ->set_adc_cmd_steeringrate(adc_cmd_steeringrate(bytes, length));
  chassis->mutable_transit()
      ->mutable_adc_motioncontrollimits1_12()
      ->set_adc_cmd_steerwheelanglelimit(
          adc_cmd_steerwheelanglelimit(bytes, length));
}

// config detail: {'description': 'Set limit for throttle position', 'offset':
// 0.0, 'precision': 0.5, 'len': 8, 'name': 'adc_cmd_throttlecommandlimit',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 24, 'type':
// 'double', 'order': 'intel', 'physical_unit': '%'}
double Adcmotioncontrollimits112::adc_cmd_throttlecommandlimit(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'description': 'Set steering rate', 'offset': 0.0,
// 'precision': 0.05, 'len': 16, 'name': 'adc_cmd_steeringrate',
// 'is_signed_var': False, 'physical_range': '[0|3276.75]', 'bit': 0, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'deg/s'}
double Adcmotioncontrollimits112::adc_cmd_steeringrate(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.050000;
  return ret;
}

// config detail: {'description': 'Set limit for steering wheel angle. Applies
// in both positive and negative', 'offset': 0.0, 'precision': 5.0, 'len': 8,
// 'name': 'adc_cmd_steerwheelanglelimit', 'is_signed_var': False,
// 'physical_range': '[0|1275]', 'bit': 16, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'deg'}
double Adcmotioncontrollimits112::adc_cmd_steerwheelanglelimit(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 5.000000;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
