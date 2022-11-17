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

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

const int32_t Adcmotioncontrollimits112::ID = 0x12;

// public
Adcmotioncontrollimits112::Adcmotioncontrollimits112() { Reset(); }

uint32_t Adcmotioncontrollimits112::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Adcmotioncontrollimits112::UpdateData(uint8_t* data) {
  set_p_adc_cmd_throttlecommandlimit(data, adc_cmd_throttlecommandlimit_);
  set_p_adc_cmd_steeringrate(data, adc_cmd_steeringrate_);
  set_p_adc_cmd_steerwheelanglelimit(data, adc_cmd_steerwheelanglelimit_);
}

void Adcmotioncontrollimits112::Reset() {
  // TODO(All) :  you should check this manually
  adc_cmd_throttlecommandlimit_ = 0.0;
  adc_cmd_steeringrate_ = 0.0;
  adc_cmd_steerwheelanglelimit_ = 0.0;
}

Adcmotioncontrollimits112*
Adcmotioncontrollimits112::set_adc_cmd_throttlecommandlimit(
    double adc_cmd_throttlecommandlimit) {
  adc_cmd_throttlecommandlimit_ = adc_cmd_throttlecommandlimit;
  return this;
}

// config detail: {'description': 'Set limit for throttle position', 'offset':
// 0.0, 'precision': 0.5, 'len': 8, 'name': 'ADC_CMD_ThrottleCommandLimit',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 24, 'type':
// 'double', 'order': 'intel', 'physical_unit': '%'}
void Adcmotioncontrollimits112::set_p_adc_cmd_throttlecommandlimit(
    uint8_t* data, double adc_cmd_throttlecommandlimit) {
  adc_cmd_throttlecommandlimit =
      ProtocolData::BoundedValue(0.0, 100.0, adc_cmd_throttlecommandlimit);
  int x = static_cast<int>(adc_cmd_throttlecommandlimit / 0.500000);

  Byte to_set(data + 3);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Adcmotioncontrollimits112* Adcmotioncontrollimits112::set_adc_cmd_steeringrate(
    double adc_cmd_steeringrate) {
  adc_cmd_steeringrate_ = adc_cmd_steeringrate;
  return this;
}

// config detail: {'description': 'Set steering rate', 'offset': 0.0,
// 'precision': 0.05, 'len': 16, 'name': 'ADC_CMD_SteeringRate',
// 'is_signed_var': False, 'physical_range': '[0|3276.75]', 'bit': 0, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'deg/s'}
void Adcmotioncontrollimits112::set_p_adc_cmd_steeringrate(
    uint8_t* data, double adc_cmd_steeringrate) {
  adc_cmd_steeringrate =
      ProtocolData::BoundedValue(0.0, 3276.75, adc_cmd_steeringrate);
  int x = static_cast<int>(adc_cmd_steeringrate / 0.050000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

Adcmotioncontrollimits112*
Adcmotioncontrollimits112::set_adc_cmd_steerwheelanglelimit(
    double adc_cmd_steerwheelanglelimit) {
  adc_cmd_steerwheelanglelimit_ = adc_cmd_steerwheelanglelimit;
  return this;
}

// config detail: {'description': 'Set limit for steering wheel angle. Applies
// in both positive and negative', 'offset': 0.0, 'precision': 5.0, 'len': 8,
// 'name': 'ADC_CMD_SteerWheelAngleLimit', 'is_signed_var': False,
// 'physical_range': '[0|1275]', 'bit': 16, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'deg'}
void Adcmotioncontrollimits112::set_p_adc_cmd_steerwheelanglelimit(
    uint8_t* data, double adc_cmd_steerwheelanglelimit) {
  adc_cmd_steerwheelanglelimit =
      ProtocolData::BoundedValue(0.0, 1275.0, adc_cmd_steerwheelanglelimit);
  int x = static_cast<int>(adc_cmd_steerwheelanglelimit / 5.000000);

  Byte to_set(data + 2);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
