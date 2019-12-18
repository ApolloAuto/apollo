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

#include "modules/canbus/vehicle/ch/protocol/ecu_status_2_516.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Ecustatus2516::Ecustatus2516() {}
const int32_t Ecustatus2516::ID = 0x516;

void Ecustatus2516::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_ch()
      ->mutable_ecu_status_2_516()
      ->set_battery_remaining_capacity(
          battery_remaining_capacity(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_2_516()->set_battery_voltage(
      battery_voltage(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_2_516()->set_battery_current(
      battery_current(bytes, length));
  chassis->mutable_ch()->mutable_ecu_status_2_516()->set_battery_temperature(
      battery_temperature(bytes, length));
}

// config detail: {'description': 'Percentage of battery remaining (BMS
// status)', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name':
// 'battery_remaining_capacity', 'is_signed_var': False, 'physical_range':
// '[0|100]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '%'}
int Ecustatus2516::battery_remaining_capacity(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'description': 'Current battery voltage (BMS status)',
// 'offset': 0.0, 'precision': 0.1, 'len': 16, 'name': 'battery_voltage',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'V'}
double Ecustatus2516::battery_voltage(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'Current battery current (BMS status)',
// 'offset': 0.0, 'precision': 0.1, 'len': 16, 'name': 'battery_current',
// 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 32, 'type':
// 'double', 'order': 'intel', 'physical_unit': 'A'}
double Ecustatus2516::battery_current(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'Current battery temperature (BMS status)',
// 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name': 'battery_temperature',
// 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 48, 'type': 'int',
// 'order': 'intel', 'physical_unit': '?'}
int Ecustatus2516::battery_temperature(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
