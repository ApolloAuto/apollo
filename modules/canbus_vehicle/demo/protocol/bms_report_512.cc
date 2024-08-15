/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/protocol/bms_report_512.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace demo {

using ::apollo::drivers::canbus::Byte;

Bmsreport512::Bmsreport512() {}
const int32_t Bmsreport512::ID = 0x512;

void Bmsreport512::Parse(const std::uint8_t* bytes, int32_t length,
                         Demo* chassis) const {
  chassis->mutable_bms_report_512()->set_battery_flt_overtemp(
      battery_flt_overtemp(bytes, length));
  chassis->mutable_bms_report_512()->set_battery_flt_lowtemp(
      battery_flt_lowtemp(bytes, length));
  chassis->mutable_bms_report_512()->set_battery_inside_temperature(
      battery_inside_temperature(bytes, length));
  chassis->mutable_bms_report_512()->set_battery_current(
      battery_current(bytes, length));
  chassis->mutable_bms_report_512()->set_battery_voltage(
      battery_voltage(bytes, length));
  chassis->mutable_bms_report_512()->set_battery_soc(
      battery_soc(bytes, length));
}

// config detail: {'bit': 49, 'enum': {0: 'BATTERY_FLT_OVERTEMP_NO', 1:
// 'BATTERY_FLT_OVERTEMP_FAULT'}, 'is_signed_var': False, 'len': 1, 'name':
// 'battery_flt_overtemp', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Bms_report_512::Battery_flt_overtempType Bmsreport512::battery_flt_overtemp(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(1, 1);

  Bms_report_512::Battery_flt_overtempType ret =
      static_cast<Bms_report_512::Battery_flt_overtempType>(x);
  return ret;
}

// config detail: {'bit': 48, 'enum': {0: 'BATTERY_FLT_LOWTEMP_NO', 1:
// 'BATTERY_FLT_LOWTEMP_FAULT'}, 'is_signed_var': False, 'len': 1, 'name':
// 'battery_flt_lowtemp', 'offset': 0.0, 'order': 'motorola', 'physical_range':
// '[0|15]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Bms_report_512::Battery_flt_lowtempType Bmsreport512::battery_flt_lowtemp(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  Bms_report_512::Battery_flt_lowtempType ret =
      static_cast<Bms_report_512::Battery_flt_lowtempType>(x);
  return ret;
}

// config detail: {'bit': 47, 'description': 'Battery Total Voltage',
// 'is_signed_var': False, 'len': 8, 'name': 'battery_inside_temperature',
// 'offset': -40.0, 'order': 'motorola', 'physical_range': '[-40|215]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsreport512::battery_inside_temperature(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 23, 'description': 'Battery Total Current',
// 'is_signed_var': False, 'len': 16, 'name': 'battery_current', 'offset':
// -3200.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]',
// 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Bmsreport512::battery_current(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -3200.000000;
  return ret;
}

// config detail: {'bit': 7, 'description': 'Battery Total Voltage',
// 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0,
// 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V',
// 'precision': 0.01, 'type': 'double'}
double Bmsreport512::battery_voltage(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 39, 'description': 'Battery Soc percentage',
// 'is_signed_var': False, 'len': 8, 'name': 'battery_soc', 'offset': 0.0,
// 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%',
// 'precision': 1.0, 'type': 'int'}
int Bmsreport512::battery_soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace demo
}  // namespace canbus
}  // namespace apollo
