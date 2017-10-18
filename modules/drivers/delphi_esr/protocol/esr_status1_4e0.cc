/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/delphi_esr/protocol/esr_status1_4e0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus14e0::Esrstatus14e0() {}
const int32_t Esrstatus14e0::ID = 0x4E0;

void Esrstatus14e0::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_dsp_timestamp(
      can_tx_dsp_timestamp(bytes, length));
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_comm_error(
      can_tx_comm_error(bytes, length));
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_yaw_rate_calc(
      can_tx_yaw_rate_calc(bytes, length));
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_vehicle_speed_calc(
      can_tx_vehicle_speed_calc(bytes, length));
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_scan_index(
      can_tx_scan_index(bytes, length));
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_rolling_count_1(
      can_tx_rolling_count_1(bytes, length));
  delphi_esr->mutable_esr_status1_4e0()->set_can_tx_radius_curvature_calc(
      can_tx_radius_curvature_calc(bytes, length));
}

// config detail: {'name': 'can_tx_dsp_timestamp', 'offset': 0.0, 'precision':
// 2.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 5,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'ms'}
double Esrstatus14e0::can_tx_dsp_timestamp(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'can_tx_comm_error', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 14,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Esrstatus14e0::can_tx_comm_error(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_yaw_rate_calc', 'offset': 0.0, 'precision':
// 0.0625, 'len': 12, 'is_signed_var': True, 'physical_range':
// '[-128|127.9375]', 'bit': 47, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'deg/s'}
double Esrstatus14e0::can_tx_yaw_rate_calc(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_tx_vehicle_speed_calc', 'offset': 0.0,
// 'precision': 0.0625, 'len': 11, 'is_signed_var': False, 'physical_range':
// '[0|127.9375]', 'bit': 50, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'm/s'}
double Esrstatus14e0::can_tx_vehicle_speed_calc(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_tx_scan_index', 'offset': 0.0, 'precision': 1.0,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 31,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus14e0::can_tx_scan_index(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_rolling_count_1', 'offset': 0.0, 'precision':
// 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus14e0::can_tx_rolling_count_1(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_radius_curvature_calc', 'offset': 0.0,
// 'precision': 1.0, 'len': 14, 'is_signed_var': True, 'physical_range':
// '[-8192|8191]', 'bit': 13, 'type': 'int', 'order': 'motorola',
// 'physical_unit': 'm'}
int Esrstatus14e0::can_tx_radius_curvature_calc(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 18;
  x >>= 18;

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
