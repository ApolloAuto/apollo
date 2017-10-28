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

#include "modules/drivers/delphi_esr/protocol/vehicle5_5f4.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Vehicle55f4::Vehicle55f4() {}
const int32_t Vehicle55f4::ID = 0x5F4;

void Vehicle55f4::Parse(const std::uint8_t* bytes, int32_t length,
                        DelphiESR* delphi_esr) const {
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_yaw_rate_bias_shift(
      can_rx_yaw_rate_bias_shift(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_steering_gear_ratio(
      can_rx_steering_gear_ratio(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_wheelbase(
      can_rx_wheelbase(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_distance_rear_axle(
      can_rx_distance_rear_axle(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_cw_blockage_threshold(
      can_rx_cw_blockage_threshold(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_funnel_offset_right(
      can_rx_funnel_offset_right(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_funnel_offset_left(
      can_rx_funnel_offset_left(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_beamwidth_vert(
      can_rx_beamwidth_vert(bytes, length));
  delphi_esr->mutable_vehicle5_5f4()->set_can_rx_oversteer_understeer(
      can_rx_oversteer_understeer(bytes, length));
}

// config detail: {'name': 'can_rx_yaw_rate_bias_shift', 'enum': {0:
// 'CAN_RX_YAW_RATE_BIAS_SHIFT_NO_DETECT', 1:
// 'CAN_RX_YAW_RATE_BIAS_SHIFT_DETECT'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 15,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle5_5f4::Can_rx_yaw_rate_bias_shiftType
Vehicle55f4::can_rx_yaw_rate_bias_shift(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Vehicle5_5f4::Can_rx_yaw_rate_bias_shiftType ret =
      static_cast<Vehicle5_5f4::Can_rx_yaw_rate_bias_shiftType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_steering_gear_ratio', 'offset': 0.0,
// 'precision': 0.125, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|31.875]', 'bit': 63, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Vehicle55f4::can_rx_steering_gear_ratio(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.125000;
  return ret;
}

// config detail: {'name': 'can_rx_wheelbase', 'offset': 200.0, 'precision':
// 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[200|710]', 'bit':
// 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Vehicle55f4::can_rx_wheelbase(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000 + 200.000000;
  return ret;
}

// config detail: {'name': 'can_rx_distance_rear_axle', 'offset': 200.0,
// 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[200|710]', 'bit': 47, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'cm'}
double Vehicle55f4::can_rx_distance_rear_axle(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000 + 200.000000;
  return ret;
}

// config detail: {'name': 'can_rx_cw_blockage_threshold', 'offset': 0.0,
// 'precision': 0.0078125, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|1.9921875]', 'bit': 39, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Vehicle55f4::can_rx_cw_blockage_threshold(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.007812;
  return ret;
}

// config detail: {'name': 'can_rx_funnel_offset_right', 'offset': 0.0,
// 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-2|10]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm'}
double Vehicle55f4::can_rx_funnel_offset_right(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'can_rx_funnel_offset_left', 'offset': 0.0,
// 'precision': 0.1, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-2|10]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm'}
double Vehicle55f4::can_rx_funnel_offset_left(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'can_rx_beamwidth_vert', 'offset': 0.0, 'precision':
// 0.0625, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|6]', 'bit':
// 14, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Vehicle55f4::can_rx_beamwidth_vert(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 7);

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_rx_oversteer_understeer', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-128|127]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit':
// '%'}
int Vehicle55f4::can_rx_oversteer_understeer(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
