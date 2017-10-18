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

#include "modules/drivers/delphi_esr/protocol/esr_status2_4e1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus24e1::Esrstatus24e1() {}
const int32_t Esrstatus24e1::ID = 0x4E1;

void Esrstatus24e1::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_yaw_rate_bias(
      can_tx_yaw_rate_bias(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_veh_spd_comp_factor(
      can_tx_veh_spd_comp_factor(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_sw_version_dsp(
      can_tx_sw_version_dsp(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_temperature(
      can_tx_temperature(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_raw_data_mode(
      can_tx_raw_data_mode(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_range_perf_error(
      can_tx_range_perf_error(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_overheat_error(
      can_tx_overheat_error(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_maximum_tracks_ack(
      can_tx_maximum_tracks_ack(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_internal_error(
      can_tx_internal_error(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_grouping_mode(
      can_tx_grouping_mode(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_xcvr_operational(
      can_tx_xcvr_operational(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_steering_angle_ack(
      can_tx_steering_angle_ack(bytes, length));
  delphi_esr->mutable_esr_status2_4e1()->set_can_tx_rolling_count_2(
      can_tx_rolling_count_2(bytes, length));
}

// config detail: {'name': 'can_tx_yaw_rate_bias', 'offset': 0.0, 'precision':
// 0.125, 'len': 8, 'is_signed_var': True, 'physical_range': '[-16|15.875]',
// 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Esrstatus24e1::can_tx_yaw_rate_bias(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.125000;
  return ret;
}

// config detail: {'name': 'can_tx_veh_spd_comp_factor', 'offset': 1.0,
// 'precision': 0.001953125, 'len': 6, 'is_signed_var': True, 'physical_range':
// '[0.9375|1.060546875]', 'bit': 39, 'type': 'double', 'order': 'motorola',
// 'physical_unit': ''}
double Esrstatus24e1::can_tx_veh_spd_comp_factor(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 6);

  x <<= 26;
  x >>= 26;

  double ret = x * 0.001953 + 1.000000;
  return ret;
}

// config detail: {'name': 'can_tx_sw_version_dsp', 'offset': 0.0, 'precision':
// 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus24e1::can_tx_sw_version_dsp(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_temperature', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': True, 'physical_range': '[-128|127]', 'bit':
// 31, 'type': 'int', 'order': 'motorola', 'physical_unit': 'degC'}
int Esrstatus24e1::can_tx_temperature(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_raw_data_mode', 'enum': {0:
// 'CAN_TX_RAW_DATA_MODE_FILTERED', 1: 'CAN_TX_RAW_DATA_MODE_RAW'}, 'precision':
// 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|0]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status2_4e1::Can_tx_raw_data_modeType Esrstatus24e1::can_tx_raw_data_mode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  Esr_status2_4e1::Can_tx_raw_data_modeType ret =
      static_cast<Esr_status2_4e1::Can_tx_raw_data_modeType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_range_perf_error', 'enum': {0:
// 'CAN_TX_RANGE_PERF_ERROR_NOT_BLOCKED', 1: 'CAN_TX_RANGE_PERF_ERROR_BLOCKED'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 14, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_status2_4e1::Can_tx_range_perf_errorType
Esrstatus24e1::can_tx_range_perf_error(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Esr_status2_4e1::Can_tx_range_perf_errorType ret =
      static_cast<Esr_status2_4e1::Can_tx_range_perf_errorType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_overheat_error', 'enum': {0:
// 'CAN_TX_OVERHEAT_ERROR_NOT_OVERTEMP', 1: 'CAN_TX_OVERHEAT_ERROR_OVERTEMP'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 15, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_status2_4e1::Can_tx_overheat_errorType Esrstatus24e1::can_tx_overheat_error(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Esr_status2_4e1::Can_tx_overheat_errorType ret =
      static_cast<Esr_status2_4e1::Can_tx_overheat_errorType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_maximum_tracks_ack', 'offset': 1.0,
// 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range':
// '[1|64]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus24e1::can_tx_maximum_tracks_ack(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 6);

  int ret = x + 1.000000;
  return ret;
}

// config detail: {'name': 'can_tx_internal_error', 'enum': {0:
// 'CAN_TX_INTERNAL_ERROR_NOT_FAILED', 1: 'CAN_TX_INTERNAL_ERROR_FAILED'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 13, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_status2_4e1::Can_tx_internal_errorType Esrstatus24e1::can_tx_internal_error(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  Esr_status2_4e1::Can_tx_internal_errorType ret =
      static_cast<Esr_status2_4e1::Can_tx_internal_errorType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_grouping_mode', 'enum': {0:
// 'CAN_TX_GROUPING_MODE_NO_GROUPING', 1:
// 'CAN_TX_GROUPING_MODE_GROUP_MOVING_ONLY', 2:
// 'CAN_TX_GROUPING_MODE_GROUP_STATIONARY_ONLY', 3:
// 'CAN_TX_GROUPING_MODE_GROUP_MOVING_STATIONARY'}, 'precision': 1.0, 'len': 2,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 33,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status2_4e1::Can_tx_grouping_modeType Esrstatus24e1::can_tx_grouping_mode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Esr_status2_4e1::Can_tx_grouping_modeType ret =
      static_cast<Esr_status2_4e1::Can_tx_grouping_modeType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_xcvr_operational', 'enum': {0:
// 'CAN_TX_XCVR_OPERATIONAL_OFF', 1: 'CAN_TX_XCVR_OPERATIONAL_ON'}, 'precision':
// 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|0]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_status2_4e1::Can_tx_xcvr_operationalType
Esrstatus24e1::can_tx_xcvr_operational(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Esr_status2_4e1::Can_tx_xcvr_operationalType ret =
      static_cast<Esr_status2_4e1::Can_tx_xcvr_operationalType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_steering_angle_ack', 'offset': 0.0,
// 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range':
// '[0|2047]', 'bit': 10, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'deg'}
int Esrstatus24e1::can_tx_steering_angle_ack(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_rolling_count_2', 'offset': 0.0, 'precision':
// 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 1,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus24e1::can_tx_rolling_count_2(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
