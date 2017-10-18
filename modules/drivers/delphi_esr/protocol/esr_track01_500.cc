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

#include "modules/drivers/delphi_esr/protocol/esr_track01_500.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrtrack01500::Esrtrack01500() {}
const int32_t Esrtrack01500::ID = 0x500;

void Esrtrack01500::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  auto* esr_track01_500 = delphi_esr->add_esr_track01_500();
  esr_track01_500->set_can_tx_track_grouping_changed(
      can_tx_track_grouping_changed(bytes, length));
  esr_track01_500->set_can_tx_track_oncoming(
      can_tx_track_oncoming(bytes, length));
  esr_track01_500->set_can_tx_track_lat_rate(
      can_tx_track_lat_rate(bytes, length));
  esr_track01_500->set_can_tx_track_bridge_object(
      can_tx_track_bridge_object(bytes, length));
  esr_track01_500->set_can_tx_track_width(can_tx_track_width(bytes, length));
  esr_track01_500->set_can_tx_track_status(can_tx_track_status(bytes, length));
  esr_track01_500->set_can_tx_track_rolling_count(
      can_tx_track_rolling_count(bytes, length));
  esr_track01_500->set_can_tx_track_range_rate(
      can_tx_track_range_rate(bytes, length));
  esr_track01_500->set_can_tx_track_range_accel(
      can_tx_track_range_accel(bytes, length));
  esr_track01_500->set_can_tx_track_range(can_tx_track_range(bytes, length));
  esr_track01_500->set_can_tx_track_med_range_mode(
      can_tx_track_med_range_mode(bytes, length));
  esr_track01_500->set_can_tx_track_angle(can_tx_track_angle(bytes, length));
}

// config detail: {'name': 'can_tx_track_grouping_changed', 'enum': {0:
// 'CAN_TX_TRACK_GROUPING_CHANGED_GROUPINGUNCHANGED', 1:
// 'CAN_TX_TRACK_GROUPING_CHANGED_GROUPINGCHANGED'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 1,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_track01_500::Can_tx_track_grouping_changedType
Esrtrack01500::can_tx_track_grouping_changed(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Esr_track01_500::Can_tx_track_grouping_changedType ret =
      static_cast<Esr_track01_500::Can_tx_track_grouping_changedType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_track_oncoming', 'enum': {0:
// 'CAN_TX_TRACK_ONCOMING_NOTONCOMING', 1: 'CAN_TX_TRACK_ONCOMING_ONCOMING'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Esr_track01_500::Can_tx_track_oncomingType Esrtrack01500::can_tx_track_oncoming(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Esr_track01_500::Can_tx_track_oncomingType ret =
      static_cast<Esr_track01_500::Can_tx_track_oncomingType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_track_lat_rate', 'offset': 0.0, 'precision':
// 0.25, 'len': 6, 'is_signed_var': True, 'physical_range': '[-8|7.75]', 'bit':
// 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Esrtrack01500::can_tx_track_lat_rate(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 6);

  x <<= 26;
  x >>= 26;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'can_tx_track_bridge_object', 'enum': {0:
// 'CAN_TX_TRACK_BRIDGE_OBJECT_NOT_BRIDGE', 1:
// 'CAN_TX_TRACK_BRIDGE_OBJECT_BRIDGE'}, 'precision': 1.0, 'len': 1,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_track01_500::Can_tx_track_bridge_objectType
Esrtrack01500::can_tx_track_bridge_object(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Esr_track01_500::Can_tx_track_bridge_objectType ret =
      static_cast<Esr_track01_500::Can_tx_track_bridge_objectType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_track_width', 'offset': 0.0, 'precision':
// 0.5, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|7.5]', 'bit':
// 37, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Esrtrack01500::can_tx_track_width(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 4);

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'name': 'can_tx_track_status', 'enum': {0:
// 'CAN_TX_TRACK_STATUS_NO_TARGET', 1: 'CAN_TX_TRACK_STATUS_NEW_TARGET', 2:
// 'CAN_TX_TRACK_STATUS_NEW_UPDATED_TARGET', 3:
// 'CAN_TX_TRACK_STATUS_UPDATED_TARGET', 4:
// 'CAN_TX_TRACK_STATUS_COASTED_TARGET', 5: 'CAN_TX_TRACK_STATUS_MERGED_TARGET',
// 6: 'CAN_TX_TRACK_STATUS_INVALID_COASTED_TARGET', 7:
// 'CAN_TX_TRACK_STATUS_NEW_COASTED_TARGET'}, 'precision': 1.0, 'len': 3,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_track01_500::Can_tx_track_statusType Esrtrack01500::can_tx_track_status(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 3);

  Esr_track01_500::Can_tx_track_statusType ret =
      static_cast<Esr_track01_500::Can_tx_track_statusType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_track_rolling_count', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 38, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Esrtrack01500::can_tx_track_rolling_count(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_track_range_rate', 'offset': 0.0,
// 'precision': 0.01, 'len': 14, 'is_signed_var': True, 'physical_range':
// '[-81.92|81.91]', 'bit': 53, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'm/s'}
double Esrtrack01500::can_tx_track_range_rate(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 18;
  x >>= 18;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'can_tx_track_range_accel', 'offset': 0.0,
// 'precision': 0.05, 'len': 10, 'is_signed_var': True, 'physical_range':
// '[-25.6|25.55]', 'bit': 33, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'm/s/s'}
double Esrtrack01500::can_tx_track_range_accel(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 22;
  x >>= 22;

  double ret = x * 0.050000;
  return ret;
}

// config detail: {'name': 'can_tx_track_range', 'offset': 0.0, 'precision':
// 0.1, 'len': 11, 'is_signed_var': False, 'physical_range': '[0|204.7]', 'bit':
// 18, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Esrtrack01500::can_tx_track_range(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'can_tx_track_med_range_mode', 'enum': {0:
// 'CAN_TX_TRACK_MED_RANGE_MODE_NO_MR_LR_UPDATE', 1:
// 'CAN_TX_TRACK_MED_RANGE_MODE_MR_UPDATE_ONLY', 2:
// 'CAN_TX_TRACK_MED_RANGE_MODE_LR_UPDATE_ONLY', 3:
// 'CAN_TX_TRACK_MED_RANGE_MODE_BOTH_MR_LR_UPDATE'}, 'precision': 1.0, 'len': 2,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_track01_500::Can_tx_track_med_range_modeType
Esrtrack01500::can_tx_track_med_range_mode(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Esr_track01_500::Can_tx_track_med_range_modeType ret =
      static_cast<Esr_track01_500::Can_tx_track_med_range_modeType>(x);
  return ret;
}

// config detail: {'name': 'can_tx_track_angle', 'offset': 0.0, 'precision':
// 0.1, 'len': 10, 'is_signed_var': True, 'physical_range': '[-51.2|51.1]',
// 'bit': 12, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Esrtrack01500::can_tx_track_angle(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  x <<= 22;
  x >>= 22;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
