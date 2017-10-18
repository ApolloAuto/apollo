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

#include "modules/drivers/delphi_esr/protocol/esr_sim1_5c0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrsim15c0::Esrsim15c0() {}
const int32_t Esrsim15c0::ID = 0x5C0;

void Esrsim15c0::Parse(const std::uint8_t* bytes, int32_t length,
                       DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_track_id(
      can_rx_sim_track_id(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_status(
      can_rx_sim_status(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_range_rate(
      can_rx_sim_range_rate(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_range_accel(
      can_rx_sim_range_accel(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_range(
      can_rx_sim_range(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_lat_rate(
      can_rx_sim_lat_rate(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_lat_pos(
      can_rx_sim_lat_pos(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_function(
      can_rx_sim_function(bytes, length));
  delphi_esr->mutable_esr_sim1_5c0()->set_can_rx_sim_angle(
      can_rx_sim_angle(bytes, length));
}

// config detail: {'name': 'can_rx_sim_track_id', 'enum': {0:
// 'CAN_RX_SIM_TRACK_ID_NO_TARGET', 1: 'CAN_RX_SIM_TRACK_ID_TARGET_1', 2:
// 'CAN_RX_SIM_TRACK_ID_TARGET_2'}, 'precision': 1.0, 'len': 2, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 6, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Esr_sim1_5c0::Can_rx_sim_track_idType Esrsim15c0::can_rx_sim_track_id(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 2);

  Esr_sim1_5c0::Can_rx_sim_track_idType ret =
      static_cast<Esr_sim1_5c0::Can_rx_sim_track_idType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_sim_status', 'enum': {0:
// 'CAN_RX_SIM_STATUS_INVALID', 1: 'CAN_RX_SIM_STATUS_NEW', 2:
// 'CAN_RX_SIM_STATUS_UPDATED', 3: 'CAN_RX_SIM_STATUS_COASTED'}, 'precision':
// 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|0]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_sim1_5c0::Can_rx_sim_statusType Esrsim15c0::can_rx_sim_status(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 2);

  Esr_sim1_5c0::Can_rx_sim_statusType ret =
      static_cast<Esr_sim1_5c0::Can_rx_sim_statusType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_sim_range_rate', 'offset': 0.0, 'precision':
// 0.25, 'len': 8, 'is_signed_var': True, 'physical_range': '[-32|31.75]',
// 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Esrsim15c0::can_rx_sim_range_rate(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'can_rx_sim_range_accel', 'offset': 0.0, 'precision':
// 0.25, 'len': 8, 'is_signed_var': True, 'physical_range': '[-32|31.75]',
// 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s/s'}
double Esrsim15c0::can_rx_sim_range_accel(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'can_rx_sim_range', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39,
// 'type': 'int', 'order': 'motorola', 'physical_unit': 'm'}
int Esrsim15c0::can_rx_sim_range(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_sim_lat_rate', 'offset': 0.0, 'precision':
// 0.25, 'len': 8, 'is_signed_var': True, 'physical_range': '[-32|31.75]',
// 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Esrsim15c0::can_rx_sim_lat_rate(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'can_rx_sim_lat_pos', 'offset': 0.0, 'precision':
// 0.25, 'len': 8, 'is_signed_var': True, 'physical_range': '[-32|31.75]',
// 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Esrsim15c0::can_rx_sim_lat_pos(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'can_rx_sim_function', 'enum': {0:
// 'CAN_RX_SIM_FUNCTION_ACC', 1: 'CAN_RX_SIM_FUNCTION_RI', 2:
// 'CAN_RX_SIM_FUNCTION_FCW_MOVE', 3: 'CAN_RX_SIM_FUNCTION_FCW_STAT', 4:
// 'CAN_RX_SIM_FUNCTION_CMBB_MOVE', 5: 'CAN_RX_SIM_FUNCTION_CMBB_STAT', 6:
// 'CAN_RX_SIM_FUNCTION_ALL_MOVING_ACC_FCW_CMBB', 7:
// 'CAN_RX_SIM_FUNCTION_ALL_STAT_RI_FCW_CMBB'}, 'precision': 1.0, 'len': 3,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 2,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esr_sim1_5c0::Can_rx_sim_functionType Esrsim15c0::can_rx_sim_function(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 3);

  Esr_sim1_5c0::Can_rx_sim_functionType ret =
      static_cast<Esr_sim1_5c0::Can_rx_sim_functionType>(x);
  return ret;
}

// config detail: {'name': 'can_rx_sim_angle', 'offset': 0.0, 'precision': 0.5,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-64|63.5]', 'bit': 15,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Esrsim15c0::can_rx_sim_angle(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.500000;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
