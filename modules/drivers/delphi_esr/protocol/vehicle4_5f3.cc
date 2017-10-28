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

#include "modules/drivers/delphi_esr/protocol/vehicle4_5f3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Vehicle45f3::Vehicle45f3() {}
const int32_t Vehicle45f3::ID = 0x5F3;

void Vehicle45f3::Parse(const std::uint8_t* bytes, int32_t length,
                        DelphiESR* delphi_esr) const {
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_tgt_range_r2m(
      can_rx_fac_tgt_range_r2m(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_tgt_range_m2t(
      can_rx_fac_tgt_range_m2t(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_tgt_range_1(
      can_rx_fac_tgt_range_1(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_tgt_mtg_space_ver(
      can_rx_fac_tgt_mtg_space_ver(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_tgt_mtg_space_hor(
      can_rx_fac_tgt_mtg_space_hor(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_tgt_mtg_offset(
      can_rx_fac_tgt_mtg_offset(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_align_samp_req(
      can_rx_fac_align_samp_req(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_align_max_nt(
      can_rx_fac_align_max_nt(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_align_cmd_2(
      can_rx_fac_align_cmd_2(bytes, length));
  delphi_esr->mutable_vehicle4_5f3()->set_can_rx_fac_align_cmd_1(
      can_rx_fac_align_cmd_1(bytes, length));
}

// config detail: {'name': 'can_rx_fac_tgt_range_r2m', 'offset': 0.0,
// 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[2|10]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm'}
double Vehicle45f3::can_rx_fac_tgt_range_r2m(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_rx_fac_tgt_range_m2t', 'offset': 0.0,
// 'precision': 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[1|10]', 'bit': 63, 'type': 'double', 'order': 'motorola', 'physical_unit':
// 'm'}
double Vehicle45f3::can_rx_fac_tgt_range_m2t(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_rx_fac_tgt_range_1', 'offset': 0.0, 'precision':
// 0.0625, 'len': 8, 'is_signed_var': False, 'physical_range': '[2|10]', 'bit':
// 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Vehicle45f3::can_rx_fac_tgt_range_1(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'can_rx_fac_tgt_mtg_space_ver', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-100|100]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'cm'}
int Vehicle45f3::can_rx_fac_tgt_mtg_space_ver(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_fac_tgt_mtg_space_hor', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-100|100]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'cm'}
int Vehicle45f3::can_rx_fac_tgt_mtg_space_hor(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_fac_tgt_mtg_offset', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
// '[-100|100]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit':
// 'cm'}
int Vehicle45f3::can_rx_fac_tgt_mtg_offset(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_fac_align_samp_req', 'offset': 0.0,
// 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range':
// '[0|100]', 'bit': 14, 'type': 'int', 'order': 'motorola', 'physical_unit':
// ''}
int Vehicle45f3::can_rx_fac_align_samp_req(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 7);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_fac_align_max_nt', 'offset': 0.0,
// 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range':
// '[0|100]', 'bit': 6, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vehicle45f3::can_rx_fac_align_max_nt(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 7);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_rx_fac_align_cmd_2', 'enum': {0:
// 'CAN_RX_FAC_ALIGN_CMD_2_OFF', 1: 'CAN_RX_FAC_ALIGN_CMD_2_ON'}, 'precision':
// 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|0]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle4_5f3::Can_rx_fac_align_cmd_2Type Vehicle45f3::can_rx_fac_align_cmd_2(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Vehicle4_5f3::Can_rx_fac_align_cmd_2Type ret =
      static_cast<Vehicle4_5f3::Can_rx_fac_align_cmd_2Type>(x);
  return ret;
}

// config detail: {'name': 'can_rx_fac_align_cmd_1', 'enum': {0:
// 'CAN_RX_FAC_ALIGN_CMD_1_OFF', 1: 'CAN_RX_FAC_ALIGN_CMD_1_ON'}, 'precision':
// 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|0]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Vehicle4_5f3::Can_rx_fac_align_cmd_1Type Vehicle45f3::can_rx_fac_align_cmd_1(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Vehicle4_5f3::Can_rx_fac_align_cmd_1Type ret =
      static_cast<Vehicle4_5f3::Can_rx_fac_align_cmd_1Type>(x);
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
