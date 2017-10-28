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

#include "modules/drivers/delphi_esr/protocol/esr_status9_5e8.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrstatus95e8::Esrstatus95e8() {}
const int32_t Esrstatus95e8::ID = 0x5E8;

void Esrstatus95e8::Parse(const std::uint8_t* bytes, int32_t length,
                          DelphiESR* delphi_esr) const {
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_path_id_acc_3(
      can_tx_path_id_acc_3(bytes, length));
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_path_id_acc_2(
      can_tx_path_id_acc_2(bytes, length));
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_filtered_xohp_acc_cipv(
      can_tx_filtered_xohp_acc_cipv(bytes, length));
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_water_spray_target_id(
      can_tx_water_spray_target_id(bytes, length));
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_serial_num_3rd_byte(
      can_tx_serial_num_3rd_byte(bytes, length));
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_sideslip_angle(
      can_tx_sideslip_angle(bytes, length));
  delphi_esr->mutable_esr_status9_5e8()->set_can_tx_avg_pwr_cwblkg(
      can_tx_avg_pwr_cwblkg(bytes, length));
}

// config detail: {'name': 'can_tx_path_id_acc_3', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|64]', 'bit': 63,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus95e8::can_tx_path_id_acc_3(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_path_id_acc_2', 'offset': 0.0, 'precision':
// 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|64]', 'bit': 55,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus95e8::can_tx_path_id_acc_2(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_filtered_xohp_acc_cipv', 'offset': 0.0,
// 'precision': 0.03125, 'len': 9, 'is_signed_var': True, 'physical_range':
// '[-8|7.96875]', 'bit': 32, 'type': 'double', 'order': 'motorola',
// 'physical_unit': 'm'}
double Esrstatus95e8::can_tx_filtered_xohp_acc_cipv(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 23;
  x >>= 23;

  double ret = x * 0.031250;
  return ret;
}

// config detail: {'name': 'can_tx_water_spray_target_id', 'offset': 0.0,
// 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range':
// '[0|64]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus95e8::can_tx_water_spray_target_id(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 7);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_serial_num_3rd_byte', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus95e8::can_tx_serial_num_3rd_byte(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'can_tx_sideslip_angle', 'offset': 0.0, 'precision':
// 0.125, 'len': 10, 'is_signed_var': True, 'physical_range': '[-64|63.875]',
// 'bit': 9, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Esrstatus95e8::can_tx_sideslip_angle(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 22;
  x >>= 22;

  double ret = x * 0.125000;
  return ret;
}

// config detail: {'name': 'can_tx_avg_pwr_cwblkg', 'offset': 0.0, 'precision':
// 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7,
// 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esrstatus95e8::can_tx_avg_pwr_cwblkg(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
