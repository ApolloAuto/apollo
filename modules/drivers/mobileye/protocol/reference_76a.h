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

/**
 * @file reference_76a.h
 * @brief the class of 76a (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_REFERENCE_76A_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_REFERENCE_76A_H

#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::drivers::mobileye
 * @brief apollo::drivers::mobileye
 */

namespace apollo {
namespace drivers {
namespace mobileye {

using ::apollo::drivers::Mobileye;

class Reference76a : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'ref_point_1_position', 'offset': -127.99609375,
  // 'precision': 0.00390625, 'len': 16, 'f_type': 'value', 'is_signed_var':
  // False, 'physical_range': '[-127.99609375|128]', 'bit': 0, 'type': 'double',
  // 'order': 'intel', 'physical_unit': '"meters"'}
  double ref_point_1_position(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'ref_point_1_distance', 'offset': 0.0, 'precision':
  // 0.00390625, 'len': 15, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|127.99609375]', 'bit': 16, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters"'}
  double ref_point_1_distance(const uint8_t* bytes, int32_t length) const;

  bool is_ref_point_1_validity(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'ref_point_2_position', 'offset': -127.99609375,
  // 'precision': 0.00390625, 'len': 16, 'f_type': 'value', 'is_signed_var':
  // False, 'physical_range': '[-127.99609375|128]', 'bit': 32, 'type':
  // 'double', 'order': 'intel', 'physical_unit': '"meters"'}
  double ref_point_2_position(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'ref_point_2_distance', 'offset': 0.0, 'precision':
  // 0.00390625, 'len': 15, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|127.99609375]', 'bit': 48, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters"'}
  double ref_point_2_distance(const uint8_t* bytes, int32_t length) const;

  bool is_ref_point_2_validity(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_REFERENCE_76A_H
