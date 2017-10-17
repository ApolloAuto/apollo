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
 * @file lka_768.h
 * @brief the class of 768 (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_LKA_768_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_LKA_768_H

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

class Lka768 : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'lane_type', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int lane_type(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'quality', 'offset': 0.0, 'precision': 1.0, 'len':
  // 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int quality(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'model_degree', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 6, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int model_degree(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'position', 'offset': 0.0, 'precision': 0.00390625,
  // 'len': 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 8, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '"meter"'}
  double position(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'curvature', 'offset': -0.031999023438,
  // 'precision': 9.765625e-07, 'len': 16, 'f_type': 'value', 'is_signed_var':
  // False, 'physical_range': '[-0.02|0.02]', 'bit': 24, 'type': 'double',
  // 'order': 'intel', 'physical_unit': '"1/meter"'}
  double curvature(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'curvature_derivative', 'offset':
  // -0.00012206658721, 'precision': 3.7252902985e-09, 'len': 16, 'f_type':
  // 'value', 'is_signed_var': False, 'physical_range': '[-0.00012|0.00012]',
  // 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit':
  // '"1/meter^2"'}
  double curvature_derivative(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'width_right_marking', 'offset': 0.0, 'precision':
  // 0.01, 'len': 8, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|2.55]', 'bit': 56, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"m"'}
  double width_right_marking(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_LKA_768_H
