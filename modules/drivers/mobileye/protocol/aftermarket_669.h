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
 * @file aftermarket_669.h
 * @brief the class of 669 (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_AFTERMARKET_669_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_AFTERMARKET_669_H

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

class Aftermarket669 : public apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'lane_conf_left', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[0|3]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int lane_conf_left(const uint8_t* bytes, int32_t length) const;

  bool is_ldw_availability_left(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'lane_type_left', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[0|6]', 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int lane_type_left(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'distance_to_lane_l', 'offset': 0.0, 'precision':
  // 0.02, 'len': 12, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-40|40]', 'bit': 12, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters"'}
  double distance_to_lane_l(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'lane_conf_right', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[0|3]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int lane_conf_right(const uint8_t* bytes, int32_t length) const;

  bool is_ldw_availability_right(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'lane_type_right', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[0|6]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int lane_type_right(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'distance_to_lane_r', 'offset': 0.0, 'precision':
  // 0.02, 'len': 12, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-40|40]', 'bit': 52, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters"'}
  double distance_to_lane_r(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_AFTERMARKET_669_H
