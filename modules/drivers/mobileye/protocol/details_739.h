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
 * @file details_739.h
 * @brief the class of 739 (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_739_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_739_H

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

class Details739 : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'obstacle_id', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|63]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int obstacle_id(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_pos_x', 'offset': 0.0, 'precision':
  // 0.0625, 'len': 12, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|255.9375]', 'bit': 8, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters"'}
  double obstacle_pos_x(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'reseved_2', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 20, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int reseved_2(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_pos_y', 'offset': 0.0, 'precision':
  // 0.0625, 'len': 10, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-32|31.9375]', 'bit': 24, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters"'}
  double obstacle_pos_y(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'blinker_info', 'offset': 0.0, 'precision': 1.0,
  // 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|7]', 'bit': 34, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int blinker_info(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'cut_in_and_out', 'offset': 0.0, 'precision': 1.0,
  // 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|7]', 'bit': 37, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int cut_in_and_out(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_rel_vel_x', 'offset': 0.0, 'precision':
  // 0.0625, 'len': 12, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-128|127.9375]', 'bit': 40, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meters/sec"'}
  double obstacle_rel_vel_x(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_type', 'offset': 0.0, 'precision': 1.0,
  // 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|7]', 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int obstacle_type(const uint8_t* bytes, int32_t length) const;

  bool is_reserved_3(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_status', 'offset': 0.0, 'precision': 1.0,
  // 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|7]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int obstacle_status(const uint8_t* bytes, int32_t length) const;

  bool is_obstacle_brake_lights(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'reserved_4', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[0|0]', 'bit': 60, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int reserved_4(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_valid', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 62, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int obstacle_valid(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_739_H
