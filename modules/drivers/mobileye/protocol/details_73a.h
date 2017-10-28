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
 * @file details_73a.h
 * @brief the class of 73a (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_73A_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_73A_H

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

class Details73a : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'obstacle_length', 'offset': 0.0, 'precision': 0.5,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|31]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit':
  // '"meters"'}
  double obstacle_length(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_width', 'offset': 0.0, 'precision': 0.05,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|12.75]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit':
  // '"meters"'}
  double obstacle_width(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_age', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '""'}
  int obstacle_age(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_lane', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int obstacle_lane(const uint8_t* bytes, int32_t length) const;

  bool is_cipv_flag(const uint8_t* bytes, int32_t length) const;

  bool is_reserved_5(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'radar_pos_x', 'offset': 0.0, 'precision': 0.0625,
  // 'len': 12, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255.9375]', 'bit': 28, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '"meters"'}
  double radar_pos_x(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'radar_vel_x', 'offset': 0.0, 'precision': 0.0625,
  // 'len': 12, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[-128|127.9375]', 'bit': 40, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '"meters/sec"'}
  double radar_vel_x(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'radar_match_confidence', 'offset': 0.0,
  // 'precision': 1.0, 'len': 3, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|7]', 'bit': 52, 'type': 'int', 'order': 'intel',
  // 'physical_unit': '""'}
  int radar_match_confidence(const uint8_t* bytes, int32_t length) const;

  bool is_reserved_6(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'matched_radar_id', 'offset': 0.0, 'precision':
  // 1.0, 'len': 7, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|127]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '""'}
  int matched_radar_id(const uint8_t* bytes, int32_t length) const;

  bool is_reserved_7(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_73A_H
