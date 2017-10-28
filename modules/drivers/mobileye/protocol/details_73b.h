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
 * @file details_73b.h
 * @brief the class of 73b (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_73B_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_73B_H

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

class Details73b : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'obstacle_angle_rate', 'offset': 0.0, 'precision':
  // 0.01, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-327.68|327.67]', 'bit': 0, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"degree"'}
  double obstacle_angle_rate(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_scale_change', 'offset': 0.0,
  // 'precision': 0.0002, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-6.5536|6.5534]', 'bit': 16, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"pix/sec"'}
  double obstacle_scale_change(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'object_accel_x', 'offset': 0.0, 'precision': 0.03,
  // 'len': 10, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[-15.36|15.33]', 'bit': 32, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '"m/S2"'}
  double object_accel_x(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'reserved_8', 'offset': 0.0, 'precision': 1.0,
  // 'len': 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 42, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int reserved_8(const uint8_t* bytes, int32_t length) const;

  bool is_obstacle_replaced(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'reserved_9', 'offset': 0.0, 'precision': 1.0,
  // 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 45, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int reserved_9(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'obstacle_angle', 'offset': 0.0, 'precision': 0.01,
  // 'len': 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[-327.68|327.67]', 'bit': 48, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '"degree"'}
  double obstacle_angle(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_73B_H
