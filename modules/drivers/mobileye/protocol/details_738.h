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
 * @file details_738.h
 * @brief the class of 738 (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_738_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_738_H

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

class Details738 : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t *bytes, int32_t length,
             Mobileye *mobileye) const override;

  // config detail: {'name': 'num_obstacles', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '""'}
  int num_obstacles(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'timestamp', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '"ms"'}
  int timestamp(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'application_version', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '""'}
  int application_version(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'active_version_number_section', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|3]', 'bit': 24, 'type': 'int', 'order': 'intel',
  // 'physical_unit': '""'}
  int active_version_number_section(const uint8_t *bytes, int32_t length) const;

  bool is_left_close_rang_cut_in(const uint8_t *bytes, int32_t length) const;

  bool is_right_close_rang_cut_in(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'go', 'offset': 0.0, 'precision': 1.0, 'len': 4,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 28, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int go(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'protocol_version', 'offset': 0.0, 'precision':
  // 1.0, 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '""'}
  int protocol_version(const uint8_t *bytes, int32_t length) const;

  bool is_close_car(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'failsafe', 'offset': 0.0, 'precision': 1.0, 'len':
  // 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 41, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int failsafe(const uint8_t *bytes, int32_t length) const;

  // config detail: {'name': 'reserved_10', 'offset': 0.0, 'precision': 1.0,
  // 'len': 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|0]', 'bit': 45, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int reserved_10(const uint8_t *bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_738_H
