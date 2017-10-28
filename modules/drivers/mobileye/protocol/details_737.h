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
 * @file details_737.h
 * @brief the class of 737 (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_737_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_737_H

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

class Details737 : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'lane_curvature', 'offset': 0.0, 'precision':
  // 3.81e-06, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-0.12484608|0.12484227]', 'bit': 0, 'type': 'double',
  // 'order': 'intel', 'physical_unit': '"1/meters"'}
  double lane_curvature(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'lane_heading', 'offset': 0.0, 'precision': 0.0005,
  // 'len': 12, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
  // '[-1|1]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit':
  // '""'}
  double lane_heading(const uint8_t* bytes, int32_t length) const;

  bool is_ca_construction_area(const uint8_t* bytes, int32_t length) const;

  bool is_right_ldw_availability(const uint8_t* bytes, int32_t length) const;

  bool is_left_ldw_availability(const uint8_t* bytes, int32_t length) const;

  bool is_reserved_1(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'yaw_angle', 'offset': 0.0, 'precision':
  // 0.0009765625, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-32|31.999023438]', 'bit': 32, 'type': 'double',
  // 'order': 'intel', 'physical_unit': '"radians"'}
  double yaw_angle(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'pitch_angle', 'offset': 0.0, 'precision':
  // 1.90734e-06, 'len': 16, 'f_type': 'value', 'is_signed_var': True,
  // 'physical_range': '[-0.06249971712|0.06249780978]', 'bit': 48, 'type':
  // 'double', 'order': 'intel', 'physical_unit': '"radians"'}
  double pitch_angle(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_DETAILS_737_H
