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
 * @file lka_767.h
 * @brief the class of 767 (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_LKA_767_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_LKA_767_H

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

class Lka767 : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'heading_angle', 'offset': -31.9990234375,
  // 'precision': 0.0009765625, 'len': 16, 'f_type': 'value', 'is_signed_var':
  // False, 'physical_range': '[-0.357|0.357]', 'bit': 0, 'type': 'double',
  // 'order': 'intel', 'physical_unit': '"radians"'}
  double heading_angle(const uint8_t* bytes, int32_t length) const;

  // config detail: {'name': 'view_range', 'offset': 0.0, 'precision':
  // 0.00390625, 'len': 15, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|127.99609375]', 'bit': 16, 'type': 'double', 'order':
  // 'intel', 'physical_unit': '"meter"'}
  double view_range(const uint8_t* bytes, int32_t length) const;

  bool is_view_range_availability(const uint8_t* bytes, int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_LKA_767_H
