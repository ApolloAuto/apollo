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
 * @file num_76b.h
 * @brief the class of 76b (for mobileye)
 */

#ifndef MODULES_DRIVERS_MOBILEYE_PROTOCOL_NUM_76B_H
#define MODULES_DRIVERS_MOBILEYE_PROTOCOL_NUM_76B_H

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

class Num76b : public ::apollo::drivers::canbus::ProtocolData<Mobileye> {
 public:
  static const int ID;

  void Parse(const uint8_t* bytes, int32_t length,
             Mobileye* mobileye) const override;

  // config detail: {'name': 'num_of_next_lane_mark_reported', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '"Number  apollo  of  apollo  next  apollo  lane  apollo
  // markers"'}
  int num_of_next_lane_mark_reported(const uint8_t* bytes,
                                     int32_t length) const;
};

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_PROTOCOL_NUM_76B_H
