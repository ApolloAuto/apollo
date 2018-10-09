/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Vehiclespeedrpt6f : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vehiclespeedrpt6f();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'VEHICLE_SPEED', 'offset': 0.0, 'precision': 0.01,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
  // 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
  double vehicle_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VEHICLE_SPEED_VALID', 'enum': {0:
  // 'VEHICLE_SPEED_VALID_INVALID', 1: 'VEHICLE_SPEED_VALID_VALID'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Vehicle_speed_rpt_6f::Vehicle_speed_validType vehicle_speed_valid(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
