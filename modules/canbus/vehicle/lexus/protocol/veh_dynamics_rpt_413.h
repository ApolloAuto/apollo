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
namespace lexus {

class Vehdynamicsrpt413 : public ::apollo::drivers::canbus::ProtocolData<
                              ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vehdynamicsrpt413();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'VEH_G_FORCES', 'offset': 0.0, 'precision': 0.001,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[-32.768|32.767]',
  // 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double veh_g_forces(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
