/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/wey/proto/wey.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fbs4235 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Wey> {
 public:
  static const int32_t ID;
  Fbs4235();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Wey* chassis) const override;

 private:
  // config detail: {'description': 'angle of steering wheel ', 'offset': 0.0,
  // 'precision': 0.1, 'len': 15, 'name': 'SteerWheelAngle',
  // 'is_signed_var': False, 'physical_range': '[0|780]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': '\xa1\xe3'}
  double steerwheelangle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'steering wheel rotation speed',
  // 'offset': 0.0, 'precision': 0.1, 'len': 15, 'name': 'SteerWheelSpd',
  // 'is_signed_var': False, 'physical_range': '[0|1016]', 'bit': 39,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': '\xa1\xe3/s'}
  double steerwheelspd(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
