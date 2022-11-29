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

#include <string>

#include "modules/canbus_vehicle/devkit/proto/devkit.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

class Vinresp3516 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::Devkit> {
 public:
  static const int32_t ID;
  Vinresp3516();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Devkit* chassis) const override;

 private:
  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN16', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  std::string vin16(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
