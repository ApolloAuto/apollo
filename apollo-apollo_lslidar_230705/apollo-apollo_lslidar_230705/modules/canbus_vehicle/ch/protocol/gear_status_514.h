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

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Gearstatus514 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Gearstatus514();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'PRND control(Status)', 'enum':
  // {1: 'GEAR_STS_PARK', 2: 'GEAR_STS_REVERSE', 3: 'GEAR_STS_NEUTRAL', 4:
  // 'GEAR_STS_DRIVE'}, 'is_signed_var': False, 'len': 8, 'name': 'GEAR_STS',
  // 'offset': 0.0, 'order': 'intel', 'physical_range': '[1|4]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Gear_status_514::Gear_stsType gear_sts(const std::uint8_t* bytes,
                                         const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
