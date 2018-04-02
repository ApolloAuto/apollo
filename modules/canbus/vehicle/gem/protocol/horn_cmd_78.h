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

#ifndef MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_HORN_CMD_78_H_
#define MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_HORN_CMD_78_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Horncmd78 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Horncmd78();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'HORN_CMD', 'enum': {0: 'HORN_CMD_OFF', 1:
  // 'HORN_CMD_ON'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Horn_cmd_78::Horn_cmdType horn_cmd(const std::uint8_t* bytes,
                                     const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_GEM_PROTOCOL_HORN_CMD_78_H_
