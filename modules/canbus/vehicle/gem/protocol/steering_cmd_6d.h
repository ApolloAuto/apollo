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

#ifndef MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_STEERING_CMD_6D_H_
#define MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_STEERING_CMD_6D_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Steeringcmd6d : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Steeringcmd6d();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'POSITION_VALUE', 'offset': 0.0, 'precision':
  // 0.001, 'len': 32, 'is_signed_var': True, 'physical_range':
  // '[-2147483.648|2147483.647]', 'bit': 7, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'radians'}
  Steeringcmd6d* set_position_value(double position_value);

  // config detail: {'name': 'SPEED_LIMIT', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit':
  // 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
  Steeringcmd6d* set_speed_limit(double speed_limit);

 private:
  // config detail: {'name': 'POSITION_VALUE', 'offset': 0.0, 'precision':
  // 0.001, 'len': 32, 'is_signed_var': True, 'physical_range':
  // '[-2147483.648|2147483.647]', 'bit': 7, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'radians'}
  void set_p_position_value(uint8_t* data, double position_value);

  // config detail: {'name': 'SPEED_LIMIT', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit':
  // 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad/s'}
  void set_p_speed_limit(uint8_t* data, double speed_limit);

 private:
  double position_value_;
  double speed_limit_;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_GEM_PROTOCOL_STEERING_CMD_6D_H_
