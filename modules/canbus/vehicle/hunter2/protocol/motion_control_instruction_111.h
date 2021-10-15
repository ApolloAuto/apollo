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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

class Motioncontrolinstruction111 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Motioncontrolinstruction111();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'steer_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  Motioncontrolinstruction111* set_steer_instruction(double steer_instruction);

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'speed_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  Motioncontrolinstruction111* set_speed_instruction(double speed_instruction);

 private:

  // config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'steer_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  void set_p_steer_instruction(uint8_t* data, double steer_instruction);

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'speed_instruction', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  void set_p_speed_instruction(uint8_t* data, double speed_instruction);

 private:
  double steer_instruction_;
  double speed_instruction_;
};

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo


