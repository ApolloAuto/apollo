/******************************************************************************
 * Copyright 2019 The Apollo Authors. All -Rights Reserved.
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

#include "modules/canbus/vehicle/zhongyun/protocol/vehicle_state_feedback_2_c4.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

Vehiclestatefeedback2c4::Vehiclestatefeedback2c4() {}
const int32_t Vehiclestatefeedback2c4::ID = 0xC4;

void Vehiclestatefeedback2c4::Parse(const std::uint8_t* bytes, int32_t length,
                                    ChassisDetail* chassis) const {
  chassis->mutable_zhongyun()
      ->mutable_vehicle_state_feedback_2_c4()
      ->set_motor_speed(motor_speed(bytes, length));
  chassis->mutable_zhongyun()
      ->mutable_vehicle_state_feedback_2_c4()
      ->set_driven_torque_feedback(driven_torque_feedback(bytes, length));
}

// config detail: {'name': 'motor_speed', 'offset': 0.0, 'precision': 1.0,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-3000|3000]', 'bit': 0,
// 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
int Vehiclestatefeedback2c4::motor_speed(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'name': 'driven_torque_feedback', 'offset': 0.0, 'precision':
// 0.05, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit':
// 16, 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
double Vehiclestatefeedback2c4::driven_torque_feedback(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.050000;
  return ret;
}
}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
