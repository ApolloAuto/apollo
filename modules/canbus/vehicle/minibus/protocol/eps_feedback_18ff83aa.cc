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

#include "modules/canbus/vehicle/minibus/protocol/eps_feedback_18ff83aa.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Epsfeedback18ff83aa::Epsfeedback18ff83aa() {}
const int32_t Epsfeedback18ff83aa::ID = 0x38ff83aa;

void Epsfeedback18ff83aa::Parse(const std::uint8_t* bytes, int32_t length,
                                ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_eps_feedback_18ff83aa()
      ->set_eps_vcu_status(eps_vcu_status(bytes, length));
  chassis->mutable_minibus()
      ->mutable_eps_feedback_18ff83aa()
      ->set_eps_target_steering_velocity(
          eps_target_steering_velocity(bytes, length));
  chassis->mutable_minibus()
      ->mutable_eps_feedback_18ff83aa()
      ->set_eps_real_angle(eps_real_angle(bytes, length));
  chassis->mutable_minibus()
      ->mutable_eps_feedback_18ff83aa()
      ->set_eps_targit_angle(eps_targit_angle(bytes, length));
  chassis->mutable_minibus()
      ->mutable_eps_feedback_18ff83aa()
      ->set_eps_wheel_torque(eps_wheel_torque(bytes, length));
  chassis->mutable_check_response()->set_is_eps_online(
      eps_vcu_status(bytes, length) == 1);
}

// config detail: {'bit': 56, 'enum': {0: 'EPS_VCU_STATUS_READY', 1:
// 'EPS_VCU_STATUS_AUTO_DRIVE_MODE', 2: 'EPS_VCU_STATUS_SPEED_MODE', 3:
// 'EPS_VCU_STATUS_NULL', 4: 'EPS_VCU_STATUS_MANUAL_MODE', 5:
// 'EPS_VCU_STATUS_MANUAL_INTERVENTION_MODE', 6: 'EPS_VCU_STATUS_WARNING', 7:
// 'EPS_VCU_STATUS_ERROR'}, 'is_signed_var': False, 'len': 4, 'name':
// 'eps_vcu_status', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Eps_feedback_18ff83aa::Eps_vcu_statusType Epsfeedback18ff83aa::eps_vcu_status(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 4);

  Eps_feedback_18ff83aa::Eps_vcu_statusType ret =
      static_cast<Eps_feedback_18ff83aa::Eps_vcu_statusType>(x);
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'eps_target_steering_velocity', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 10.0, 'type':
// 'double'}
double Epsfeedback18ff83aa::eps_target_steering_velocity(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 10.000000;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name':
// 'eps_real_angle', 'offset': -870.0, 'order': 'motorola', 'physical_range':
// '[-870|870]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Epsfeedback18ff83aa::eps_real_angle(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -870.000000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name':
// 'eps_targit_angle', 'offset': -870.0, 'order': 'motorola', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Epsfeedback18ff83aa::eps_targit_angle(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -870.000000;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'eps_wheel_torque', 'offset': -8.96, 'order': 'intel', 'physical_range':
// '[0|8.96]', 'physical_unit': 'Nm', 'precision': 0.07, 'type': 'double'}
double Epsfeedback18ff83aa::eps_wheel_torque(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.070000 + -8.960000;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
