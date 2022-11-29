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

#include "modules/canbus_vehicle/zhongyun/protocol/error_state_e1.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::drivers::canbus::Byte;

Errorstatee1::Errorstatee1() {}
const int32_t Errorstatee1::ID = 0xE1;

void Errorstatee1::Parse(const std::uint8_t* bytes, int32_t length,
                         Zhongyun* chassis) const {
  chassis->mutable_error_state_e1()->set_brake_error_code(
      brake_error_code(bytes, length));
  chassis->mutable_error_state_e1()->set_driven_error_code(
      driven_error_code(bytes, length));
  chassis->mutable_error_state_e1()->set_steering_error_code(
      steering_error_code(bytes, length));
  chassis->mutable_error_state_e1()->set_parking_error_code(
      parking_error_code(bytes, length));
  chassis->mutable_error_state_e1()->set_gear_error_msg(
      gear_error_msg(bytes, length));
}

// config detail: {'name': 'brake_error_code', 'enum': {0:
// 'BRAKE_ERROR_CODE_NO_ERROR', 1: 'BRAKE_ERROR_CODE_ERROR'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 32, 'type': 'enum', 'order': 'intel', 'physical_unit': 'bit'}
Error_state_e1::Brake_error_codeType Errorstatee1::brake_error_code(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Error_state_e1::Brake_error_codeType ret =
      static_cast<Error_state_e1::Brake_error_codeType>(x);
  return ret;
}

// config detail: {'name': 'driven_error_code', 'enum': {0:
// 'DRIVEN_ERROR_CODE_NO_ERROR', 1: 'DRIVEN_ERROR_CODE_ERROR'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel',
// 'physical_unit': 'bit'}
Error_state_e1::Driven_error_codeType Errorstatee1::driven_error_code(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Error_state_e1::Driven_error_codeType ret =
      static_cast<Error_state_e1::Driven_error_codeType>(x);
  return ret;
}

// config detail: {'name': 'steering_error_code', 'enum': {0:
// 'STEERING_ERROR_CODE_NO_ERROR', 1: 'STEERING_ERROR_CODE_ERROR'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'intel',
// 'physical_unit': 'bit'}
Error_state_e1::Steering_error_codeType Errorstatee1::steering_error_code(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Error_state_e1::Steering_error_codeType ret =
      static_cast<Error_state_e1::Steering_error_codeType>(x);
  return ret;
}

// config detail: {'name': 'parking_error_code', 'enum': {0:
// 'PARKING_ERROR_CODE_NO_ERROR', 1: 'PARKING_ERROR_CODE_ERROR'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'intel',
// 'physical_unit': 'bit'}
Error_state_e1::Parking_error_codeType Errorstatee1::parking_error_code(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Error_state_e1::Parking_error_codeType ret =
      static_cast<Error_state_e1::Parking_error_codeType>(x);
  return ret;
}

// config detail: {'name': 'gear_error_msg', 'enum': {0:
// 'GEAR_ERROR_MSG_NO_ERROR', 1: 'GEAR_ERROR_MSG_ERROR'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Error_state_e1::Gear_error_msgType Errorstatee1::gear_error_msg(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Error_state_e1::Gear_error_msgType ret =
      static_cast<Error_state_e1::Gear_error_msgType>(x);
  return ret;
}
}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
