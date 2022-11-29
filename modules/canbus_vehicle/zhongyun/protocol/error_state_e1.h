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

#include "modules/canbus_vehicle/zhongyun/proto/zhongyun.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Errorstatee1 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::Zhongyun> {
 public:
  static const int32_t ID;
  Errorstatee1();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Zhongyun* chassis) const override;

 private:
  // config detail: {'name': 'brake_error_code', 'enum': {0:
  // 'BRAKE_ERROR_CODE_NO_ERROR', 1: 'BRAKE_ERROR_CODE_ERROR'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': 'bit'}
  Error_state_e1::Brake_error_codeType brake_error_code(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Driven_error_code', 'enum': {0:
  // 'DRIVEN_ERROR_CODE_NO_ERROR', 1: 'DRIVEN_ERROR_CODE_ERROR'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': 'bit'}
  Error_state_e1::Driven_error_codeType driven_error_code(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'steering_error_code', 'enum': {0:
  // 'STEERING_ERROR_CODE_NO_ERROR', 1: 'STEERING_ERROR_CODE_ERROR'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': 'bit'}
  Error_state_e1::Steering_error_codeType steering_error_code(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Parking_error_code', 'enum': {0:
  // 'PARKING_ERROR_CODE_NO_ERROR', 1: 'PARKING_ERROR_CODE_ERROR'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': 'bit'}
  Error_state_e1::Parking_error_codeType parking_error_code(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Gear_error_msg', 'enum': {0:
  // 'GEAR_ERROR_MSG_NO_ERROR', 1: 'GEAR_ERROR_MSG_ERROR'}, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Error_state_e1::Gear_error_msgType gear_error_msg(const std::uint8_t* bytes,
                                                    const int32_t length) const;
};

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
