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

class Steerstatus512 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Steerstatus512();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'steering angle enable
  // bit(Status)', 'enum': {0: 'STEER_ANGLE_EN_STS_DISABLE', 1:
  // 'STEER_ANGLE_EN_STS_ENABLE', 2: 'STEER_ANGLE_EN_STS_TAKEOVER'},
  // 'is_signed_var': False, 'len': 8, 'name': 'STEER_ANGLE_EN_STS', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Steer_status__512::Steer_angle_en_stsType steer_angle_en_sts(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'Current steering angle(Status)',
  // 'is_signed_var': True, 'len': 16, 'name': 'STEER_ANGLE_STS', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[-0.524|0.524]', 'physical_unit':
  // 'radian', 'precision': 0.001, 'type': 'double'}
  double steer_angle_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'enum': {0: 'STEER_ERR_NOERR', 1:
  // 'STEER_ERR_STEER_MOTOR_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'STEER_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Steer_status__512::Steer_errType steer_err(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'bit': 32, 'enum': {0: 'SENSOR_ERR_NOERR', 1:
  // 'SENSOR_ERR_STEER_SENSOR_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'SENSOR_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Steer_status__512::Sensor_errType sensor_err(const std::uint8_t* bytes,
                                               const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
