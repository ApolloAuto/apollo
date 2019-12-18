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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Brakestatus511 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Brakestatus511();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'brake pedal enable bit(Status)', 'enum':
  // {0: 'BRAKE_PEDAL_EN_STS_DISABLE', 1: 'BRAKE_PEDAL_EN_STS_ENABLE', 2:
  // 'BRAKE_PEDAL_EN_STS_TAKEOVER'}, 'precision': 1.0, 'len': 8, 'name':
  // 'BRAKE_PEDAL_EN_STS', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Brake_status__511::Brake_pedal_en_stsType brake_pedal_en_sts(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Percentage of brake pedal(Status)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'BRAKE_PEDAL_STS',
  // 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type':
  // 'int', 'order': 'intel', 'physical_unit': '%'}
  int brake_pedal_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BRAKE_ERR', 'enum': {0: 'BRAKE_ERR_NOERR', 1:
  // 'BRAKE_ERR_BRAKE_SYSTEM_ERR'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Brake_status__511::Brake_errType brake_err(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'name': 'EMERGENCY_BTN_ENV', 'enum': {0:
  // 'EMERGENCY_BTN_ENV_NOENV', 1: 'EMERGENCY_BTN_ENV_EMERGENCY_BUTTON_ENV'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Brake_status__511::Emergency_btn_envType emergency_btn_env(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'FRONT_BUMP_ENV', 'enum': {0:
  // 'FRONT_BUMP_ENV_NOENV', 1: 'FRONT_BUMP_ENV_FRONT_BUMPER_ENV'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Brake_status__511::Front_bump_envType front_bump_env(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BACK_BUMP_ENV', 'enum': {0: 'BACK_BUMP_ENV_NOENV',
  // 1: 'BACK_BUMP_ENV_BACK_BUMPER_ENV'}, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit':
  // 40, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Brake_status__511::Back_bump_envType back_bump_env(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'OVERSPD_ENV', 'enum': {0: 'OVERSPD_ENV_NOENV', 1:
  // 'OVERSPD_ENV_OVERSPEED_ENV'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Brake_status__511::Overspd_envType overspd_env(const std::uint8_t* bytes,
                                                 const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
