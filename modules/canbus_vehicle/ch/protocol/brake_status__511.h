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

class Brakestatus511 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Brakestatus511();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 48, 'enum': {0: 'OVERSPD_ENV_NOENV', 1:
  // 'OVERSPD_ENV_OVERSPEED_ENV'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'OVERSPD_ENV', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Brake_status__511::Overspd_envType overspd_env(const std::uint8_t* bytes,
                                                 const int32_t length) const;

  // config detail: {'bit': 0, 'description': 'brake pedal enable bit(Status)',
  // 'enum': {0: 'BRAKE_PEDAL_EN_STS_DISABLE', 1: 'BRAKE_PEDAL_EN_STS_ENABLE',
  // 2: 'BRAKE_PEDAL_EN_STS_TAKEOVER'}, 'is_signed_var': False, 'len': 8,
  // 'name': 'BRAKE_PEDAL_EN_STS', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Brake_status__511::Brake_pedal_en_stsType brake_pedal_en_sts(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'Percentage of brake
  // pedal(Status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'BRAKE_PEDAL_STS', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int brake_pedal_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'enum': {0: 'BRAKE_ERR_NOERR', 1:
  // 'BRAKE_ERR_BRAKE_SYSTEM_ERR'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'BRAKE_ERR', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Brake_status__511::Brake_errType brake_err(const std::uint8_t* bytes,
                                             const int32_t length) const;

  // config detail: {'bit': 24, 'enum': {0: 'EMERGENCY_BTN_ENV_NOENV', 1:
  // 'EMERGENCY_BTN_ENV_EMERGENCY_BUTTON_ENV'}, 'is_signed_var': False, 'len':
  // 8, 'name': 'EMERGENCY_BTN_ENV', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Brake_status__511::Emergency_btn_envType emergency_btn_env(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'enum': {0: 'FRONT_BUMP_ENV_NOENV', 1:
  // 'FRONT_BUMP_ENV_FRONT_BUMPER_ENV'}, 'is_signed_var': False, 'len': 8,
  // 'name': 'FRONT_BUMP_ENV', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Brake_status__511::Front_bump_envType front_bump_env(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'enum': {0: 'BACK_BUMP_ENV_NOENV', 1:
  // 'BACK_BUMP_ENV_BACK_BUMPER_ENV'}, 'is_signed_var': False, 'len': 8, 'name':
  // 'BACK_BUMP_ENV', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Brake_status__511::Back_bump_envType back_bump_env(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'enum': {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF',
  // 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}, 'is_signed_var': False, 'len': 1,
  // 'name': 'Brake_Light_Actual', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Brake_status__511::Brake_light_actualType brake_light_actual(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
