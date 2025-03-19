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

#include "modules/canbus/vehicle/ch/protocol/brake_status__511.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

Brakestatus511::Brakestatus511() {}
const int32_t Brakestatus511::ID = 0x511;

void Brakestatus511::Parse(const std::uint8_t* bytes, int32_t length,
                           ChassisDetail* chassis) const {
  chassis->mutable_ch()->mutable_brake_status__511()->set_brake_pedal_en_sts(
      brake_pedal_en_sts(bytes, length));
  chassis->mutable_ch()->mutable_brake_status__511()->set_brake_pedal_sts(
      brake_pedal_sts(bytes, length));
  chassis->mutable_ch()->mutable_brake_status__511()->set_brake_err(
      brake_err(bytes, length));
  chassis->mutable_ch()->mutable_brake_status__511()->set_emergency_btn_env(
      emergency_btn_env(bytes, length));
  chassis->mutable_ch()->mutable_brake_status__511()->set_front_bump_env(
      front_bump_env(bytes, length));
  chassis->mutable_ch()->mutable_brake_status__511()->set_back_bump_env(
      back_bump_env(bytes, length));
  chassis->mutable_ch()->mutable_brake_status__511()->set_overspd_env(
      overspd_env(bytes, length));
  chassis->mutable_check_response()->set_is_esp_online(
      brake_pedal_en_sts(bytes, length) == 1);
}

// config detail: {'description': 'brake pedal enable bit(Status)', 'enum': {0:
// 'BRAKE_PEDAL_EN_STS_DISABLE', 1: 'BRAKE_PEDAL_EN_STS_ENABLE', 2:
// 'BRAKE_PEDAL_EN_STS_TAKEOVER'}, 'precision': 1.0, 'len': 8, 'name':
// 'brake_pedal_en_sts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Brake_status__511::Brake_pedal_en_stsType Brakestatus511::brake_pedal_en_sts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);
  Brake_status__511::Brake_pedal_en_stsType ret =
      static_cast<Brake_status__511::Brake_pedal_en_stsType>(x);
  return ret;
}

// config detail: {'description': 'Percentage of brake pedal(Status)', 'offset':
// 0.0, 'precision': 1.0, 'len': 8, 'name': 'brake_pedal_sts', 'is_signed_var':
// False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int', 'order':
// 'intel', 'physical_unit': '%'}
int Brakestatus511::brake_pedal_sts(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'brake_err', 'enum': {0: 'BRAKE_ERR_NOERR', 1:
// 'BRAKE_ERR_BRAKE_SYSTEM_ERR'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
Brake_status__511::Brake_errType Brakestatus511::brake_err(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Brake_status__511::Brake_errType ret =
      static_cast<Brake_status__511::Brake_errType>(x);
  return ret;
}

// config detail: {'name': 'emergency_btn_env', 'enum': {0:
// 'EMERGENCY_BTN_ENV_NOENV', 1: 'EMERGENCY_BTN_ENV_EMERGENCY_BUTTON_ENV'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Brake_status__511::Emergency_btn_envType Brakestatus511::emergency_btn_env(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Brake_status__511::Emergency_btn_envType ret =
      static_cast<Brake_status__511::Emergency_btn_envType>(x);
  return ret;
}

// config detail: {'name': 'front_bump_env', 'enum': {0: 'FRONT_BUMP_ENV_NOENV',
// 1: 'FRONT_BUMP_ENV_FRONT_BUMPER_ENV'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Brake_status__511::Front_bump_envType Brakestatus511::front_bump_env(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Brake_status__511::Front_bump_envType ret =
      static_cast<Brake_status__511::Front_bump_envType>(x);
  return ret;
}

// config detail: {'name': 'back_bump_env', 'enum': {0: 'BACK_BUMP_ENV_NOENV',
// 1: 'BACK_BUMP_ENV_BACK_BUMPER_ENV'}, 'precision': 1.0, 'len': 8,
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Brake_status__511::Back_bump_envType Brakestatus511::back_bump_env(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Brake_status__511::Back_bump_envType ret =
      static_cast<Brake_status__511::Back_bump_envType>(x);
  return ret;
}

// config detail: {'name': 'overspd_env', 'enum': {0: 'OVERSPD_ENV_NOENV', 1:
// 'OVERSPD_ENV_OVERSPEED_ENV'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
Brake_status__511::Overspd_envType Brakestatus511::overspd_env(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Brake_status__511::Overspd_envType ret =
      static_cast<Brake_status__511::Overspd_envType>(x);
  return ret;
}
}  // namespace ch
}  // namespace canbus
}  // namespace apollo
